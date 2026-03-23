// ============================================================================
// HELIGYRO VTOL - CUSTOM IMPLEMENTATION
// ============================================================================
// File: ActuatorEffectivenessHeligyroVTOL.cpp
// Purpose: Actuator effectiveness calculations for heligyro VTOL
// Key Methods:
//   - getEffectivenessMatrix(): 5-rotor 6DOF matrix computation
//   - updateSetpoint(): Differential thrust mixing
//   - computeRotor0Effectiveness(): Airspeed-based scaling
// Location: src/modules/control_allocator/VehicleActuatorEffectiveness/
// ============================================================================

/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessHeligyroVTOL.cpp
 *
 * Actuator effectiveness for Heligyro VTOL
 * Heligyro VTOL Configuration:
 * - 5-rotor 6DOF aircraft with hybrid lift/thrust design
 * - Rotor 0: upward-facing (axis 0,0,-1), unidirectional, passive auto-rotation
 * - Rotors 1-4: forward-facing (axis 1,0,0), reversible, provide forward/backward thrust
 * - Roll control: differential torque between CCW rotors (1,2) and CW rotors (3,4)
 * - Yaw control: differential thrust between left/right rotor pairs
 * - Pitch control: differential thrust between upper/lower rotor pairs
 * - Lateral control: thrust differential between left/right sides
 * - Forward/backward: all front rotors synchronized
 * - SINGLE UNIFIED FLIGHT MODE: No separate ROTARY_WING/FIXED_WING modes
 * - Autorotation: rotor 0 auto-rotates when motor speed < auto-rotation speed (physics-based)
 * - Parameters: CA_HELI_* for throttle curve, yaw scaling, roll torque, etc.
 * - Reverse thrust: rotors 1-4 reversible with efficiency ratios (KM/CT)
 * - Geometry: rotor positions defined in SDF model and airframe config
 *
 * KEY DESIGN PRINCIPLES:
 * - **NO SEPARATE FIXED_WING/ROTARY_WING MODES**: Physical auto-transition via differential
 *   rotor speeds. Flight control handles transition automatically through control allocation.
 * - **ROTOR 0 IS UNIDIRECTIONAL AND PASSIVE**: Auto-rotates when motor speed < auto-rotation
 *   speed; powered when motor speed > auto-rotation speed. This is physics-based, not mode-based.
 *
 * @author Modified from StandardVTOL by heligyro developer
 */

#include "ActuatorEffectivenessHeligyroVTOL.hpp"

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessHeligyroVTOL::ActuatorEffectivenessHeligyroVTOL(ModuleParams *parent)
	: ModuleParams(parent),
	  _rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, false)
{
	// Rotor 0: Upward facing, non-reversible (CW)
	// Rotors 1-4: Forward facing, reversible
	// Rotor 0: CW rotation
	// Rotor 1&2: CCW rotation
	// Rotor 3&4: CW rotation

	// Motor 0 is upward facing (only thrust in Z-axis)
	_upward_motor_mask = (1 << 0);

	// Motors 1-4 are forward facing (only thrust in X-axis)
	_forward_motor_mask = ((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4));

	// Initialize parameter handles for throttle curve
	updateParams();
}

void ActuatorEffectivenessHeligyroVTOL::updateParams()
{
	ModuleParams::updateParams();

	// Update throttle curve from params
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		param_t param_handle = param_find(buffer);

		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &_geometry.throttle_curve[i]);
		}
	}

	// Update geometry parameters
	param_t yaw_throttle_scale_hdl = param_find("CA_HELI_YAW_TH_S");
	param_get(yaw_throttle_scale_hdl, &_geometry.yaw_throttle_scale);

	param_t yaw_ccw_hdl = param_find("CA_HELI_YAW_CCW");
	int32_t yaw_ccw = 0;
	param_get(yaw_ccw_hdl, &yaw_ccw);
	_geometry.yaw_sign = (yaw_ccw == 1) ? -1.f : 1.f;

	// Use PX4 core param COM_SPOOLUP_TIME
	_geometry.spoolup_time = _param_com_spoolup_time.get();

	// Update autorotation threshold (PX4 core HG_AUTOROT_THR)
	updateAutorotationThreshold();
}

void ActuatorEffectivenessHeligyroVTOL::updateAutorotationThreshold()
{
	// Use override parameter if set, else use PX4 core autorotation airspeed
	if (_param_hg_autorot_thr.get() > 0.0f) {
		_autorotation_threshold = _param_hg_autorot_thr.get();
		return;
	}

	// Use PX4 core param (no MPC_ALT_MODE misalignment)
	param_t autorot_param = param_find("HG_AUTOROT_THR");
	if (autorot_param != PARAM_INVALID) {
		float value = 4.0f; // default value
		param_get(autorot_param, &value);
		_autorotation_threshold = value;
	} else {
		_autorotation_threshold = 4.0f; // fallback default
	}
}

bool ActuatorEffectivenessHeligyroVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Validate actuator limits to prevent out-of-bounds access
	if (configuration.num_actuators_matrix[0] > MAX_ACTUATORS_HELI ||
	    configuration.num_actuators_matrix[1] > MAX_ACTUATORS_HELI) {
		PX4_ERR("Actuator count exceeds Heligyro maximum");
		return false;
	}

	// UNIFIED FLIGHT MODE: all 5 rotors active, auto-transition via differential rotor speeds
	configuration.selected_matrix = 0;
	_rotors.enablePropellerTorqueNonUpwards(false);
	const bool rotors_added = _rotors.addActuators(configuration);

	_upward_motor_mask = _rotors.getUpwardsMotors();
	_forward_motor_mask = _rotors.getForwardsMotors();

	// Add roll axis effectiveness for differential rotor torque (Heligyro-specific)
	// Roll torque comes from differential thrust between CCW (rotors 1,2) and CW (rotors 3,4) rotors
	const int matrix_index = 0;
	const int num_actuators = configuration.num_actuators_matrix[matrix_index];

	for (int i = 0; i < num_actuators; i++) {
		// Only add roll effectiveness to forward-facing rotors (1-4)
		if (_forward_motor_mask & (1 << i)) {
			// Rotor 1 & 2: CCW rotation (positive KM) -> positive roll contribution
			// Rotor 3 & 4: CW rotation (negative KM) -> negative roll contribution
			float roll_effectiveness = 0.0f;

			if (i == 1 || i == 2) {
				// CCW rotors: positive roll effectiveness
				roll_effectiveness = _param_hg_roll_torque.get();

			} else if (i == 3 || i == 4) {
				// CW rotors: negative roll effectiveness
				roll_effectiveness = -_param_hg_roll_torque.get();
			}

			// Add roll axis to the effectiveness matrix
			// Note: We're adding to the roll axis (axis 0) of the effectiveness matrix
			configuration.effectiveness_matrices[matrix_index](0, i) = roll_effectiveness;
		}
	}

	return rotors_added;
}

void ActuatorEffectivenessHeligyroVTOL::stopMaskedMotorsWithZeroThrust(uint32_t motor_mask, ActuatorVector &actuator_sp)
{
	// Sanity check: actuator_sp size must match expected rotor count
	if (actuator_sp.size() < NUM_HELI_ROTORS) {
		PX4_WARN("Actuator vector too small for Heligyro rotors");
		return;
	}

	// Set masked motors to zero thrust
	for (int i = 0; i < NUM_HELI_ROTORS; ++i) {
		if (motor_mask & (1 << i)) {
			actuator_sp(i) = 0.0f;
		}
	}
}

void ActuatorEffectivenessHeligyroVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// Reset saturation flags
	_saturation_flags = {};

	// Sanity checks
	if (actuator_sp.size() < NUM_HELI_ROTORS ||
	    actuator_min.size() < NUM_HELI_ROTORS ||
	    actuator_max.size() < NUM_HELI_ROTORS) {
		PX4_ERR("Actuator vector size mismatch (expected %d rotors)", NUM_HELI_ROTORS);
		return;
	}

	// Get spoolup progress and RPM control correction
	const float spoolup_progress = throttleSpoolupProgress();
	float rpm_control_output = 0.0f;

	_rpm_control.setSpoolupProgress(spoolup_progress);
	rpm_control_output = _rpm_control.getActuatorCorrection();


	// UNIFIED FLIGHT MODE: all 5 rotors active for 6-DOF control
	// Rotor 0: Upward facing (Z-axis thrust only) - physics-based auto-rotation
	// Rotors 1-4: Forward facing (X-axis thrust) + yaw compensation

	// Throttle/collective pitch curve with RPM correction
	const float throttle = (math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.throttle_curve)
				+ rpm_control_output) * spoolup_progress;

	// Rotor 0: Upward thrust (unidirectional, passive auto-rotation)
	// Physics handles auto-rotation when motor speed < auto-rotation speed
	actuator_sp(0) = math::constrain(throttle, actuator_min(0), actuator_max(0));

	// Get rotor 0 RPM for yaw compensation (if available)
	float yaw_compensation = 0.0f;

	rpm_s rpm_data{};

	if (_rpm_sub.copy(&rpm_data)) {
		// Yaw compensation using main rotor torque (Heligyro-specific param)
		const float rotor0_rpm = rpm_data.rpm_estimate;
		const float main_rotor_torque = rotor0_rpm * rotor0_rpm * _param_hg_main_torque.get();
		yaw_compensation = main_rotor_torque * _param_hg_yaw_ff_rpm.get();
	}

	// Apply yaw compensation to rotors 1-4
	const float left_front_boost = yaw_compensation * 0.5f;
	const float right_front_dampen = yaw_compensation * 0.5f;

	// 6-DOF control for rotors 1-4:
	// - Yaw: Differential thrust between left/right rotors
	// - Pitch: Differential thrust between upper/lower rotors
	// - Roll: Differential thrust between CCW/CW rotors
	// - Forward/Backward: All rotors same direction
	// - Lateral: Thrust differential between left/right sides

	const float yaw_sp = control_sp(ControlAxis::YAW);
	const float pitch_sp = control_sp(ControlAxis::PITCH);
	const float roll_sp = control_sp(ControlAxis::ROLL);
	const float forward_sp = control_sp(ControlAxis::THRUST_X);
	const float lateral_sp = control_sp(ControlAxis::THRUST_Y);

	// Calculate lateral yaw coupling
	float lateral_yaw_coupling = 0.0f;
	if (fabs(lateral_sp) > 0.01f) {
		lateral_yaw_coupling = lateral_sp * _param_hg_lat_yaw_rat.get();
	}

	// Base forward/backward thrust (standard control allocation handles reverse)
	float rotor1_thrust = forward_sp;
	float rotor2_thrust = forward_sp;
	float rotor3_thrust = forward_sp;
	float rotor4_thrust = forward_sp;


	// Apply enhanced lateral control with rate limiting, deadzone, and feedforward
	if (fabs(lateral_sp) > 0.01f) {
		// Calculate delta time for rate limiting (simplified - use fixed small dt for now)
		// In a real implementation, we would track actual time between calls
		const float dt = 0.01f; // Approximate 100Hz update rate

		// Use enhanced lateral control with all new features
		float lateral_thrust = updateLateralControl(lateral_sp, dt);

		// Apply thrust differential
		if (lateral_sp > 0.0f) { // Left movement: increase left rotors, decrease right rotors
			rotor2_thrust += lateral_thrust;  // Left front upper (left)
			rotor3_thrust += lateral_thrust;  // Left front lower (left)
			rotor1_thrust -= lateral_thrust;  // Right front lower (right)
			rotor4_thrust -= lateral_thrust;  // Right front upper (right)
		} else { // Right movement: increase right rotors, decrease left rotors
			rotor1_thrust -= lateral_thrust;  // Right front lower (right) - note: lateral_thrust is negative
			rotor4_thrust -= lateral_thrust;  // Right front upper (right) - note: lateral_thrust is negative
			rotor2_thrust += lateral_thrust;  // Left front upper (left) - note: lateral_thrust is negative
			rotor3_thrust += lateral_thrust;  // Left front lower (left) - note: lateral_thrust is negative
		}
	}

	// Apply yaw control (differential thrust between left/right rotors)
	float yaw_control = yaw_sp + lateral_yaw_coupling;
	rotor1_thrust += yaw_control;  // Right front lower
	rotor4_thrust += yaw_control;  // Right front upper
	rotor2_thrust -= yaw_control;  // Left front upper
	rotor3_thrust -= yaw_control;  // Left front lower

	// Apply pitch control (differential thrust between upper/lower rotors)
	float pitch_control = pitch_sp;
	rotor2_thrust += pitch_control;  // Left front upper
	rotor4_thrust += pitch_control;  // Right front upper
	rotor1_thrust -= pitch_control;  // Right front lower
	rotor3_thrust -= pitch_control;  // Left front lower

	// Apply enhanced roll control with gain scheduling and yaw decoupling
	float scheduled_roll_gain = getScheduledRollGain();
	float roll_control = roll_sp * scheduled_roll_gain;

	// Apply roll-yaw decoupling compensation
	float yaw_decoupling = calculateRollYawDecoupling(roll_sp);

	rotor1_thrust += roll_control;  // Right front lower (CW)
	rotor2_thrust += roll_control;  // Left front upper (CCW)
	rotor3_thrust -= roll_control;  // Left front lower (CCW)
	rotor4_thrust -= roll_control;  // Right front upper (CW)

	// Apply yaw decoupling compensation
	rotor1_thrust += yaw_decoupling;  // Right front lower
	rotor4_thrust += yaw_decoupling;  // Right front upper
	rotor2_thrust -= yaw_decoupling;  // Left front upper
	rotor3_thrust -= yaw_decoupling;  // Left front lower

	// Apply yaw compensation from rotor 0
	rotor2_thrust += left_front_boost;   // Left front upper
	rotor3_thrust += left_front_boost;   // Left front lower
	rotor1_thrust -= right_front_dampen; // Right front lower
	rotor4_thrust -= right_front_dampen; // Right front upper

	// Constrain and set actuator outputs
	actuator_sp(1) = math::constrain(rotor1_thrust, actuator_min(1), actuator_max(1));
	actuator_sp(2) = math::constrain(rotor2_thrust, actuator_min(2), actuator_max(2));
	actuator_sp(3) = math::constrain(rotor3_thrust, actuator_min(3), actuator_max(3));
	actuator_sp(4) = math::constrain(rotor4_thrust, actuator_min(4), actuator_max(4));

	// Check for saturation
	for (int i = 0; i < NUM_HELI_ROTORS; ++i) {
		if (actuator_sp(i) >= actuator_max(i) - 0.01f) {
			setSaturationFlag(actuator_sp(i), _saturation_flags.thrust_pos, _saturation_flags.thrust_neg);
		} else if (actuator_sp(i) <= actuator_min(i) + 0.01f) {
			setSaturationFlag(actuator_sp(i), _saturation_flags.thrust_pos, _saturation_flags.thrust_neg);
		}
	}
}

void ActuatorEffectivenessHeligyroVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	// Simplified: No separate flight phases needed for unified flight mode
	// Rotor 0 behavior is physics-based (auto-rotates when motor speed < auto-rotation speed)
	// The control allocator handles all flight conditions with a single unified mode
}

void ActuatorEffectivenessHeligyroVTOL::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Simplified unallocated control reporting for unified flight mode
	status.unallocated_torque[0] = 0.0f;
	status.unallocated_torque[1] = 0.0f;
	status.unallocated_torque[2] = 0.0f;
	status.unallocated_thrust[0] = 0.0f;
	status.unallocated_thrust[1] = 0.0f;
	status.unallocated_thrust[2] = 0.0f;
}

void ActuatorEffectivenessHeligyroVTOL::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > 0.01f) {
		positive_flag = true;
	} else if (coeff < -0.01f) {
		negative_flag = true;
	}
}

float ActuatorEffectivenessHeligyroVTOL::updateLateralControl(float lateral_command, float dt)
{
	// Apply deadzone to filter small inputs
	const float deadzone = _param_hg_lat_deadzone.get();

	if (fabsf(lateral_command) < deadzone) {
		lateral_command = 0.0f;
	}

	// Select lateral control mode based on parameter
	const int lat_mode = _param_hg_lat_mode.get();
	float lateral_output = 0.0f;

	switch (lat_mode) {
	case 0:
	default:
		// Mode 0: Simple differential control (stable, default)
		lateral_output = simpleDifferentialControl(lateral_command);
		break;

	case 1:
		// Mode 1: Reserved for oscillation pattern (zig-zag)
		// Currently uses simple differential - zig-zag to be implemented later
		lateral_output = simpleDifferentialControl(lateral_command);
		break;

	case 2:
		// Mode 2: Adaptive control (hybrid approach)
		lateral_output = adaptiveLateralControl(lateral_command, dt);
		break;
	}

	// Apply rate limiting for smooth transitions
	const float rate_limit = _param_hg_lat_rate_lim.get();
	const float max_change = rate_limit * dt;
	const float delta = lateral_output - _lateral_rate_limited;

	if (fabsf(delta) > max_change) {
		_lateral_rate_limited += (delta > 0.0f) ? max_change : -max_change;

	} else {
		_lateral_rate_limited = lateral_output;
	}

	// Update state for next iteration
	_last_lateral_command = lateral_command;

	return _lateral_rate_limited;
}

float ActuatorEffectivenessHeligyroVTOL::simpleDifferentialControl(float lateral_command)
{
	// Simple differential control with damping and feedforward
	const float lateral_gain = _param_hg_lat_gain.get();
	const float lateral_max = _param_hg_lat_max.get();
	const float lateral_damp = _param_hg_lat_damp.get();
	const float lateral_ff = _param_hg_lat_ff.get();

	// Basic proportional control
	float lateral_thrust = lateral_command * lateral_gain;

	// Add feedforward for momentum compensation (improves response)
	float command_rate = lateral_command - _last_lateral_command;
	lateral_thrust += command_rate * lateral_ff;

	// Apply damping based on previous command (reduces oscillations)
	float damping = _last_lateral_command * lateral_damp;
	lateral_thrust -= damping;

	// Constrain to maximum lateral thrust
	lateral_thrust = math::constrain(lateral_thrust, -lateral_max, lateral_max);

	return lateral_thrust;
}

float ActuatorEffectivenessHeligyroVTOL::adaptiveLateralControl(float lateral_command, float dt)
{
	// Adaptive control: stronger response for sustained commands
	const float lateral_gain = _param_hg_lat_gain.get();
	const float lateral_max = _param_hg_lat_max.get();
	const float lateral_ff = _param_hg_lat_ff.get();

	// Calculate command magnitude for adaptive gain
	const float cmd_magnitude = fabsf(lateral_command);

	// Adaptive gain: increase gain for larger sustained commands
	float adaptive_gain = lateral_gain;

	if (cmd_magnitude > 0.3f) {
		// Boost gain for larger commands (up to 1.5x at full command)
		adaptive_gain *= 1.0f + 0.5f * (cmd_magnitude - 0.3f) / 0.7f;
	}

	// Apply adaptive gain
	float lateral_thrust = lateral_command * adaptive_gain;

	// Add feedforward for improved response
	float command_rate = lateral_command - _last_lateral_command;
	lateral_thrust += command_rate * lateral_ff;

	// Constrain to maximum
	lateral_thrust = math::constrain(lateral_thrust, -lateral_max, lateral_max);

	return lateral_thrust;
}

float ActuatorEffectivenessHeligyroVTOL::getScheduledRollGain()
{
	// Gain scheduling based on airspeed for roll control
	const float hover_gain = _param_hg_roll_hover.get();
	const float fwd_gain = _param_hg_roll_fwd.get();
	const float trans_speed = _param_hg_roll_tr_spd.get();

	// Get current airspeed from subscription
	airspeed_s airspeed_data{};

	if (_airspeed_sub.copy(&airspeed_data)) {
		_current_airspeed = airspeed_data.indicated_airspeed_m_s;
	}

	// Interpolate between hover and forward flight gains based on airspeed
	if (_current_airspeed < trans_speed * 0.5f) {
		// Below half transition speed: use full hover gain
		return hover_gain;

	} else if (_current_airspeed > trans_speed * 1.5f) {
		// Above 1.5x transition speed: use full forward flight gain
		return fwd_gain;

	} else {
		// Linear interpolation between hover and forward gains
		float blend = (_current_airspeed - trans_speed * 0.5f) / trans_speed;
		blend = math::constrain(blend, 0.0f, 1.0f);
		return hover_gain * (1.0f - blend) + fwd_gain * blend;
	}
}

float ActuatorEffectivenessHeligyroVTOL::calculateRollYawDecoupling(float roll_command)
{
	// Calculate yaw compensation to reduce roll-yaw coupling
	// The main rotor produces yaw torque when rolling due to asymmetric lift
	const float decoupling_gain = _param_hg_roll_yaw_dcp.get();

	// Calculate roll rate for rate-based compensation
	float roll_rate = roll_command - _last_roll_command;
	_last_roll_command = roll_command;

	// Yaw compensation: proportional to roll command with rate component
	float yaw_compensation = roll_command * decoupling_gain;
	yaw_compensation += roll_rate * decoupling_gain * 2.0f;  // Rate-based component (2x gain)

	return yaw_compensation;
}

float ActuatorEffectivenessHeligyroVTOL::throttleSpoolupProgress()
{
	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			if (!_armed) {
				_armed = true;
				_armed_time = hrt_absolute_time();
			}

			hrt_abstime now = hrt_absolute_time();
			float elapsed = (now - _armed_time) / 1e6f; // Convert to seconds

			if (elapsed < _geometry.spoolup_time) {
				return elapsed / _geometry.spoolup_time;
			} else {
				return 1.0f;
			}
		} else {
			_armed = false;
			_armed_time = 0;
		}
	}

	return 1.0f; // Default to fully spooled up
}

bool ActuatorEffectivenessHeligyroVTOL::mainMotorEngaged() const
{
	// Simplified: main motor is always engaged in unified flight mode
	// Rotor 0 auto-rotation is physics-based, not mode-based
	return true;
}
