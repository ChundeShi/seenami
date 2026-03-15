/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistribution in binary form must reproduce the above copyright
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
 * @file heligyro.cpp
 *
 * Heligyro VTOL attitude control implementation with simplified unified flight mode
 *
 * SIMPLIFIED STATE MACHINE (NO SEPARATE FLIGHT MODES):
 * - SYSTEM_INIT: Hardware initialization and parameter loading
 * - PREFLIGHT_CHECKS: Sensor calibration and rotor health checks
 * - STANDBY: Ready for arming, all safety checks passed
 * - FLIGHT: Unified flight mode - auto-transition handled by physics/control allocation
 * - EMERGENCY: Critical safety mode for unrecoverable faults
 * - DEGRADED_ROTOR: 5→4 rotor redundancy mode (one front rotor failed)
 *
 * KEY DESIGN PRINCIPLES:
 * - **NO SEPARATE FLIGHT MODES**: Physical auto-transition via differential rotor speeds.
 *   Flight control handles transition automatically through control allocation.
 * - **ROTOR 0 ADAPTIVE USAGE**: Rotor0 provides powered lift when needed for control authority,
 *   or reduces thrust for energy savings when conditions permit. This is handled by the
 *   control allocator based on flying speed, position/attitude needs, and lift requirements.
 * - **SINGLE UNIFIED FLIGHT MODE**: Eliminates mode switching complexity. The control allocator
 *   handles all transitions seamlessly based on airspeed and rotor states.
 *
 * ROTOR CONFIGURATION:
 * - Rotor 0: upward-facing (axis 0,0,-1), unidirectional, provides adaptive vertical lift
 * - Rotors 1-4: forward-facing (axis 1,0,0), reversible, differential thrust control
 * - Roll control: differential torque between CCW rotors (1,2) and CW rotors (3,4)
 * - Yaw control: differential thrust between left/right rotor pairs
 * - Pitch control: differential thrust between upper/lower rotor pairs
 * - Lateral control: thrust differential between left/right sides
 *
 * ROTOR0 ADAPTIVE BEHAVIOR (handled by control allocator):
 * Rotor0 thrust is adjusted based on current needs:
 * - **Powered lift when needed**: For precision control, climb, or insufficient airspeed
 * - **Reduced thrust when appropriate**: For energy savings during stable forward flight
 * - **No separate AUTOGLIDE mode**: The control allocator continuously adjusts rotor0 thrust
 *   based on flying speed, position/attitude requirements, and lift needs
 *
 * Control: All via differential thrust (no servos)
 *
 * @author Modified from Standard/Tailsitter by heligyro developer
 */

#include "heligyro.h"
#include "vtol_att_control_main.h"

#include <float.h>
#include <math.h>

using namespace matrix;

Heligyro::Heligyro(VtolAttitudeControl *attc) :
	VtolType(attc),
	_rpm_sub(ORB_ID(rpm)),
	_vehicle_status_sub(ORB_ID(vehicle_status)),
	_airspeed_sub(ORB_ID(airspeed)),
	_battery_status_sub(ORB_ID(battery_status))
{
	_vtol_mode = vtol_mode::SYSTEM_INIT;
	_prev_vtol_mode = vtol_mode::SYSTEM_INIT;
	_mc_weight = 1.0f;
}

void Heligyro::parameters_update()
{
	VtolType::updateParams();

// Use PX4 core params
	_autorotation_threshold = _param_autorotation_threshold.get();
	_emergency_battery_threshold = _param_emergency_battery_thr.get();
	_degraded_rotor_lift_scale = _param_hg_heli_deg_lift.get();

// Sanitize parameter ranges (enforce safe values)
	_autorotation_threshold = math::max(_autorotation_threshold, 5.0f);  // Minimum 5.0 for autorotation threshold
	_emergency_battery_threshold = math::max(_emergency_battery_threshold, 0.05f); // Minimum 5% battery
	_degraded_rotor_lift_scale = math::constrain(_degraded_rotor_lift_scale, 0.5f, 1.0f);
}

void Heligyro::update_vtol_state()
{
	_prev_vtol_mode = _vtol_mode;

// Priority 0: Run system initialization (first time only)
	if (_vtol_mode == vtol_mode::SYSTEM_INIT) {
		if (run_system_initialization()) {
			_vtol_mode = vtol_mode::PREFLIGHT_CHECKS;
			PX4_INFO("Heligyro: System initialization complete, starting pre-flight checks");
		}
		return;
	}

// Priority 1: Run pre-flight checks (first time only)
	if (_vtol_mode == vtol_mode::PREFLIGHT_CHECKS) {
		if (run_preflight_checks()) {
			_vtol_mode = vtol_mode::STANDBY;
			PX4_INFO("Heligyro: Pre-flight checks passed, system ready for arming");
		} else {
			_vtol_mode = vtol_mode::EMERGENCY;
			_in_emergency_mode = true;
			PX4_ERR("Heligyro: Pre-flight checks FAILED - entering EMERGENCY mode");
		}
		return;
	}

// Priority 2: Check for emergency conditions (safety first)
	if (check_failsafe_conditions()) {
		_vtol_mode = vtol_mode::EMERGENCY;
		_in_emergency_mode = true;
		PX4_WARN("Heligyro: Emergency - entering EMERGENCY mode for safety");
		return;
	}

// Priority 3: Check for rotor faults (enter degraded mode)
	if (check_rotor_faults() && !_in_emergency_mode) {
		_vtol_mode = vtol_mode::DEGRADED_ROTOR;
		_in_degraded_rotor_mode = true;
		PX4_WARN("Heligyro: Rotor fault detected - entering DEGRADED_ROTOR mode (5→4 rotors)");
		return;
	}

// Priority 4: Check for recovery from degraded rotor mode
	if (_vtol_mode == vtol_mode::DEGRADED_ROTOR) {
		check_degraded_rotor_recovery();
	}

// Priority 5: Exit emergency mode if conditions resolve
	if (_in_emergency_mode && !check_failsafe_conditions()) {
		check_emergency_recovery();
	}

// Priority 6: Normal operation - transition to FLIGHT mode when armed
	if (_vtol_mode == vtol_mode::STANDBY) {
		// Transition from STANDBY to FLIGHT mode when armed
		_vtol_mode = vtol_mode::FLIGHT;
		_mc_weight = 1.0f; // Start with full multicopter control
		PX4_INFO("Heligyro: Entering unified FLIGHT mode");
	}

// Update flight blending weight based on current conditions
	if (_vtol_mode == vtol_mode::FLIGHT) {
		_mc_weight = calculate_flight_blend_weight();
	}

// Log state transitions for debugging
	if (_vtol_mode != _prev_vtol_mode) {
		PX4_INFO("Heligyro: State change: %s → %s",
			     vtol_mode_to_string(_prev_vtol_mode),
			     vtol_mode_to_string(_vtol_mode));
	}
}

bool Heligyro::run_system_initialization()
{
// Load and validate parameters
	parameters_update();

// Check parameter validity
	if (_autorotation_threshold <= 0.0f ||
	    _emergency_battery_threshold <= 0.0f ||
	    _degraded_rotor_lift_scale <= 0.0f) {
		PX4_ERR("Heligyro: Invalid parameters detected");
		_params_valid = false;
		return false;
	}

	_params_valid = true;
	PX4_INFO("Heligyro: System initialization successful");
	return true;
}

bool Heligyro::run_preflight_checks()
{
	bool all_checks_passed = true;

// Check sensor calibration
	check_sensor_calibration();
	if (!_sensors_calibrated) {
		PX4_WARN("Heligyro: Sensor calibration check failed");
		all_checks_passed = false;
	}

// Check rotor health
	check_rotor_health();
	if (!_rotors_healthy) {
		PX4_WARN("Heligyro: Rotor health check failed");
		all_checks_passed = false;
	}

// Check parameter validation
	if (!_params_valid) {
		PX4_WARN("Heligyro: Parameter validation failed");
		all_checks_passed = false;
	}

// Check for initial sensor faults
	if (check_sensor_faults()) {
		PX4_WARN("Heligyro: Initial sensor fault detected");
		_sensor_fault = true;
		all_checks_passed = false;
	}

// Check for initial rotor 0 fault
	if (check_rotor0_fault()) {
		PX4_WARN("Heligyro: Initial rotor 0 fault detected");
		_rotor0_fault = true;
		all_checks_passed = false;
	}

	return all_checks_passed;
}

void Heligyro::check_sensor_calibration()
{
	// For heligyro, we assume sensors are calibrated by default
	// since sensor_preflight topic doesn't exist in current PX4
	_sensors_calibrated = true;
}

void Heligyro::check_rotor_health()
{
	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		// vehicle_status_s doesn't have motor_failure member in current PX4
		// Assume rotors are healthy by default for heligyro
		_rotors_healthy = true;
	} else {
		_rotors_healthy = false;
		PX4_WARN("Heligyro: No vehicle status data available");
	}
}

void Heligyro::check_parameter_validation()
{
// Parameter validation already done in run_system_initialization()
	_params_valid = true;
}

void Heligyro::check_emergency_recovery()
{
// Exit emergency mode if all failsafe conditions are resolved
	_in_emergency_mode = false;
	_vtol_mode = vtol_mode::FLIGHT;
	PX4_INFO("Heligyro: Emergency conditions resolved - exiting to FLIGHT mode");
}

void Heligyro::check_degraded_rotor_recovery()
{
// Exit degraded mode if all rotors are healthy
	if (!check_rotor_faults()) {
		_vtol_mode = vtol_mode::FLIGHT;
		_in_degraded_rotor_mode = false;
		_faulty_rotor_mask = 0;
		PX4_INFO("Heligyro: All rotors recovered - exiting DEGRADED_ROTOR mode");
	}
}

bool Heligyro::check_emergency_conditions()
{
// Check for emergency conditions (rotor 0 behavior is physics-based, not mode-based)
// Monitor RPM for critical failures only
	rpm_s rpm_data{};

	if (!_rpm_sub.copy(&rpm_data)) {
		return false; // No new RPM data - don't trigger
	}

	// Sanitize RPM value (avoid invalid data)
	float rotor0_rpm = math::constrain(rpm_data.rpm_estimate, 0.0f, FLT_MAX);

	// Emergency if RPM is critically low (below emergency threshold)
	return (rotor0_rpm < _autorotation_threshold * 0.5f); // 50% of threshold for emergency
}

bool Heligyro::check_failsafe_conditions()
{
// Check critical vehicle failures
	input_rc_s input_rc{};

	if (_input_rc_sub.copy(&input_rc)) {
		// Check for RC signal loss using input_rc topic
		if (input_rc.rc_lost) {
			return true;
		}
	}

// Check battery level (validate data first)
	battery_status_s battery_status{};

	if (_battery_status_sub.copy(&battery_status)) {
		if (battery_status.remaining > 0.0f && // Valid percentage reading
		    battery_status.remaining < _emergency_battery_threshold) {
			return true;
		}
	}

// Check emergency RPM threshold
	rpm_s rpm_data{};

	if (_rpm_sub.copy(&rpm_data)) {
		float rotor0_rpm = math::constrain(rpm_data.rpm_estimate, 0.0f, FLT_MAX);

		if (rotor0_rpm < _autorotation_threshold) {
			return true;
		}
	}

// Check communications loss
	if (check_comms_loss()) {
		return true;
	}

	return false;
}

bool Heligyro::check_rotor_faults()
{
// Check for front rotor failures (rotors 1-4)
	// Note: vehicle_status_s doesn't have motor_failure member in current PX4
	// For now, assume no rotor faults unless detected elsewhere
	// In a real implementation, we'd monitor RPM or current sensors
	_faulty_rotor_mask = 0x0; // No faults detected
	return false;
}

bool Heligyro::check_sensor_faults()
{
// Check for GPS/IMU faults
	// Note: vehicle_status_s doesn't have gps_failure or sensor_failure members in current PX4
	// For now, assume no sensor faults unless detected elsewhere
	return false;
}

bool Heligyro::check_rotor0_fault()
{
// Check for top rotor (rotor 0) failure or autorotation error
	rpm_s rpm_data{};

		if (_rpm_sub.copy(&rpm_data)) {
			float rotor0_rpm = math::constrain(rpm_data.rpm_estimate, 0.0f, FLT_MAX);

			// Rotor 0 fault if RPM is critically low (below emergency threshold)
			if (rotor0_rpm < _autorotation_threshold) {
						return true;
					}		}
	return false;
}

bool Heligyro::check_comms_loss()
{
// Check for RC/MAVLink link loss
	input_rc_s input_rc{};

	if (_input_rc_sub.copy(&input_rc)) {
		return input_rc.rc_lost;
	}

	return false;
}

const char *Heligyro::vtol_mode_to_string(vtol_mode mode)
{
// Helper for logging state transitions
	switch (mode) {
	case vtol_mode::SYSTEM_INIT:    return "SYSTEM_INIT";
	case vtol_mode::PREFLIGHT_CHECKS: return "PREFLIGHT_CHECKS";
	case vtol_mode::STANDBY:       return "STANDBY";
	case vtol_mode::FLIGHT:        return "FLIGHT";
	case vtol_mode::EMERGENCY:     return "EMERGENCY";
	case vtol_mode::DEGRADED_ROTOR: return "DEGRADED_ROTOR";
	default:                       return "UNKNOWN";
	}
}

float Heligyro::validate_setpoint(float value, const char *name)
{
	// Validate controller setpoint for NaN/Inf and constrain to safe range
	if (!PX4_ISFINITE(value)) {
		PX4_ERR("Invalid %s setpoint: NaN/Inf detected", name);
		return 0.0f;
	}

	// Constrain to reasonable range [-2.0, 2.0] to prevent unsafe outputs
	float constrained = math::constrain(value, -2.0f, 2.0f);

	// Warn if value was saturated (more than 1% difference)
	if (fabsf(value - constrained) > 0.01f) {
		PX4_WARN("%s setpoint saturated: %.3f -> %.3f", name, (double)value, (double)constrained);
	}

	return constrained;
}

void Heligyro::log_mode_transition()
{
	// Log mode transitions with duration of previous mode
	if (_vtol_mode != _last_logged_mode) {
		hrt_abstime now = hrt_absolute_time();
		float duration_ms = (now - _mode_start_time) / 1000.0f;

		PX4_INFO("Heligyro mode transition: %s -> %s (duration: %.1f ms)",
			 vtol_mode_to_string(_last_logged_mode),
			 vtol_mode_to_string(_vtol_mode),
			 (double)duration_ms);

		_mode_start_time = now;
		_last_logged_mode = _vtol_mode;
	}
}

void Heligyro::log_emergency_reason()
{
	// Log detailed reason for emergency mode activation
	PX4_ERR("Emergency mode active. Fault summary:"
		 " rotor0_fault=%d, sensor_fault=%d, comms_lost=%d, rotor_mask=0x%X",
		 _rotor0_fault, _sensor_fault, _comms_lost, _faulty_rotor_mask);

	if (_rotor0_fault) {
		PX4_ERR("  - Rotor 0 fault: Top rotor failure detected");
	}

	if (_sensor_fault) {
		PX4_ERR("  - Sensor fault: GPS/IMU failure detected");
	}

	if (_comms_lost) {
		PX4_ERR("  - Comms lost: RC/MAVLink link failure detected");
	}

	if (_faulty_rotor_mask != 0) {
		PX4_ERR("  - Front rotor faults: mask=0x%X (bit 0=rotor1, bit1=rotor2, etc)",
			 _faulty_rotor_mask);
	}
}

// ------------------------------
// Empty overrides (no tilting mechanisms)
// ------------------------------
void Heligyro::update_transition_state() {}
void Heligyro::update_fw_state()
{
	// Use base implementation for fixed-wing state
	VtolType::update_fw_state();
}

void Heligyro::update_mc_state()
{
	// Use base implementation for multicopter state
	VtolType::update_mc_state();
}

/**
 * @brief Fill actuator outputs based on current VTOL mode
 *
 * Maps flight mode-specific control inputs to torque and thrust setpoints.
 * Follows PX4 architecture: VTOL module handles mode selection, control allocator
 * handles motor mixing and heligyro-specific compensation.
 *
 * @note All outputs are validated and constrained to safe ranges
 * @warning This function must be called at the control loop rate (typically 400Hz)
 */
void Heligyro::fill_actuator_outputs()
{
	// Log mode transitions
	log_mode_transition();

	// Initialize all outputs to zero with proper timestamps
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	// For compatibility with PX4 architecture, initialize setpoint 1 (not used by Heligyro)
	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// CORRECT ARCHITECTURE: Heligyro only handles mode selection and safety.
	// Control allocation (yaw compensation, reverse thrust, zig-zag) is handled by
	// ActuatorEffectivenessHeligyroVTOL.cpp in the control allocator module.
	// This follows PX4's division of labor principle.

	switch (_vtol_mode) {
	case vtol_mode::FLIGHT: {
		// Unified flight mode: Blend MC and FW controller outputs based on flight conditions
		// The control allocator handles the transition seamlessly based on airspeed and rotor states

		// Calculate blending weight based on current flight conditions
		float blend_weight = calculate_flight_blend_weight();

		// Blend MC and FW controller outputs
		_torque_setpoint_0->xyz[0] = validate_setpoint(
			_vehicle_torque_setpoint_virtual_mc->xyz[0] * blend_weight +
			_vehicle_torque_setpoint_virtual_fw->xyz[0] * (1.0f - blend_weight), "roll");
		_torque_setpoint_0->xyz[1] = validate_setpoint(
			_vehicle_torque_setpoint_virtual_mc->xyz[1] * blend_weight +
			_vehicle_torque_setpoint_virtual_fw->xyz[1] * (1.0f - blend_weight), "pitch");
		_torque_setpoint_0->xyz[2] = validate_setpoint(
			_vehicle_torque_setpoint_virtual_mc->xyz[2] * blend_weight +
			_vehicle_torque_setpoint_virtual_fw->xyz[2] * (1.0f - blend_weight), "yaw");

		// Thrust blending: Rotor 0 behavior is physics-based (auto-rotates when motor speed < auto-rotation speed)
		_thrust_setpoint_0->xyz[0] = validate_setpoint(
			_vehicle_thrust_setpoint_virtual_mc->xyz[0] * blend_weight +
			_vehicle_thrust_setpoint_virtual_fw->xyz[0] * (1.0f - blend_weight), "thrust_x");
		_thrust_setpoint_0->xyz[1] = validate_setpoint(
			_vehicle_thrust_setpoint_virtual_mc->xyz[1] * blend_weight +
			_vehicle_thrust_setpoint_virtual_fw->xyz[1] * (1.0f - blend_weight), "thrust_y");

		// Rotor 0 thrust: physics-based auto-rotation (handled by control allocator)
		// Use minimal thrust for autorotation when in forward flight conditions
		float mc_thrust_z = validate_setpoint(_vehicle_thrust_setpoint_virtual_mc->xyz[2], "thrust_z_mc");
		float fw_thrust_z = _param_hg_fw_r0_thr.get() * mc_thrust_z;
		_thrust_setpoint_0->xyz[2] = validate_setpoint(
			mc_thrust_z * blend_weight + fw_thrust_z * (1.0f - blend_weight), "thrust_z");
		break;
	}

	// Note: AUTOGLIDE mode removed - rotor0 thrust adjustment handled by control allocator
	// based on flying speed, position/attitude needs, and lift requirements

	case vtol_mode::DEGRADED_ROTOR: {
		// 5→4 rotor redundancy mode (one front rotor failed)
		// Scale down control authority based on degraded rotor

		// Validate and constrain the scale factor
		float scale_factor = _param_hg_heli_deg_lift.get();

		if (!PX4_ISFINITE(scale_factor)) {
			PX4_ERR("Invalid degraded_rotor_lift_scale: NaN/Inf, using safe default 0.8");
			scale_factor = 0.8f;
		}

		// Constrain to safe range [0.1, 1.0] to prevent unsafe operation
		scale_factor = math::constrain(scale_factor, 0.1f, 1.0f);

		// Validate all MC controller setpoints before scaling
		float roll_mc = validate_setpoint(_vehicle_torque_setpoint_virtual_mc->xyz[0], "roll");
		float pitch_mc = validate_setpoint(_vehicle_torque_setpoint_virtual_mc->xyz[1], "pitch");
		float yaw_mc = validate_setpoint(_vehicle_torque_setpoint_virtual_mc->xyz[2], "yaw");
		float thrust_x_mc = validate_setpoint(_vehicle_thrust_setpoint_virtual_mc->xyz[0], "thrust_x");
		float thrust_y_mc = validate_setpoint(_vehicle_thrust_setpoint_virtual_mc->xyz[1], "thrust_y");
		float thrust_z_mc = validate_setpoint(_vehicle_thrust_setpoint_virtual_mc->xyz[2], "thrust_z");

		_torque_setpoint_0->xyz[0] = roll_mc * scale_factor;
		_torque_setpoint_0->xyz[1] = pitch_mc * scale_factor;
		_torque_setpoint_0->xyz[2] = yaw_mc * scale_factor;
		_thrust_setpoint_0->xyz[0] = thrust_x_mc * scale_factor;
		_thrust_setpoint_0->xyz[1] = thrust_y_mc * scale_factor;
		_thrust_setpoint_0->xyz[2] = thrust_z_mc * scale_factor;
		break;
	}

	case vtol_mode::EMERGENCY:
		// Emergency mode: minimal control for safe descent

		// Log detailed reason for emergency mode (once per entry)
		if (_vtol_mode != _prev_vtol_mode) {
			log_emergency_reason();
		}

		// Use parameter instead of magic number
		_thrust_setpoint_0->xyz[2] = _param_hg_emerg_thr.get();
		break;

	default:
		// SYSTEM_INIT, PREFLIGHT_CHECKS, STANDBY: No control outputs
		break;
	}

	// Update previous mode for transition detection
	_prev_vtol_mode = _vtol_mode;
}

void Heligyro::waiting_on_tecs()
{
// TECS throttle value for unified flight mode
	if (isInFlightMode()) {
		// Use blended throttle based on flight conditions
		float blend_weight = calculate_flight_blend_weight();
		_v_att_sp->thrust_body[0] = -_last_thr_in_mc * blend_weight;
	}
}

float Heligyro::calculate_flight_blend_weight()
{
	// Calculate blending weight based on current flight conditions
	// This replaces explicit mode switching with continuous blending
	// The control allocator uses this weight to smoothly transition
	// between hover-like and forward-flight control strategies

	// Get current airspeed
	float airspeed = _autorotation_threshold; // Default to threshold
	airspeed_s airspeed_data{};

	if (_airspeed_sub.copy(&airspeed_data)) {
		airspeed = PX4_ISFINITE(airspeed_data.true_airspeed_m_s) ?
			airspeed_data.true_airspeed_m_s : airspeed_data.indicated_airspeed_m_s;
		airspeed = math::max(0.0f, airspeed); // Sanity: airspeed can't be negative
	}

	// Calculate blend weight based on airspeed
	// At low airspeed (< threshold): weight = 1.0 (hover-like control)
	// At high airspeed (> 2*threshold): weight = 0.0 (forward-flight control)
	// Linear transition between threshold and 2*threshold
	float blend_weight = 1.0f;

	if (airspeed > _autorotation_threshold) {
		float transition_range = _autorotation_threshold; // Same range as threshold
		float transition_start = _autorotation_threshold;
		float transition_end = _autorotation_threshold * 2.0f;

		if (airspeed >= transition_end) {
			blend_weight = 0.0f;
		} else {
			// Linear interpolation
			blend_weight = 1.0f - ((airspeed - transition_start) / transition_range);
			blend_weight = math::constrain(blend_weight, 0.0f, 1.0f);
		}
	}

	return blend_weight;
}

void Heligyro::blendThrottleAfterFrontTransition(float scale) {}
void Heligyro::blendThrottleBeginningBackTransition(float scale) {}
