// ============================================================================
// HELIGYRO VTOL - CUSTOM IMPLEMENTATION
// ============================================================================
// File: ActuatorEffectivenessHeligyroVTOL.hpp
// Purpose: Actuator effectiveness for 5-rotor 6DOF heligyro VTOL
// Unique Features:
//   - 5-rotor effectiveness matrix
//   - Differential torque roll control
//   - Forward thrust autorotation logic
//   - Reverse thrust support for rotors 1-4
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
 * @file ActuatorEffectivenessHeligyroVTOL.hpp
 *
 * Actuator effectiveness for Heligyro VTOL
 * Heligyro VTOL Actuator Effectiveness:
 * - Manages 5-rotor 6DOF hybrid aircraft with simplified unified flight mode
 * - Single matrix (unified flight): all 5 rotors active, auto-transition via differential rotor speeds
 * - Roll torque: CCW rotors (1,2) positive KM, CW rotors (3,4) negative KM
 * - Yaw compensation: uses main rotor torque from RPM feedback
 * - Lateral control: thrust differential between left/right rotor pairs
 * - Reverse thrust: reversible rotors 1-4 with efficiency ratios (RV_THR_CT_RAT, RV_KM_CT_RAT)
 * - Parameters: HG_* for Heligyro-specific tuning, CA_HELI_* for throttle curve
 * - Core PX4 params: COM_SPOOLUP_TIME, HG_AUTOROT_THR (autorotation threshold)
 *
 * KEY DESIGN PRINCIPLES:
 * - **NO SEPARATE FLIGHT MODES**: Physical auto-transition via differential rotor speeds.
 *   Flight control handles transition automatically through control allocation.
 * - **ROTOR 0 ADAPTIVE USAGE**: Rotor0 provides powered lift when needed for control authority,
 *   or reduces thrust for energy savings when conditions permit. This is handled by the
 *   control allocator based on flying speed, position/attitude needs, and lift requirements.
 * - **SINGLE UNIFIED FLIGHT MODE**: Eliminates mode switching complexity.
 *
 * ROTOR0 ADAPTIVE BEHAVIOR:
 * Rotor0 thrust is adjusted based on current needs:
 * - **Powered lift when needed**: For precision control, climb, or insufficient airspeed
 * - **Reduced thrust when appropriate**: For energy savings during stable forward flight
 * - **No separate AUTOGLIDE mode**: The control allocator continuously adjusts rotor0 thrust
 *   based on flying speed, position/attitude requirements, and lift needs
 *
 * @author Modified from StandardVTOL by heligyro developer
 *
 * Autorotation threshold uses PX4 core param HG_AUTOROT_THR (autorotation airspeed)
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "RpmControl.hpp"

#include <px4_platform_common/module_params.h>
#include <parameters/param.h>

#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/manual_control_switches.h>

class ActuatorEffectivenessHeligyroVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_CURVE_POINTS = 5;
	static constexpr int NUM_HELI_ROTORS = 5; // Explicit rotor count for Heligyro
	static constexpr int MAX_ACTUATORS_HELI = 8; // Safety bound for actuators

	struct Geometry {
		float throttle_curve[NUM_CURVE_POINTS];
		float yaw_throttle_scale;
		float yaw_sign;
		float spoolup_time;
	};

	ActuatorEffectivenessHeligyroVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessHeligyroVTOL() = default;

	/**
	 * @brief Get effectiveness matrix for control allocation
	 * @param configuration Control allocation config
	 * @param external_update Reason for update
	 * @return true if matrix updated successfully
	 */
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "Heligyro"; }

	int numMatrices() const override { return 1; } // Single unified flight mode

	/**
	 * @brief Get desired allocation method for each matrix
	 * @param allocation_method_out Output array of allocation methods
	 */
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 1, "expecting at least 1 matrix");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	/**
	 * @brief Get RPY normalization flag for each matrix
	 * @param normalize Output array of normalization flags
	 */
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	/**
	 * @brief Update actuator setpoints based on unified flight mode and control inputs
	 * @param control_sp Desired control setpoints (roll/pitch/yaw/thrust)
	 * @param matrix_index Active control matrix index (always 0 for unified flight mode)
	 * @param actuator_sp Output actuator setpoints
	 * @param actuator_min Minimum actuator limits
	 * @param actuator_max Maximum actuator limits
	 */
	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	/**
	 * @brief Set flight phase (simplified - no separate phases for unified flight mode)
	 * @param flight_phase Current flight phase (ignored for Heligyro)
	 */
	void setFlightPhase(const FlightPhase &flight_phase) override;

	/**
	 * @brief Get unallocated control for status reporting
	 * @param matrix_index Active matrix index (always 0 for unified flight mode)
	 * @param status Output control allocator status
	 */
	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

// Check if rotor 0 is in autorotation (physics-based, not mode-based)
	bool isAutorotationMode() const { return false; } // Always false - autorotation is physics-based

private:
// Parameter handles (mix of PX4 core + Heligyro-specific)
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_HELI_YAW_TH_S>) _param_hg_main_torque,
		(ParamFloat<px4::params::HG_YAW_FF_RPM>) _param_hg_yaw_ff_rpm,
		(ParamFloat<px4::params::HG_ROLL_COMP_FWD>) _param_hg_roll_comp_fwd,
		(ParamFloat<px4::params::HG_ROLL_TORQUE>) _param_hg_roll_torque,
		//(ParamFloat<px4::params::CA_HELI_MIN_RPM>) _param_vt_min_rpm,
		// HG_AUTOROT_THR is a core PX4 parameter, not defined in control_allocator module
		(ParamFloat<px4::params::HG_AUTOROT_THR>) _param_hg_autorot_thr,
		(ParamFloat<px4::params::HG_HELI_DEG_LIFT>) _param_hg_heli_deg_lift,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time, // PX4 core (spoolup time)
		// Original lateral control parameters
		(ParamFloat<px4::params::HG_LAT_GAIN>) _param_hg_lat_gain,
		(ParamFloat<px4::params::HG_LAT_MAX>) _param_hg_lat_max,
		(ParamFloat<px4::params::HG_LAT_DAMP>) _param_hg_lat_damp,
		(ParamFloat<px4::params::HG_LAT_YAW_RAT>) _param_hg_lat_yaw_rat,
		// Enhanced lateral control parameters (Phase 1)
		(ParamInt<px4::params::HG_LAT_MODE>) _param_hg_lat_mode,
		(ParamFloat<px4::params::HG_LAT_RATE_LIM>) _param_hg_lat_rate_lim,
		(ParamFloat<px4::params::HG_LAT_DEADZONE>) _param_hg_lat_deadzone,
		(ParamFloat<px4::params::HG_LAT_FF>) _param_hg_lat_ff,
		// Roll control gain scheduling parameters (Phase 1)
		(ParamFloat<px4::params::HG_ROLL_HOVER>) _param_hg_roll_hover,
		(ParamFloat<px4::params::HG_ROLL_FWD>) _param_hg_roll_fwd,
		(ParamFloat<px4::params::HG_ROLL_TR_SPD>) _param_hg_roll_tr_spd,
		(ParamFloat<px4::params::HG_ROLL_YAW_DCP>) _param_hg_roll_yaw_dcp
	)

	/**
	 * @brief Update autorotation threshold (use core PX4 param HG_AUTOROT_THR)
	 */
	void updateAutorotationThreshold();

	/**
	 * @brief Stop masked motors by setting their thrust to zero
	 * @param motor_mask Bitmask of motors to stop
	 * @param actuator_sp Actuator setpoints to modify
	 */
	void stopMaskedMotorsWithZeroThrust(uint32_t motor_mask, ActuatorVector &actuator_sp);

	/**
	 * @brief Set saturation flag based on actuator value
	 * @param coeff Scaling coefficient
	 * @param positive_flag Flag for positive saturation
	 * @param negative_flag Flag for negative saturation
	 */
	void setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag);

	void updateParams() override;
	float throttleSpoolupProgress();
	bool mainMotorEngaged() const;

	ActuatorEffectivenessRotors _rotors;

	uint32_t _upward_motor_mask{};  // Rotor 0 (upward facing, non-reversible)
	uint32_t _forward_motor_mask{}; // Rotors 1-4 (forward facing, reversible)

	uORB::Subscription _vehicle_odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	mutable uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};

	RpmControl _rpm_control {this};
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};

	// Rotor health monitoring (for failsafe)
	bool _rotor_healthy[NUM_HELI_ROTORS] {}; // Health status per rotor
	float _rotor_rpm[NUM_HELI_ROTORS] {};    // Last known RPM per rotor
	uint64_t _rotor_failure_time[NUM_HELI_ROTORS] {}; // Timestamp of failure detection
	bool _failsafe_triggered{false};         // Failsafe active flag
	int _failed_rotor_count{0};              // Count of failed rotors

	struct SaturationFlags {
		bool roll_pos{false};
		bool roll_neg{false};
		bool pitch_pos{false};
		bool pitch_neg{false};
		bool yaw_pos{false};
		bool yaw_neg{false};
		bool thrust_pos{false};
		bool thrust_neg{false};
	};

	SaturationFlags _saturation_flags{};

// Throttle spoolup state
	bool _armed{false};
	uint64_t _armed_time{0};
	mutable bool _main_motor_engaged{true};

// Autorotation threshold (uses PX4 core param HG_AUTOROT_THR)
	float _autorotation_threshold{4.0f};

// Lateral thrust differential control state (enhanced Phase 1)
	float _last_lateral_command{0.0f};
	float _lateral_rate_limited{0.0f};      // Rate-limited lateral command
	uint64_t _last_lateral_update_time{0};  // Timestamp for rate limiting

// Roll control state for gain scheduling
	float _current_airspeed{0.0f};          // Current airspeed for gain scheduling
	float _last_roll_command{0.0f};         // Previous roll command for decoupling

	/**
	 * @brief Update lateral thrust differential control with enhanced features
	 * @param lateral_command Lateral control command (-1 to 1, positive = left)
	 * @param dt Delta time since last update
	 * @return Lateral thrust differential value
	 */
	float updateLateralControl(float lateral_command, float dt);

	/**
	 * @brief Simple differential lateral control (mode 0)
	 */
	float simpleDifferentialControl(float lateral_command);

	/**
	 * @brief Adaptive lateral control (mode 2, hybrid)
	 */
	float adaptiveLateralControl(float lateral_command, float dt);

	/**
	 * @brief Get roll gain based on current airspeed (gain scheduling)
	 */
	float getScheduledRollGain();

	/**
	 * @brief Calculate roll-yaw decoupling compensation
	 */
	float calculateRollYawDecoupling(float roll_command);

	Geometry _geometry{};
};
