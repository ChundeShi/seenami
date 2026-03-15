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
* @file heligyro.h
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
* @author Modified from Standard/Tailsitter by heligyro developer
*/

#ifndef HELIGYRO_H
#define HELIGYRO_H

#include "vtol_type.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>

class Heligyro : public VtolType
{

public:
	Heligyro(VtolAttitudeControl *_att_controller);
	~Heligyro() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendThrottleBeginningBackTransition(float scale);

	// Mode checking methods
	bool isInFlightMode() const { return _vtol_mode == vtol_mode::FLIGHT; }
	bool isInEmergencyMode() const { return _in_emergency_mode; }
	bool isInDegradedRotorMode() const { return _in_degraded_rotor_mode; }

private:
	/**
	 * SIMPLIFIED STATE MACHINE (NO SEPARATE FLIGHT MODES):
	 * - FLIGHT: Unified flight mode - auto-transition via differential rotor speeds
	 *   Flight control handles transition automatically through control allocation
	 * - No ROTARY_WING/FIXED_WING/AUTOGLIDE modes - handled by physics and control allocator
	 */
	enum class vtol_mode {
		SYSTEM_INIT = 0,       /**< Hardware initialization and parameter loading */
		PREFLIGHT_CHECKS,      /**< Sensor calibration and rotor health checks */
		STANDBY,               /**< Ready for arming, all safety checks passed */
		FLIGHT,                /**< Unified flight mode - auto-transition via control allocation */
		EMERGENCY,             /**< Critical safety mode for unrecoverable faults */
		DEGRADED_ROTOR         /**< 5→4 rotor redundancy mode */
	};

	// Core state tracking
	vtol_mode _vtol_mode{vtol_mode::SYSTEM_INIT};
	vtol_mode _prev_vtol_mode{vtol_mode::SYSTEM_INIT};

	// Safety mode flags
	bool _in_emergency_mode{false};
	bool _in_degraded_rotor_mode{false};

	// Pre-flight check flags
	bool _sensors_calibrated{false};
	bool _rotors_healthy{false};
	bool _params_valid{false};

	// Fault tracking
	uint8_t _faulty_rotor_mask{0};    // Bitmask of faulty rotors (0=none, 1-4=front rotors)
	bool _sensor_fault{false};         // GPS/IMU fault flag
	bool _rotor0_fault{false};        // Top rotor fault flag
	bool _comms_lost{false};           // RC/MAVLink link lost flag

	// Mode weights (used for control blending based on airspeed)
	float _mc_weight{1.0f};           // Blending weight based on flight conditions
	float _last_thr_in_mc{0.0f};

	// Subscriptions
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	// Parameter storage
	float _autorotation_threshold{5.0f}; // Airspeed threshold for rotor 0 auto-rotation behavior
	float _emergency_battery_threshold{0.12f}; // 12% battery level threshold
	float _degraded_rotor_lift_scale{0.8f}; // Lift scaling for 4-rotor mode

	// Mode tracking for logging
	hrt_abstime _mode_start_time{0};
	vtol_mode _last_logged_mode{vtol_mode::SYSTEM_INIT};

	// Parameter handles
	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
		(ParamFloat<px4::params::HG_AUTOROT_THR>) _param_autorotation_threshold,
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_emergency_battery_thr,
		(ParamFloat<px4::params::HG_HELI_DEG_LIFT>) _param_hg_heli_deg_lift,
		(ParamFloat<px4::params::HG_FW_R0_THR>) _param_hg_fw_r0_thr,
		// Note: HG_AUTOGLIDE_THR parameter removed - rotor0 thrust adjustment handled by control allocator
		(ParamFloat<px4::params::HG_EMERG_THR>) _param_hg_emerg_thr
	)

	// Helper methods
	void parameters_update() override;
	const char *vtol_mode_to_string(vtol_mode mode);
	float validate_setpoint(float value, const char *name);
	void log_mode_transition();
	void log_emergency_reason();

	// Pre-flight checks
	bool run_system_initialization();
	bool run_preflight_checks();
	void check_sensor_calibration();
	void check_rotor_health();
	void check_parameter_validation();

	// Safety checks (monitoring only - rotor 0 behavior is physics-based)
	bool check_emergency_conditions();
	bool check_failsafe_conditions();

	// Fault detection
	bool check_rotor_faults();
	bool check_sensor_faults();
	bool check_rotor0_fault();
	bool check_comms_loss();

	// Recovery logic
	void check_emergency_recovery();
	void check_degraded_rotor_recovery();

	/**
	 * Calculate MC/FW blending weight based on current flight conditions.
	 * This replaces explicit mode switching with continuous blending.
	 * The control allocator uses this weight to smoothly transition
	 * between hover-like and forward-flight control strategies.
	 */
	float calculate_flight_blend_weight();
};

#endif // HELIGYRO_H
