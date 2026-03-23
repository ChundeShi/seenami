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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
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
 * Heligyro VTOL - Three-Option Takeoff Implementation
 *
 * DESIGN:
 * - Two modes: MC_MODE and FW_MODE
 * - Reports TRANSITION_TO_FW/MC for one cycle on mode change
 *   (required for Navigator mission compatibility)
 * - Always uses MC controller for attitude control
 * - Only rotor 0 thrust varies: full in MC_MODE, idle in FW_MODE
 *
 * TAKEOFF MODES (set by navigator via position_setpoint_triplet):
 * - vertical_takeoff only:     Vertical takeoff using MC controller only
 * - runway_takeoff only:       Runway takeoff using FW controller, rotor 0 idle
 * - Both takeoffs enabled:     Hybrid runway using FW + MC blended
 *
 * @author Modified from Standard/Tailsitter by heligyro developer
 */

#include "heligyro.h"
#include "vtol_att_control_main.h"

using namespace matrix;

Heligyro::Heligyro(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void Heligyro::parameters_update()
{
	VtolType::updateParams();
}

void Heligyro::detect_takeoff_mode()
{
	// Get position setpoint triplet from vtol_att_control
	position_setpoint_triplet_s *pos_sp_triplet = _attc->get_pos_sp_triplet();

	// Check if we're in takeoff mode based on position setpoint type
	_in_takeoff = (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF);

	if (_in_takeoff) {
		// For now, default to vertical takeoff during takeoff
		// TODO: Implement mechanism to pass vertical_takeoff/runway_takeoff flags
		// from navigator to VTOL (may need custom uORB message or extend position_setpoint)
		_use_vertical_takeoff = true;
		_use_runway_takeoff = false;
	} else {
		// Not in takeoff - clear flags
		_use_vertical_takeoff = false;
		_use_runway_takeoff = false;
	}
}

void Heligyro::update_vtol_state()
{
	// Detect takeoff mode from navigator setpoint
	detect_takeoff_mode();

	// Get requested mode from navigator/QGC
	bool fw_requested = _attc->is_fixed_wing_requested();

	// Handle mode switching with transition reporting
	if (fw_requested && _vtol_mode == vtol_mode::MC_MODE) {
		// Starting transition to FW
		_vtol_mode = vtol_mode::FW_MODE;
		// Report transition for one cycle (required by Navigator)
		_common_vtol_mode = mode::TRANSITION_TO_FW;

	} else if (!fw_requested && _vtol_mode == vtol_mode::FW_MODE) {
		// Starting transition to MC
		_vtol_mode = vtol_mode::MC_MODE;
		// Report transition for one cycle (required by Navigator)
		_common_vtol_mode = mode::TRANSITION_TO_MC;

	} else {
		// Steady state - report actual mode
		_common_vtol_mode = (_vtol_mode == vtol_mode::FW_MODE) ?
				    mode::FIXED_WING : mode::ROTARY_WING;
	}
}

void Heligyro::update_transition_state()
{
	// Heligyro has immediate transition, but we need to generate attitude setpoint
	// during the transition cycle for Navigator compatibility

	const hrt_abstime now = hrt_absolute_time();

	// Call base class to handle standard transition logic
	VtolType::update_transition_state();

	// For Heligyro, use MC attitude setpoint during transition
	// (since we always need MC controller for 6DOF attitude control)
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		return;
	}

	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
}

void Heligyro::update_mc_state()
{
	// Use base implementation - MC controller active
	VtolType::update_mc_state();
}

void Heligyro::update_fw_state()
{
	// Use base implementation - FW controller active
	// Note: We still call this for FW controller thrust calculations
	// But we override in fill_actuator_outputs to use MC attitude control
	VtolType::update_fw_state();
}

void Heligyro::fill_actuator_outputs()
{
	// Initialize timestamps
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;

	// ============================================================================
	// TAKEOFF MODE BLENDING
	// ============================================================================
	// During takeoff, navigator sets the method via position_setpoint_triplet
	// This determines how we blend MC and FW controller outputs
	// ============================================================================

	if (_in_takeoff) {
		// ============================================================================
		// ORTHOGONAL TAKEOFF MODE BLENDING
		// ============================================================================
		// vertical_takeoff and runway_takeoff are independent - blend based on flags
		// _use_vertical_takeoff: Use MC for attitude and/or vertical thrust
		// _use_runway_takeoff: Use FW for runway logic and forward thrust
		// ============================================================================

		// Attitude: Always use MC for 6DOF control (heligyro has no control surfaces)
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];

		// Forward thrust: Use FW if enabled, else MC
		if (_use_runway_takeoff) {
			_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
			_thrust_setpoint_0->xyz[1] = _vehicle_thrust_setpoint_virtual_fw->xyz[1];
		} else {
			_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_mc->xyz[0];
			_thrust_setpoint_0->xyz[1] = _vehicle_thrust_setpoint_virtual_mc->xyz[1];
		}

		// Vertical thrust (rotor 0): Use MC if enabled, else idle
		if (_use_vertical_takeoff) {
			_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];
			_last_thr_in_mc = _thrust_setpoint_0->xyz[2];
		} else {
			// Rotor 0 at idle (auto-rotation)
			float mc_thrust_z = _vehicle_thrust_setpoint_virtual_mc->xyz[2];
			_thrust_setpoint_0->xyz[2] = _param_hg_fw_r0_thr.get() * mc_thrust_z;
		}
	}

	// ============================================================================
	// NORMAL FLIGHT (Not in takeoff)
	// ============================================================================
	else {
		// ALWAYS use MC controller for attitude (torque)
		// Heligyro needs 6DOF control via differential thrust on rotors 1-4
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];

		// XY thrust: always from MC controller (forward/lateral via rotors 1-4)
		_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_mc->xyz[0];
		_thrust_setpoint_0->xyz[1] = _vehicle_thrust_setpoint_virtual_mc->xyz[1];

		// Z thrust (rotor 0): depends on mode
		if (_vtol_mode == vtol_mode::FW_MODE) {
			// FW mode: rotor 0 at idle (auto-rotation)
			// Use HG_FW_R0_THR parameter for idle thrust level
			float mc_thrust_z = _vehicle_thrust_setpoint_virtual_mc->xyz[2];
			_thrust_setpoint_0->xyz[2] = _param_hg_fw_r0_thr.get() * mc_thrust_z;
		} else {
			// MC mode: rotor 0 at full thrust
			_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];
			_last_thr_in_mc = _thrust_setpoint_0->xyz[2];
		}
	}

	// Setpoint 1 not used by Heligyro (all 5 rotors on setpoint 0)
	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		_torque_setpoint_1->xyz[i] = 0.f;
		_thrust_setpoint_1->xyz[i] = 0.f;
	}
}
