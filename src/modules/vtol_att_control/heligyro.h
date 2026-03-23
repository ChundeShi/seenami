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
 * @file heligyro.h
 *
 * Heligyro VTOL - Three-Option Takeoff Support
 *
 * DESIGN:
 * - Two modes: MC_MODE and FW_MODE
 * - Reports TRANSITION_TO_FW/MC for one cycle on mode change
 *   (required for Navigator mission compatibility)
 * - Always uses MC controller for attitude control
 * - Only rotor 0 thrust varies: full in MC_MODE, idle in FW_MODE
 *
 * TAKEOFF MODES (set by navigator via position_setpoint_triplet):
 * - param1=1, param2=0: vertical_takeoff only, MC controller
 * - param1=0, param2=1: runway_takeoff only, FW controller, rotor 0 idle
 * - param1=1, param2=1: Both takeoffs, FW controller + MC blending
 *
 * @author Modified from Standard/Tailsitter by heligyro developer
 */

#ifndef HELIGYRO_H
#define HELIGYRO_H

#include "vtol_type.h"
#include <uORB/Subscription.hpp>

class Heligyro : public VtolType
{
public:
	Heligyro(VtolAttitudeControl *_att_controller);
	~Heligyro() override = default;

	// Required overrides
	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override {}

private:
	// Mode enum - only MC and FW, no transitions
	enum class vtol_mode {
		MC_MODE = 0,
		FW_MODE
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};
	float _last_thr_in_mc{0.0f};

	// Takeoff flags - orthogonal, set independently by navigator
	// QGC can enable vertical_takeoff, runway_takeoff, or both
	bool _in_takeoff{false};
	bool _use_vertical_takeoff{false};	// Enable vertical takeoff (MC controller)
	bool _use_runway_takeoff{false};	// Enable runway takeoff (FW controller)

	// Parameters
	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
		(ParamFloat<px4::params::HG_FW_R0_THR>) _param_hg_fw_r0_thr
	)

	// Helper methods
	void parameters_update() override;
	void detect_takeoff_mode();
};

#endif // HELIGYRO_H
