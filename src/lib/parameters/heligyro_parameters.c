/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
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
 * @file heligyro_params.c
 *
 * Heligyro unified takeoff parameters.
 *
 * These parameters replace the separate COM_SPOOLUP_TIME (VTOL) and
 * RWTO_RAMP_TIME (Runway) with a single unified set used by both
 * vertical and runway takeoff paths.
 *
 * @author PX4 Development Team
 */

/*
 * UNIFIED TAKEOFF TIMING PARAMETERS
 * Used by both vertical (Rotor 0) and runway (Rotors 1-4) paths
 */

/**
 * Heligyro spoolup time
 *
 * Time from arm to all rotors at idle speed.
 * Used by both vertical and runway paths during SPOOLUP state.
 * Replaces: COM_SPOOLUP_TIME for VTOL, RWTO_RAMP_TIME for runway
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(HG_SPOOLUP_TIME, 2.0f);

/**
 * Heligyro takeoff ramp time
 *
 * Time to ramp from idle to flight thrust after pilot input.
 * Used by both vertical and runway paths during RAMP state.
 * Takes the maximum of former VTOL and Runway values.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(HG_RAMP_TIME, 3.0f);

/*
 * THRUST SETPOINT PARAMETERS
 */

/**
 * Heligyro hover thrust
 *
 * Thrust required for rotor 0 to maintain hover.
 * Used as target during vertical takeoff RAMP state.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(HG_HOVER_THR, 0.60f);

/**
 * Heligyro cruise thrust
 *
 * Thrust for forward flight with rotors 1-4.
 * Used as target during runway takeoff RAMP state.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(HG_CRUISE_THR, 0.50f);

/*
 * ROTOR 0 THROTTLE CURVE PARAMETERS (existing, referenced here)
 * These define the non-linear response of the vertical rotor
 */

/**
 * Helicopter throttle curve - point 0 (min)
 *
 * Output throttle at 0% input stick.
 * Part of the 5-point throttle curve for rotor 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C0, 0.0f);

/**
 * Helicopter throttle curve - point 1 (low)
 *
 * Output throttle at 25% input stick.
 * Part of the 5-point throttle curve for rotor 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C1, 0.25f);

/**
 * Helicopter throttle curve - point 2 (mid)
 *
 * Output throttle at 50% input stick.
 * Part of the 5-point throttle curve for rotor 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C2, 0.50f);

/**
 * Helicopter throttle curve - point 3 (high)
 *
 * Output throttle at 75% input stick.
 * Part of the 5-point throttle curve for rotor 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C3, 0.75f);

/**
 * Helicopter throttle curve - point 4 (max)
 *
 * Output throttle at 100% input stick.
 * Part of the 5-point throttle curve for rotor 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Heligyro
 */
PARAM_DEFINE_FLOAT(CA_HELI_THR_C4, 1.0f);

/*
 * PARAMETER REPLACEMENT SUMMARY
 * =============================
 *
 * OLD PARAMETERS (Replaced):
 *   COM_SPOOLUP_TIME  - VTOL spoolup time
 *   MPC_TKO_RAMP      - VTOL takeoff ramp time
 *   RWTO_RAMP_TIME    - Runway throttle ramp time
 *
 * NEW PARAMETERS (Unified):
 *   HG_SPOOLUP_TIME   - Single spoolup for all 5 rotors
 *   HG_RAMP_TIME      - Single ramp time for both paths
 *
 * The new unified parameters take the MAXIMUM of the old values:
 *   HG_SPOOLUP_TIME = max(COM_SPOOLUP_TIME, RWTO_RAMP_TIME)
 *   HG_RAMP_TIME = max(MPC_TKO_RAMP, appropriate runway value)
 *
 * This ensures both paths have sufficient time for safe takeoff.
 */
