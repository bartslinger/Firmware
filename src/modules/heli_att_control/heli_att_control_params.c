/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file heli_att_control_params.c
 * Parameters for helicopter attitude controller.
 *
 * @author Bart Slinger <bartslinger@gmail.com>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_P, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_P, 0.0f);

/**
 * Roll damping gain
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_D, 0.0f);

/**
 * Pitch damping gain
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_D, 0.0f);

/**
 * Roll hiller gain
 *
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_H, 0.0f);

/**
 * Pitch hiller gain
 *
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_H, 0.0f);

/**
 * VBar decay
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VBAR_DECAY, 0.0f);

/**
 * Roll effectiveness
 *
 * How much roll rate (rad/s) can be expected per raw roll command
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_E, 0.0f);

/**
 * Pitch effectiveness
 *
 * How much pitch rate (rad/s) can be expected per raw pitch command
 *
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_E, 0.0f);
