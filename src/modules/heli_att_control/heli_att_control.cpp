/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file heli_att_control.cpp
 * Helicopter attitude and rate controller with vbar.
 *
 * @author Bart Slinger		<bartslinger@gmail.com>
 *
 * Authors of mc_att_control_main.cpp from which some elements are re-used:
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 */

#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <systemlib/err.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <systemlib/param/param.h>

#include "heli_att_control.h"

#define MANUAL_THROTTLE_MAX_HELICOPTER	0.9f

HelicopterAttitudeControl::HelicopterAttitudeControl() :
	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_rates_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limits_sub(-1),
	_vehicle_land_detected_sub(-1),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "heli_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),

	_thrust_sp(0.0f)
{
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.yaw_auto_max = 0.0f;
	_params.auto_rate_max.zero();
	_params.mc_rate_max.zero();

	_rates_sp.zero();
	_att_control.zero();
	_integral.zero();
	_I.identity();

	_params_handles.roll_p		= param_find("HELI_ROLL_P");
	_params_handles.roll_rate_p	= param_find("HELI_ROLL_D");
	_params_handles.roll_hiller_gain	= param_find("HELI_ROLL_H");
	_params_handles.roll_effectiveness	= param_find("HELI_ROLL_E");

	_params_handles.pitch_p		= param_find("HELI_PITCH_P");
	_params_handles.pitch_rate_p	= param_find("HELI_PITCH_D");
	_params_handles.pitch_hiller_gain	= param_find("HELI_PITCH_H");
	_params_handles.pitch_effectiveness	= param_find("HELI_PITCH_E");

	_params_handles.hiller_decay	= param_find("VBAR_DECAY");

	_params_handles.yaw_p		= param_find("MC_YAW_P");
	_params_handles.yaw_rate_p	= param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i	= param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d	= param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	= param_find("MC_YAWRATE_FF");

	_params_handles.roll_rate_max	= param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= param_find("MC_ACRO_Y_MAX");

	/* fetch initial parameter values */
	parameters_update();
}

int
HelicopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&HelicopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
HelicopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
HelicopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
HelicopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
HelicopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
HelicopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
HelicopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
HelicopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
HelicopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

void
HelicopterAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
HelicopterAttitudeControl::control_attitude(float dt)
{
	(void) dt;
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	R_sp.set(_v_att_sp.R_body);

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* feed forward yaw setpoint rate */
	//_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

	/* weather-vane mode, dampen yaw rate */
	/*
	if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
	    _v_att_sp.disable_mc_yaw_control == true && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
	}*/
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
HelicopterAttitudeControl::control_attitude_rates(float dt)
{
	/* Keep track of previous rates for differentiation */
	static math::Vector<3> prev_rates = { 0.0f, 0.0f, 0.0f };

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	/* Propagate first order filter which has the vbar angles in body frame */
	float alpha = 1.0f - (dt / (_params.hiller_decay + dt)); // Hiller decay is time constant in seconds
	_integral(0) -= rates(0) * dt;
	_integral(0) *= alpha;
	_integral(1) -= rates(1) * dt;
	_integral(1) *= alpha;

	/* Calculate pitch and roll commands */
	_att_control(0) = _rates_sp(0) * _params.rate_effectiveness(0) - rates(0) * _params.rate_p(0) + _integral(0) * _params.hiller_gain(0);
	_att_control(1) = _rates_sp(1) * _params.rate_effectiveness(1) - rates(1) * _params.rate_p(1) + _integral(1) * _params.hiller_gain(1);

	/* Propagate yaw integrator */
	_integral(2) += rates_err(2) * _params.yawrate_i * dt;

	/* Calculate yaw command, similar to mc_att_control */
	_att_control(2) = _params.yawrate_p * rates_err(2) + _params.yawrate_d * (prev_rates(2) - rates(2)) / dt +
			_integral(2) + _params.yawrate_ff * _rates_sp(2);

	prev_rates = rates;
}

/**
 * @brief HelicopterAttitudeControl::task_main_trampoline
 * This function does not have a test!
 * @param argc
 * @param argv
 */
void
HelicopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	heli_att_control::g_control->task_main();
}

/**
 * @brief HelicopterAttitudeControl::task_main
 * The main task only has high-level functionality to make unit-testing easier.
 */
void HelicopterAttitudeControl::task_main()
{
	/* Subscribe to the required uORB topics */
	do_subscriptions();

	/* Initialize parameter cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	/* Loop function */
	while (!_task_should_exit)
	{
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* The actual control loop */
		if (fds[0].revents & POLLIN) {
			run_control_loop();
		}

		perf_end(_loop_perf);

	}

	_control_task = -1;
	return;
}

/**
 * @brief HelicopterAttitudeControl::do_subscriptions
 * Subscribe to all uORB channels.
 */
void HelicopterAttitudeControl::do_subscriptions()
{
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
}

void HelicopterAttitudeControl::parameters_update()
{
	float v;

	/* Roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_hiller_gain, &v);
	_params.hiller_gain(0) = v;
	param_get(_params_handles.roll_effectiveness, &v);
	_params.rate_effectiveness(0) = v;

	/* Pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_hiller_gain, &v);
	_params.hiller_gain(1) = v;
	param_get(_params_handles.pitch_effectiveness, &v);
	_params.rate_effectiveness(1) = v;

	/* VBar decay */
	param_get(_params_handles.hiller_decay, &v);
	_params.hiller_decay = v;

	/* Yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &_params.yawrate_p);
	param_get(_params_handles.yaw_rate_i, &_params.yawrate_i);
	param_get(_params_handles.yaw_rate_d, &_params.yawrate_d);
	param_get(_params_handles.yaw_rate_ff, &_params.yawrate_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);
}


void HelicopterAttitudeControl::run_control_loop()
{
	static uint64_t last_run = 0;
	float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* guard against too small (< 2ms) and too large (> 20ms) dt's */
	if (dt < 0.002f) {
		dt = 0.002f;

	} else if (dt > 0.02f) {
		dt = 0.02f;
	}

	/* copy attitude and control state topics */
	orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

	/* check for updates in other topics */
	parameter_update_poll();
	vehicle_control_mode_poll();
	arming_status_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_motor_limits_poll();
	vehicle_land_detected_poll();

	/* Run attitude controller */
	if (_v_control_mode.flag_control_attitude_enabled)
	{
		control_attitude(dt);

		/* publish attitude rates setpoint */
		_v_rates_sp.roll = _rates_sp(0);
		_v_rates_sp.pitch = _rates_sp(1);
		_v_rates_sp.yaw = _rates_sp(2);
		_v_rates_sp.thrust = _thrust_sp;
		_v_rates_sp.timestamp = hrt_absolute_time();

		if (_v_rates_sp_pub != nullptr) {
			orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

		} else if (_rates_sp_id) {
			_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
		}
	} else {
		/* attitude controller disabled, poll rates setpoint topic */
		if (_v_control_mode.flag_control_manual_enabled) {
			/* manual rates control - ACRO mode */
			_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
						    _manual_control_sp.r).emult(_params.acro_rate_max);
			_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_HELICOPTER);

			/* publish attitude rates setpoint */
			_v_rates_sp.roll = _rates_sp(0);
			_v_rates_sp.pitch = _rates_sp(1);
			_v_rates_sp.yaw = _rates_sp(2);
			_v_rates_sp.thrust = _thrust_sp;
			_v_rates_sp.timestamp = hrt_absolute_time();

			if (_v_rates_sp_pub != nullptr) {
				orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

			} else if (_rates_sp_id) {
				_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			vehicle_rates_setpoint_poll();
			_rates_sp(0) = _v_rates_sp.roll;
			_rates_sp(1) = _v_rates_sp.pitch;
			_rates_sp(2) = _v_rates_sp.yaw;
			_thrust_sp = _v_rates_sp.thrust;
		}
	}

	/* Run rate controller */
	if (_v_control_mode.flag_control_rates_enabled) {
		control_attitude_rates(dt);

		/* publish actuator controls */
		_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
		_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
		_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
		_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = _ctrl_state.timestamp;

		_controller_status.timestamp = hrt_absolute_time();

		if (!_actuators_0_circuit_breaker_enabled) {
			if (_actuators_0_pub != nullptr) {

				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
				perf_end(_controller_latency_perf);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}

		}

		/* publish controller status */
		if (_controller_status_pub != nullptr) {
			orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

		} else {
			_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
		}
	}
}
