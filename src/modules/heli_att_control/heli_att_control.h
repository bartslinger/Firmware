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
 * @file heli_att_control.h
 * Helicopter attitude and rate controller with vbar.
 *
 * @author Bart Slinger		<bartslinger@gmail.com>
 *
 * TODO Explanation how it works.
 */

#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <systemlib/perf_counter.h>
#include <lib/mathlib/mathlib.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
class HelicopterAttitudeControl {
public:
	/**
	 * Constructor
	 */
	HelicopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	virtual ~HelicopterAttitudeControl(){}

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Check for landing detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
	void		do_subscriptions();
	void		parameters_update();
	void		configure_wakeup_source(int wake_source_subscription);
	void		run_control_loop();

private:
	bool		_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;		/**< manual control setpoint subscription */
	int		_armed_sub;			/**< arming status subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_motor_limits_sub;		/**< motor limits subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */


	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;		/**< controller status publication */

	orb_id_t	_rates_sp_id;			/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t	_actuators_id;			/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct control_state_s			_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s		_actuators;		/**< actuator controls */
	struct actuator_armed_s			_armed;			/**< actuator arming status */
	struct vehicle_status_s			_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status;	/**< controller status */
	struct vehicle_land_detected_s		_vehicle_land_detected;	/**< vehicle land detected */

	perf_counter_t				_loop_perf;		/**< loop performance counter */
	perf_counter_t				_controller_latency_perf;

	math::Vector<3>				_rates_sp;		/**< angular rates setpoint */
	float					_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>				_att_control;		/**< attitude control vector */
	math::Matrix<3, 3>			_I;			/**< identity matrix */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_hiller_gain;
		param_t roll_effectiveness;

		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_hiller_gain;
		param_t pitch_effectiveness;

		param_t hiller_decay;

		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;

		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
	}					_params_handles;	/**< handles for interesting parameters */


	struct {
		math::Vector<3>			att_p;			/**< P gain on attitude error */
		math::Vector<2>			rate_p;			/**< P gain for angular rate error */
		math::Vector<2>			hiller_gain;		/**< Hiller gains */
		math::Vector<2>			rate_effectiveness;	/**< Effectiveness of commands on pitch/roll rates */
		float				hiller_decay;		/**< Time constant of virtual flybar */

		float				yawrate_p;
		float				yawrate_i;
		float				yawrate_d;
		float				yawrate_ff;

		float				roll_rate_max;
		float				pitch_rate_max;
		float				yaw_rate_max;
		float				yaw_auto_max;
		math::Vector<3>			acro_rate_max;		/**< max attitude rates in acro mode */
		math::Vector<3>			mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3>			auto_rate_max;		/**< attitude rate limits in auto modes */
	}					_params;
};

namespace heli_att_control
{

extern HelicopterAttitudeControl	*g_control;
}
