/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file throwtofly.c
 *
 * Simple output test.
 * @ref Documentation https://pixhawk.org/dev/examples/write_output
 *
 * @author yaoling
 */

#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <float.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <systemlib/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>
static bool thread_should_exit = false; /**< throwtofly exit flag */
static bool thread_running = false; /**< throwtofly status flag */
static int throwtofly_task; /**< Handle of throwtofly task / thread */

#define CONSTANTS_ONE_G					9.80665f		// m/s^2

orb_advert_t _mavlink_log_pub;
__EXPORT int throwtofly_main(int argc, char *argv[]);

int throwtofly_thread_main(int argc, char *argv[]);
/*
 Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason) {
		warnx("%s\n", reason);
	}
	warnx("usage: throwtofly {start|stop|status} [-p <additional params>]\n\n");
}
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int throwtofly_main(int argc, char *argv[]) {
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("throwtofly already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		throwtofly_task = px4_task_spawn_cmd("throwtofly", SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT, 2400, throwtofly_thread_main,
				(argv) ? (char * const *) &argv[2] : (char * const *) NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int throwtofly_thread_main(int argc, char *argv[]) {

	int count = 0;
	hrt_abstime accel_timestamp = 0;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D

	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	/* subscribe */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	param_t sys_id_param = param_find("MAV_SYS_ID");
	param_t comp_id_param = param_find("MAV_COMP_ID");

	param_t acc_time_param = param_find("THROW_ACC_TIME");
	param_t throwpower_param = param_find("THROW_POWER");
	param_t throw_derection = param_find("THROW_DERECTION");
	int32_t sys_id;
	int32_t comp_id;

	float acc_time;
	float throwpower;
	int derection;
	if (param_get(sys_id_param, &sys_id)) {
		warnx("PRM SYSID");
	}

	if (param_get(comp_id_param, &comp_id)) {
		warnx("PRM CMPID");
	}

	if (param_get(acc_time_param, &acc_time)) {
		warnx("PRM THROW_ACC_TIME");
	}
    int timecount=(int)((double)acc_time/0.004);
	if (param_get(throwpower_param, &throwpower)) {
		warnx("PRM THROW_POWER");
	}
	if (param_get(throw_derection, &derection)) {
			warnx("PRM THROW_DERECTION");
		}

	printf("time=%.3f,power=%.3f\n",(double)acc_time,(double)throwpower);
	/* first parameters read at start up */
//	struct parameter_update_s param_update;
//	orb_copy(ORB_ID(parameter_update), parameter_update_sub,
//		 &param_update); /* read from param topic to clear updated flag */
//	/* first parameters update */
//	inav_parameters_update(&pos_inav_param_handles, &params);
	thread_running = true;
	float accel_magnitude=0;
	/* main loop */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		int ret = px4_poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&_mavlink_log_pub, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */
			/* vehicle attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

			bool updated;

			/* armed */
			orb_check(armed_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}

			/* sensor combined */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_timestamp_relative != accel_timestamp) {
					if (att.R_valid) {
						/* transform acceleration vector from body frame to NED frame */
						for (int i = 0; i < 3; i++) {
							acc[i] = 0.0f;

							//	for (int j = 0; j < 3; j++) {  for head acc
							for (int j = 0; j < 3; j++) {
								acc[i] += PX4_R(att.R, i, j)
										* sensor.accelerometer_m_s2[j];
							}
						}

						acc[2] += CONSTANTS_ONE_G;

					} else {
						memset(acc, 0, sizeof(acc));
					}

					accel_timestamp = sensor.accelerometer_timestamp_relative;
				}
			}
			switch(derection)
				{
				case 0:		accel_magnitude = sqrtf(acc[0] * acc[0] + acc[1] * acc[1]);
					break ;
				case 1:		accel_magnitude = fabs(acc[2]);
					break ;
				default :   accel_magnitude=0;
					break ;
				}
				if (accel_magnitude > throwpower*CONSTANTS_ONE_G) {
					count++;
				} else {
					count = 0;
				}
				if (count >= timecount && !armed.armed) {
					struct vehicle_command_s cmd;

					cmd.target_system = sys_id;
					cmd.target_component = comp_id;
					cmd.source_system = sys_id;
					cmd.source_component = comp_id;
					/* request arming */
					cmd.param1 = 1.0f;
					cmd.param2 = 0;
					cmd.param3 = 0;
					cmd.param4 = 0;
					cmd.param5 = 0;
					cmd.param6 = 0;
					cmd.param7 = 0;
					cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM;

					/* ask to confirm command */
					cmd.confirmation = 1;

					/* send command once */
					orb_advert_t pub = orb_advertise(ORB_ID(vehicle_command),
							&cmd);

					/* spin here until IO's state has propagated into the system */

					orb_publish(ORB_ID(vehicle_command), pub, &cmd);
					//		printf("acc=%.4f\n",(double)acc[0]);

				}
			}
	}
	return OK;
}
