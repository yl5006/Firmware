/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file camera_feedback.hpp
 *
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>

using matrix::Eulerf;
using matrix::Quatf;

typedef enum : int32_t {
	CAMERA_FEEDBACK_MODE_NONE = 0,
	CAMERA_FEEDBACK_MODE_TRIGGER,
	CAMERA_FEEDBACK_MODE_PWM
} camera_feedback_mode_t;

class CameraFeedback
{
public:
	/**
	 * Constructor
	 */
	CameraFeedback();

	/**
	 * Destructor, also kills task.
	 */
	~CameraFeedback();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int			start();

	/**
	 * Stop the task.
	 */
	void		stop();
	bool		get_log_time(struct tm *tt, bool boot_time);
	int 		create_log_dir(tm *tt);
private:

	bool		_task_should_exit;		/**< if true, task should exit */
	int			_main_task;				/**< handle for task */

	int			_trigger_sub;
	int			_gpos_sub;
	int			_att_sub;
	int32_t 	utc_offset{0};
	orb_advert_t	_capture_pub;

	param_t			_p_feedback;
	param_t			_log_utc_offset{PARAM_INVALID};
	camera_feedback_mode_t _camera_feedback_mode;
	char 		_log_dir[256] {};
	char 		time[100];
	char    	line[300];
	char    	camera_file[256];
	time_t 		utc_time_sec;
	void		task_main();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);

};
