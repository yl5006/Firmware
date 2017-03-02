/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.cpp
 *
 * Implementation of accelerometer calibration.
 *
 * Transform acceleration vector to true orientation, scale and offset
 *

 * @author Du Yong <duy1102002@gmail.com>
 */


#include "commander_motor_test.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "../../drivers/drv_pwm_output.h"

#include <px4_posix.h>
#include <px4_time.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
//#include <sys/prctl.h>
#include <math.h>
#include <poll.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <geo/geo.h>
#include <conversion/rotation.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_attitude.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;
/*
209	MAV_CMD_DO_MOTOR_TEST	Mission command to perform motor test
Mission Param #1	motor sequence number (a number from 1 to max number of motors on the vehicle)
Mission Param #2	throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
Mission Param #3	throttle
Mission Param #4	timeout (in seconds)
Mission Param #5	Empty
Mission Param #6	Empty
Mission Param #7	Empty

*/
uint32_t adapt_pwm_rate(uint32_t pwm_type,uint32_t pwm_rate) {
	switch(pwm_type) {
	case 0:
		return (uint32_t)(PWM_MOTOR_OFF + pwm_rate * 0.01f * (PWM_DEFAULT_MAX - PWM_MOTOR_OFF));
		break;
	case 1:
	case 2:
		return pwm_rate;
		break;
	default:
		return pwm_rate;
	}


}

int do_commander_motor_test(struct vehicle_command_s cmd,orb_advert_t *mavlink_log_pub) {
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	uint32_t set_mask = 0;

	unsigned single_ch = cmd.param1;
	/* open for ioctl only */
	int fd = px4_open(dev, 0);

	int ret  = ioctl(fd, PWM_SERVO_ARM, 0);

	if (ret != OK) {
				err(1, "PWM_SERVO_ARM");
			}

	if (fd < 0) {
		calibration_log_info(mavlink_log_pub,"can't open %s", dev);
		return ERROR;
	}
	set_mask |= 1 << (single_ch - 1);
	unsigned pwm_value = adapt_pwm_rate(cmd.param2,cmd.param3);

	/* get the number of servo channels */
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		calibration_log_info(mavlink_log_pub, "PWM_SERVO_GET_COUNT");
		px4_close(fd);
		return ERROR;
	}
	for (unsigned i = 0; i < servo_count; i++) {
		if (set_mask & 1 << i) {
			ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);
//				calibration_log_info(mavlink_log_pub, "COUNT %d PWM_SERVO_SET(%d,%d)",servo_count,i,pwm_value);			
			if (ret != OK) {
				calibration_log_info(mavlink_log_pub, "PWM_SERVO_SET(%d,%d)",i,pwm_value);
				px4_close(fd);
				return ERROR;

			}
		}
	}
	px4_close(fd);
	return OK;
}






