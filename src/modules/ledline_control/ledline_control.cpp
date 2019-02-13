/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file ledline_control.cpp
 *
 * led line control
 *
 * @author yaoling <165577564@qq.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_workqueue.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <board_config.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

extern "C" __EXPORT int ledline_control_main(int argc, char *argv[]);



#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

class LedLineControl
{
public:
	/**
	 * Constructor
	 */
	LedLineControl();

	/**
	 * Destructor, also kills task.
	 */
	~LedLineControl();

	/**
	 * Run intervalometer update
	 */
	void		update_intervalometer();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * test 	status.
	 */
	void		test();


private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;

	static struct work_s	_work;
	float			_activation_time;
	float			_interval;
	int 			_mode;
	int  			_loop;
	bool 			_intervalcall;

	int			_command_sub;

	orb_advert_t		_cmd_ack_pub;
	/**
	 * Vehicle command handler
	 */
	static void	cycle_trampoline(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);
};

struct work_s LedLineControl::_work;

typedef enum LED_LINE
{
	LINE_A=0,   /*  | */
	LINE_B=1,   /*  | */
	LINE_AB=2,  /*  | */
	LINE_A_B=3, /*  | */
} LED_LINE;

namespace ledline_control
{
LedLineControl	*g_ledline_control;
}

LedLineControl::LedLineControl() :
	_engagecall {},
	_disengagecall {},
	_activation_time(200.0f /* ms */),
	_interval(1000.0f /* ms */),
	_mode(LINE_AB),
	_loop(0),
	_intervalcall(false),
	_command_sub(-1),
	_cmd_ack_pub(nullptr)
{

	memset(&_work, 0, sizeof(_work));
}

LedLineControl::~LedLineControl()
{
	ledline_control::g_ledline_control = nullptr;
}

void
LedLineControl::update_intervalometer()
{

	// the actual intervalometer runs in interrupt context, so we only need to call
	// control_intervalometer once on enabling/disabling trigger to schedule the calls.
		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000),
			       (hrt_callout)&LedLineControl::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&LedLineControl::disengage, this);

}

void
LedLineControl::start()
{
	// start to monitor at high rate for trigger enable command
	work_queue(LPWORK, &_work, (worker_t)&LedLineControl::cycle_trampoline, this, USEC2TICK(1));

}

void
LedLineControl::stop()
{
	work_cancel(LPWORK, &_work);
	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);

	if (ledline_control::g_ledline_control != nullptr) {
		delete (ledline_control::g_ledline_control);
	}
}
void
LedLineControl::test()
{
	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.param1 = 1.0;
	vcmd.command = vehicle_command_s::VEHICLE_CMD_SET_LED_LINE_STATUS;

	orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);
}
void
LedLineControl::cycle_trampoline(void *arg)
{

	LedLineControl *trig = reinterpret_cast<LedLineControl *>(arg);

	// default loop polling interval
	int poll_interval_usec = 50000;

	if (trig->_command_sub < 0) {
		trig->_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated = false;
	orb_check(trig->_command_sub, &updated);

	struct vehicle_command_s cmd;


	// Command handling
	if (updated) {

		orb_copy(ORB_ID(vehicle_command), trig->_command_sub, &cmd);

		if (cmd.command == vehicle_command_s::VEHICLE_CMD_SET_LED_LINE_STATUS) {

			int controlcmd = commandParamToInt(cmd.param1);
			if(trig->_intervalcall)
			{
				hrt_cancel(&trig->_engagecall);
				hrt_cancel(&trig->_disengagecall);
				trig->_intervalcall = false;
			}
			switch(controlcmd)
			{
			case LINE_A:
				trig->_mode = LINE_A;
				if(cmd.param2 > 0)
				{
					trig->_interval = cmd.param2 * 1000;
					trig->_activation_time = cmd.param4 * 1000;
					if(trig->_activation_time < 1.0f)
					{
						trig->_activation_time = 200; // 200ms
					}
					trig->update_intervalometer();
					trig->_intervalcall = true;
				}else
				{
					if(commandParamToInt(cmd.param3)> 0)
					{
						led_on(LED_LINEA);
					}
					else{
						led_off(LED_LINEA);
					}
				}
				break ;
			case LINE_B:
				trig->_mode = LINE_B;
				if(cmd.param2 > 0)
				{
					trig->_interval = cmd.param2 * 1000;
					trig->_activation_time = cmd.param4 * 1000;
					if(trig->_activation_time < 1.0f)
					{
						trig->_activation_time = 200; // 200ms
					}
					trig->update_intervalometer();
					trig->_intervalcall = true;
				}else{
					if(commandParamToInt(cmd.param3)> 0)
					{
						led_on(LED_LINEB);
					}
					else{
						led_off(LED_LINEB);
					}
				}
				break ;
			case LINE_AB:
				trig->_mode = LINE_AB;
				if(cmd.param2 > 0)
				{
					trig->_interval = cmd.param2 * 1000;
					trig->_activation_time = cmd.param4 * 1000;
					if(trig->_activation_time < 1.0f)
					{
						trig->_activation_time = 200; // 200ms
					}
					trig->update_intervalometer();
					trig->_intervalcall = true;
				}else{
					if(commandParamToInt(cmd.param3)> 0)
					{
						led_on(LED_LINEA);
						led_on(LED_LINEB);
					}
					else{
						led_off(LED_LINEA);
						led_off(LED_LINEB);
					}
				}
				break ;
			case LINE_A_B:
				trig->_mode = LINE_A_B;
				if(cmd.param2 > 0)
				{
					trig->_interval = cmd.param2 * 1000;
					trig->_activation_time = cmd.param4 * 1000;
					if(trig->_activation_time < 1.0f)
					{
						trig->_activation_time = 200; // 200ms
					}
					trig->update_intervalometer();
					trig->_intervalcall = true;
				}
				break ;
			default :
				break ;
			}


			vehicle_command_ack_s command_ack = {};
			command_ack.timestamp = hrt_absolute_time();
			command_ack.command = cmd.command;
			command_ack.result = (uint8_t)vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			command_ack.target_system = cmd.source_system;
			command_ack.target_component = cmd.source_component;

			if (trig->_cmd_ack_pub == nullptr) {
				trig->_cmd_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						     vehicle_command_ack_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command_ack), trig->_cmd_ack_pub, &command_ack);
			}
		}
	}

	work_queue(LPWORK, &_work, (worker_t)&LedLineControl::cycle_trampoline,
			ledline_control::g_ledline_control, USEC2TICK(poll_interval_usec));
}

void
LedLineControl::engage(void *arg)
{

	LedLineControl *trig = reinterpret_cast<LedLineControl *>(arg);
	switch(trig->_mode)
	{
	case LINE_A:
		led_on(LED_LINEA);
		break ;
	case LINE_B:
		led_on(LED_LINEB);
		break ;
	case LINE_AB:
		led_on(LED_LINEA);
		led_on(LED_LINEB);
		break ;
	case LINE_A_B:
		if(trig->_loop == 0){
			led_on(LED_LINEA);
		}else
		{
			led_on(LED_LINEB);
		}
		trig->_loop = 1 -trig->_loop;
		break ;;
	default :
		break ;
	}
}

void
LedLineControl::disengage(void *arg)
{
	LedLineControl *trig = reinterpret_cast<LedLineControl *>(arg);

	switch(trig->_mode)
	{
	case LINE_A:
		led_off(LED_LINEA);
		break ;
	case LINE_B:
		led_off(LED_LINEB);
		break ;
	case LINE_AB:
		led_off(LED_LINEA);
		led_off(LED_LINEB);
		break ;
	case LINE_A_B:
		led_off(LED_LINEA);
		led_off(LED_LINEB);
		break ;
	default :
		break ;
	}
}

static int usage()
{
	PX4_INFO("usage: ledline_control {start|stop|test}\n");
	return 1;
}


int ledline_control_main(int argc, char *argv[])
{

	if (!strcmp(argv[1], "start")) {

		if (ledline_control::g_ledline_control != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		ledline_control::g_ledline_control = new LedLineControl();

		if (ledline_control::g_ledline_control == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		ledline_control::g_ledline_control->start();
		return 0;
	}

	if (ledline_control::g_ledline_control == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		ledline_control::g_ledline_control->stop();
	} else if (!strcmp(argv[1], "test")) {
		ledline_control::g_ledline_control->test();
	} else {
		return usage();
	}

	return 0;
}

