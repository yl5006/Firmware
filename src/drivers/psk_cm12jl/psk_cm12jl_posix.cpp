/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file fmu.cpp
 *
 * Driver/configurator for the PX4 FMU multi-purpose port on v1 and v2 boards.
 */
#include <px4_getopt.h>
#include <px4_config.h>
#include <px4_workqueue.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string>

#include <drivers/device/device.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/systemlib.h>
#include <systemlib/board_serial.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/horizontal_distance.h>

#include <lib/rc/cm12jl.h>

//extern int	cm12jl_init(const char *device);

#define SCHEDULE_INTERVAL	10000	/**< The schedule interval in usec (50 Hz) */
#define HDISTANCE_DEVICE_PATH "/dev/hdistance"


const int HDistance_ERROR = -1;

class HDistance : public device::VDev {
public:
	HDistance(int type = horizontal_distance_s::MAV_DISTANCE_SENSOR_INFRARED,const char* path = "/dev/ttyAMA0");
	virtual ~HDistance();

	virtual int	init();
	void				print_info();

private:
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	ringbuffer::RingBuffer	*_reports;
	bool				_sensor_ok;
	int				_measure_ticks;
	int				_class_instance;
	int				_orb_class_instance;
	bool			_initialized;
	orb_advert_t		_horizontal_distance_topic;
	int		_dis_fd;
	std::string 	_device_path;
	static void	cycle_trampoline(void *arg);
	void		cycle();
	void		work_start();
	void		work_stop();
	bool frame_char_buffer(horizontal_distance_s *report ,uint8_t c);
	/* this class has pointer data members, do not allow copying it */
	HDistance(const HDistance &);
	HDistance operator=(const HDistance &);
};
/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hdistance_main(int argc, char *argv[]);

namespace {

HDistance	*g_dev;

} // namespace


HDistance::HDistance(int type,const char* path) :
	VDev("hdistance", HDISTANCE_DEVICE_PATH),
	_min_distance(0.1),
	_max_distance(20),
	_work{},
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_initialized(false),
	_horizontal_distance_topic(nullptr),
	_dis_fd(0),
	_device_path(path) {
	/* enable debug() calls */
	_debug_enabled = true;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HDistance::~HDistance() {
	/* make sure we are truly inactive */
	work_stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(HDISTANCE_DEVICE_PATH, _class_instance);
	}
}
enum {
	PARSE_STATE_A5=0,
	PARSE_STATE_5A,
	PARSE_STATE_GOT_DATA,
	PARSE_STATE_CHECKA,
	PARSE_STATE_CHECKB
};
uint16_t _distance[5];
bool HDistance::frame_char_buffer(horizontal_distance_s *report ,uint8_t c)
{
	static int data=0;
	static int parse_state = PARSE_STATE_A5;
	static uint16_t checkab=0;
	bool parse=false;
	switch (parse_state)
		{
		case PARSE_STATE_A5:
			if (c == 0xA5)
			{
				parse_state = PARSE_STATE_5A;

			}
			break;
	    case PARSE_STATE_5A:
	         if (c == 0x5A) {
	                        parse_state = PARSE_STATE_GOT_DATA;
	                        		checkab=0xA5+0x5A;
	                        		data=0;
	                } else {
	                        parse_state = PARSE_STATE_A5;
	          }
	                break;
	    case PARSE_STATE_GOT_DATA:
	    	   _distance[data/2]=(_distance[data/2]<<8)+c;
	    	   checkab=checkab+c;
	    	   data++;
	    	   if(data==10)
	    	   {
	    		   	parse_state = PARSE_STATE_CHECKA;
	    	   }
	    	   break;
	    case PARSE_STATE_CHECKA:
	    		if(c==(checkab	>>	8))
					{
	    				parse_state = PARSE_STATE_CHECKB;
					}
	    		else
	    			{
	    			parse_state = PARSE_STATE_A5;
	    			}
	    		break;
	    case PARSE_STATE_CHECKB:
	    		if(c==(checkab & 0x00FF))
	   			{
	    			parse=true;
	    			report->current_distance[0]=_distance[0]*0.01;
	    			report->current_distance[1]=_distance[1]*0.01;
	    			report->current_distance[2]=_distance[2]*0.01;
	    			report->current_distance[3]=_distance[3]*0.01;
	   			}
	    			parse_state = PARSE_STATE_A5;
			break;
		}
	return parse;
}
int
HDistance::init() {
	int ret = HDistance_ERROR;

	ret = 	VDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("VDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(horizontal_distance_s));

	if (_reports == nullptr) {
		DEVICE_DEBUG("allocate basic report buffers failed");
		return ret;
	}

	_class_instance = register_class_devname(HDISTANCE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct horizontal_distance_s ds_report = {};

	_horizontal_distance_topic = orb_advertise(ORB_ID(horizontal_distance), &ds_report);

	if (_horizontal_distance_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	// XXX we should find out why we need to wait 200 ms here
	warnx("work start");
	work_start();

	return OK;
}

void
HDistance::work_start() {
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HDistance::cycle_trampoline, this, 1);
}

void
HDistance::cycle_trampoline(void *arg) {
	HDistance *dev = reinterpret_cast<HDistance *>(arg);
	dev->cycle();
}

void
HDistance::cycle() {


	if (!_initialized) {
		_dis_fd = cm12jl_init(_device_path.c_str());
		_initialized = true;
		warnx("cm12jl_init %d",_dis_fd);
	}

	uint16_t values[6] = {0};
	uint16_t num_values = 0;

	if(cm12jl_input(_dis_fd, &values[0], &num_values) && num_values == 5 && values[0] == ClusterData) {

		struct horizontal_distance_s report;
		report.timestamp = hrt_absolute_time();
		report.type = horizontal_distance_s::MAV_DISTANCE_SENSOR_INFRARED;
		report.orientation = 8;
		int value_count = 0;
		for(value_count= 1; value_count < num_values;value_count++)
		{
			if(values[value_count] == 0)
			{
				report.current_distance[value_count-1] = 20;
			}
			else
			{
				report.current_distance[value_count-1] = values[value_count] * 0.001;
			}
		}

		report.min_distance = _min_distance;
		report.max_distance = _max_distance;
		report.covariance = 0.0f;
		/* TODO: set proper ID */
		report.id = 0;
#if 0
		warnx("###cm12jl_input values %.4f %.4f %.4f %.4f %.4f ",(double)report.current_distance[0],
		      (double)report.current_distance[1],
		      (double)report.current_distance[2],
		      (double)report.current_distance[3],
		      (double)report.current_distance[4]);
#endif
		/* publish it, if we are the primary */
		if (_horizontal_distance_topic != nullptr) {
			orb_publish(ORB_ID(horizontal_distance), _horizontal_distance_topic, &report);
		}
		//
		_reports->force(&report);
	}





	// do publish here
	work_queue(HPWORK, &_work, (worker_t)&HDistance::cycle_trampoline, this,
	           USEC2TICK(SCHEDULE_INTERVAL));

}
void
HDistance::work_stop() {
	work_cancel(HPWORK, &_work);

	::close(_dis_fd);

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

void
HDistance::print_info() {
	printf("poll interval:  %u ticks\n", USEC2TICK(SCHEDULE_INTERVAL));
	_reports->print_info("report queue");
}
/**
 * Local functions in support of the shell command.
 */
namespace {

/* oddly, ERROR is not defined for c++ */

const int HDistance_ERROR = -1;


void	start(const char* dev_path);
void	stop();
void	info();

/**
 * Start the driver.
 */
void
start(const char* dev_path) {
	if (g_dev != nullptr) {
		warnx( "already started");
		return;
	}

	/* create the driver */
	g_dev = new HDistance(horizontal_distance_s::MAV_DISTANCE_SENSOR_INFRARED,dev_path);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}
	return ;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	warnx( "driver start failed");
}

/**
 * Stop the driver
 */
void stop() {
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		warnx( "driver not running");
	}

	return;
}

/**
 * Print a little info about the driver.
 */
void
info() {
	if (g_dev == nullptr) {
		warnx( "driver not running");
		return;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return;
}

} /* namespace */

int
hdistance_main(int argc, char *argv[]) {
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if(argc == 3 && argv[2]) {
			start(argv[2]);
		} else {
			warnx("please enter the device path");
		}
		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		stop();
		return 0;
	}


	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		info();
		return 0;
	}

	warnx("unrecognized command, try 'start', 'stop' or 'info'");
	return 1;
}
