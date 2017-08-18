/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author  zhouping <zhpg_0803@163.com>
 */

#include "diff_pressure.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

const char *const UavcanDiffPressureBridge::NAME = "diff_pressure";

UavcanDiffPressureBridge::UavcanDiffPressureBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_diff_pressure", "/dev/uavcan/diff_pressure", AIRSPEED_BASE_DEVICE_PATH, ORB_ID(differential_pressure)),
	_sub_diffpressure(node)
{
	param_get(param_find("SENS_DPRES_OFF"), &(_scale.offset_pa));		//get param "SENS_DPRES_OFF"
	_scale.scale = 1.0F;

//	warnx("UavcanDiffPressureBridge  diffp_scale.offset_pa=%.7f, diffp_scale.scale=%.7f",
//						static_cast<double>(_scale.offset_pa),
//						static_cast<double>(_scale.scale)
//					  );
}

int UavcanDiffPressureBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_diffpressure.start(DiffPressureCbBinder(this, &UavcanDiffPressureBridge::diffpressure_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanDiffPressureBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct diffpressure_report *diffpressure_buf = reinterpret_cast<struct diffpressure_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct diffpressure_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _report.timestamp) {
		/* copy report */
		lock();
		*diffpressure_buf = _report;
		last_read = _report.timestamp;
		unlock();
		return sizeof(struct diffpressure_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}

int UavcanDiffPressureBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
		case SENSORIOCSQUEUEDEPTH: {
				return OK;			// Pretend that this stuff is supported to keep APM happy
			}

		case AIRSPEEDIOCSSCALE: {
				std::memcpy(&_scale, reinterpret_cast<const void *>(arg), sizeof(_scale));
//				warnx("AIRSPEEDIOCSSCALE  diffp_scale.offset_pa=%.7f, diffp_scale.scale=%.7f",
//						static_cast<double>(_scale.offset_pa),
//						static_cast<double>(_scale.scale)
//					  );
				return 0;
			}

		case AIRSPEEDIOCGSCALE: {
				std::memcpy(reinterpret_cast<void *>(arg), &_scale, sizeof(_scale));
//				warnx("AIRSPEEDIOCGSCALE  diffp_scale.offset_pa=%.7f, diffp_scale.scale=%.7f",
//						static_cast<double>(_scale.offset_pa),
//						static_cast<double>(_scale.scale)
//					  );
				return 0;
			}

		default: {
				return CDev::ioctl(filp, cmd, arg);
			}
		}
}

void UavcanDiffPressureBridge::diffpressure_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ewatt::DifferentialPressure>
		&msg)
{
	lock();

//members:
//	uint64_t timestamp; // required for logger
//	uint64_t error_count;
//	float differential_pressure_raw_pa;
//	float differential_pressure_filtered_pa;
//	float max_differential_pressure_pa;
//	float temperature;

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	_report.timestamp = hrt_absolute_time();
	_report.error_count = 0;
	_report.differential_pressure_raw_pa = msg.differential_pressure_raw_pa - _scale.offset_pa;
	_report.differential_pressure_filtered_pa = msg.differential_pressure_filtered_pa - _scale.offset_pa;
	//_report.max_differential_pressure_pa = msg.max_differential_pressure_pa;
	_report.temperature = msg.temperature;

//debug:
//	warnx("_report.differential_pressure_raw_pa=%.7f,"
//			"_report.differential_pressure_filtered_pa =%.7f,"
//			"_report.max_differential_pressure_pa =%.7f,"
//			"_report.temperature =%.7f," ,
//			static_cast<double>(_report.differential_pressure_raw_pa),
//			static_cast<double>(_report.differential_pressure_filtered_pa),
//			static_cast<double>(_report.max_differential_pressure_pa),
//			static_cast<double>(_report.temperature));
	unlock();

	publish(msg.getSrcNodeID().get(), &_report);
}
