/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file differential pressure driver interface.
 */

#ifndef _DRV_DIFFPRESSURE_H
#define _DRV_DIFFPRESSURE_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define DIFFPRESSURE_BASE_DEVICE_PATH	"/dev/diffpressure"
#define DIFFPRESSURE0_BASE_DEVICE_PATH	"/dev/diffpressure0"
#define DIFFPRESSURE1_BASE_DEVICE_PATH	"/dev/diffpressure1"
#define DIFFPRESSURE2_BASE_DEVICE_PATH	"/dev/diffpressure2"

#include <uORB/topics/differential_pressure.h>
#define diffpressure_report  differential_pressure_s


/*
 * ioctl() definitions
 */

#define _DIFFPRESSUREIOCBASE		(0x2400)
#define _DIFFPRESSUREIOC(_n)		(_PX4_IOC(_DIFFPRESSUREIOCBASE, _n))

/** set the mag internal sample rate to at least (arg) Hz */
#define DIFFPRESSUREIOCSSAMPLERATE	_DIFFPRESSUREIOC(0)

/** return the mag internal sample rate in Hz */
#define DIFFPRESSUREIOCGSAMPLERATE	_DIFFPRESSUREIOC(1)


#endif /* _DRV_DIFFPRESSURE_H */
