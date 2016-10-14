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
 * @file i2c.cpp
 *
 * Base class for devices attached via the GPIO bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "gpio.h"
#ifdef __PX4_LINUX
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define PX4_SIMULATE_GPIO 0
static int simulate = PX4_SIMULATE_GPIO;

namespace device
{

//const static int GPIO::BUFFER_MAX=3;
//const static int GPIO::DIRECTION_MAX=48;
#if 0
const int GPIO::IN=0;
const int GPIO::OUT=1;
const int GPIO::LOW=0;
const int GPIO::HIGH=1;
#endif
GPIO::GPIO(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address
	) :             
	// base class
	VDev(name, devname),
	// public
	// protected
	_retries(0),
	// private
	_bus(bus),
	_address(address),
	_fd(-1)
{
	//warnx("GPIO::GPIO name = %s devname = %s", name, devname);
	// fill in _device_id fields for a GPIO device
	_device_id.devid_s.bus_type = DeviceBusType_GPIO;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

GPIO::~GPIO()
{
	if (_fd >= 0) {
#ifndef __PX4_QURT
		::close(_fd);
#endif
		_fd = -1;
		gpio_unexport();
	}
}

int
GPIO::init()
{
	int ret = PX4_OK;

	// Assume the driver set the desired bus frequency. There is no standard
	// way to set it from user space.
	// do base class init, which will create device node, etc

	ret = VDev::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("VDev::init failed");
		return ret;
	}

#ifdef __PX4_QURT
	PX4_WARN("simulate");
	simulate = true;
#endif

	if (simulate) {
		_fd = 10000;

	} else {
#ifndef __PX4_QURT
		// Open the actual GPIO device and map to the virtual dev name
		//we use busid as the pinId
		if(gpio_export() != 0)
		{
			return PX4_ERROR;
		}
		
#if 0		 
		warnx("ready to open %s", _devname);
		_fd = ::open(_devname, O_RDWR);
		if (_fd < 0) {
			warnx("could not open %s", _devname);
			px4_errno = errno;
			return PX4_ERROR;
		}
#endif
#endif
	}
	ret = probe();
		if (ret != OK) {
			DEVICE_DEBUG("probe failed");
			return ret;
		}
	return ret;
}
int
GPIO::probe()
{
	// Assume the device is too stupid to be discoverable.
	return OK;
}

/*
	int GPIO::ioctl(device::file_t *filp, int cmd, unsigned long arg)
	filp : NULL
	cmd : IN/OUT for directions
	arg : let it be 0

*/
int GPIO::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	//we don't need this function in GPIO
	return gpio_direction(cmd);
}
/*
	ssize_t	GPIO::read(file_t *filp, char *buffer, size_t buflen)
	filp : NULL
	buffer : read values
	buflen : let it be 0

*/
ssize_t	GPIO::read(file_t *filp, char *buffer, size_t buflen)
{
	if (simulate) {
		// FIXME no idea what this should be
		warnx("2C SIM GPIO::read");
		return 0;
	}

#ifndef __PX4_QURT
	if(buffer == NULL)
	{
		warnx("2C SIM GPIO::read buffer is NULL");
		return -1;
	}
	return gpio_read((int *)buffer);
#else
	return 0;
#endif
}
/*
	ssize_t	GPIO::read(file_t *filp, char *buffer, size_t buflen)
	filp : NULL
	buffer : write values
	buflen : let it be 0

*/
ssize_t	GPIO::write(file_t *filp, const char *buffer, size_t buflen)
{
	if (simulate) {
		warnx("2C SIM GPIO::write");
		return buflen;
	}

#ifndef __PX4_QURT
	if(buffer == NULL)
	{
		warnx("2C SIM GPIO::write buffer is NULL");
		return -1;
	}
	int value = *(int *)buffer;
	return gpio_write(value);
#else
	return buflen;
#endif
}

#if 1
int GPIO::gpio_export()
{
    char buffer[128] = {0};
    int len;
    int fd;
	char gpioname[128] = {0};
	sprintf(gpioname,"/sys/class/gpio/gpio%d/value",_bus);
	
	if( (access(gpioname, 0 )) == 0)
	{
	  printf( "File %s exists:OK ",gpioname);
	  return 0;	 
	}

    fd = ::open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open export for writing!\n");
        return(-1);
    }

    len = snprintf(buffer, BUFFER_MAX, "%d", _bus);
    if (::write(fd, buffer, len) < 0) {
        fprintf(stderr, "Fail to export gpio%d!",_bus);
        return -1;      
    }
    
    ::close(fd);
    return 0;
}

int GPIO::gpio_unexport()
{
    char buffer[BUFFER_MAX];
    int len;
    int fd;

    fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open unexport for writing!\n");
        return -1;
    }

    len = snprintf(buffer, BUFFER_MAX, "%d",_bus);
    if (::write(fd, buffer, len) < 0) {
        fprintf(stderr, "Fail to unexport gpio%d!",_bus);
        return -1;
    }
    
    ::close(fd);
    return 0;
}

int GPIO::gpio_direction( int dir)
{
    static const char dir_str[]  = "in\0out";
    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction",_bus);
    fd = ::open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "failed to open gpio direction for writing!%s \n",path);
        return -1;
    }

    if (::write(fd, &dir_str[dir == IN ? 0 : 3], dir == IN ? 2 : 3) < 0) {
        fprintf(stderr, "failed to set direction!\n");
        return -1;
    }

    ::close(fd);
    return 0;
}

int
GPIO::gpio_read(int *value)
{
    char path[DIRECTION_MAX];
    char value_str[3] = {0};
    int fd = 0;
	if(value == NULL){
		fprintf(stderr, "failed to set space for reading!\n");
		return -1;
	}
    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/value",_bus);
    fd = ::open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "failed to open gpio value for reading!\n");
        return -1;
    }

    if (::read(fd, value_str, 3) < 0) {
        fprintf(stderr, "failed to read value!\n");
        return -1;
    }

    ::close(fd);
	*value = atoi(value_str);
    return 0;
}

int GPIO::gpio_write(int value)
{
    static const char values_str[] = "01";
    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/value",_bus);
    fd = ::open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "failed to open gpio value for writing!\n");
        return -1;
    }

    if (::write(fd, &values_str[value == LOW ? 0 : 1], 1) < 0) {
        fprintf(stderr, "failed to write value!\n");
        return -1;
    }

    ::close(fd);
    return 0;
}
#endif


} // namespace device
