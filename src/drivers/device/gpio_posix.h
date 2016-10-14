/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file i2c.h
 *
 * Base class for devices connected via GPIO.
 */

#ifndef _DEVICE_GPIO_H
#define _DEVICE_GPIO_H

#include "vdev.h"

#ifdef __PX4_LINUX
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#endif
#include <string>

namespace device __EXPORT
{

/**
 * Abstract class for character device on GPIO
 */
class __EXPORT GPIO : public VDev
{

public:
	/**
	 * @ Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		GPIO bus on which the device lives
	 * @param address	GPIO bus address, or zero if set_address will be used
 	 * @param frequency	GPIO bus frequency for the device (currently not used)
	 * @param irq		Interrupt assigned to the device (or zero if none)
	 */
	GPIO(const char *name,
	    const char *devname,
	    int bus,
	    uint16_t address);
	virtual ~GPIO();
	/**
	 * Get the address
	 */
	int16_t		get_address() const { return _address; }

	const static int IN=0;
	const static int OUT=1;
	const static int LOW=0;
	const static int HIGH=1;


	/**
	 * The number of times a read or write operation will be retried on
	 * error.
	 */
	unsigned		_retries;

	/**
	 * The GPIO bus number the device is attached to.
	 */
	int			_bus;



	virtual int	init();

	virtual int	probe();

	virtual ssize_t	read(file_t *handlep, char *buffer, size_t buflen);
	virtual ssize_t	write(file_t *handlep, const char *buffer, size_t buflen);
	virtual int	ioctl(file_t *handlep, int cmd, unsigned long arg);

	//set gpio port whether IN or OUT
	int gpio_direction(int dir);
	//read from pin
	int gpio_read(int *value);
	//write to pin
	int gpio_write(int value);
	/**
	 * Change the bus address.
	 *
	 * Most often useful during probe() when the driver is testing
	 * several possible bus addresses.  /////////add yaoling
	 *
	 * @param address	The new bus address to set.
	 */
	void		set_address(uint16_t address)
	{
		_address = address;
		_device_id.devid_s.address = _address;
	}
	
private:
	uint16_t		_address;
	int 			_fd;
	char 			_devname[20];
	const static int BUFFER_MAX=10;
	const static int DIRECTION_MAX=100;





	GPIO(const device::GPIO &);
	GPIO operator=(const device::GPIO &);

#if 1
	//write to /sys/class/gpio/export to create gpio handle
	int gpio_export();
	//write to /sys/class/gpio/unexport to remove gpio handle
	int gpio_unexport();
#endif
};

} // namespace device

#endif /* _DEVICE_GPIO_H */
