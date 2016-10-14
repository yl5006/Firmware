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
 * @file pwm_posix.h
 *
 * Base class for devices connected via SPI.
 */

#ifndef _DEVICE_PWM_H
#define _DEVICE_PWM_H

#include "device.h"

#ifdef __PX4_LINUX
#endif
namespace device __EXPORT
{
/**
 * Abstract class for character device on SPI
 */
class __EXPORT PWM : public VDev
{
public:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
 	 * @param npwm	        PWM npwm 
	 * @param period	PWM period
	 * @param duty_cycle	PWM duty_cycle  unit ns
	 * @param polarity	PWM polarity (or zero if none)
	 */
	PWM(const char *name,
	    const char *devname,
	    int npwm,
	    uint32_t period,
	    uint32_t duty_cycle,
	    int polarity = 0);
	virtual ~PWM();

	virtual int	init();

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int	probe();

	/**
	 * Set and get the PWM bus period duty_cycle
	 */
	int				ini_each(int index);
	int 			init(int pwm_id);
	int				pwm_set(uint8_t num, uint32_t  duty_cycle);
	uint16_t		pwm_get(uint8_t num);
	int 			pwm_export(uint8_t chip_id,uint8_t pwm_port_count);
	int 			pwm_enable(uint8_t chip_id,uint8_t pwm_port,bool enable);
	int				pwm_cmd_set(uint8_t chip_id,uint8_t pwm_port,int type,uint32_t value);
	int				pwm_cmd_get(uint8_t chip_id,uint8_t pwm_port,int type,uint32_t &value);
	enum pwmCmdType {
	ENABLE = 0,
	PERIOD,
	DUTY_CYCLE,
	POLARITY
	};
private:
	uint32_t		_npwm;
	uint32_t		_period;
	uint32_t		_duty_cycle;
	int 			_polarity;


	/* this class does not allow copying */
	PWM(const PWM &);
	PWM operator=(const PWM &);

protected:
	uint16_t		_address;
	int 			_fd;
	char 			_devname[20];
	int			_bus;
};

} // namespace device

#endif /* _DEVICE_SPI_H */
