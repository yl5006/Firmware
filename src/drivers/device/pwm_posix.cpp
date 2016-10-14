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
 * @file pwm_posix.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */   //yaoling

#include "pwm_posix.h"
#ifdef __PX4_LINUX
#endif
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>

namespace device {
PWM::PWM(const char *name,
         const char *devname,
         int npwm,
         uint32_t period,
         uint32_t duty_cycle,
         int polarity) :
	// base class
	VDev(name, devname),
	// public
	// protected
	// private
	_npwm(npwm),
	_period(period),
	_duty_cycle(duty_cycle),
	_polarity(polarity) {
	//warnx("GPIO::GPIO name = %s devname = %s", name, devname);
	// fill in _device_id fields for a GPIO device
	_device_id.devid_s.bus_type = DeviceBusType_GPIO;
	_device_id.devid_s.bus = npwm;
	//_device_id.devid_s.address = address;   no address
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

PWM::~PWM() {
	// XXX no way to let go of the bus...
	if (_fd >= 0) {
#ifndef __PX4_QURT
		::close(_fd);
#endif
		_fd = -1;
	}
}

int
PWM::init() {
	int ret = OK;
	ret = VDev::init();
	if (ret != PX4_OK) {
		DEVICE_DEBUG("VDev::init failed");
		return ret;
	}

	for(int i=0; i<_npwm; i+=2) {
		ini_each(i);
	}
	
	/* tell the workd where we are */
	warnx("init %d PWM at period %d,duty_cycle %d,polarity %s.", _npwm, _period, _duty_cycle,_polarity?"inversed":"normal");
	return 0;
}


int
PWM::init(int pwm_id) {
	int ret = OK;
	ret = VDev::init();
	if (ret != PX4_OK) {
		DEVICE_DEBUG("VDev::init failed");
		return ret;
	}
	ini_each(pwm_id);

	/* tell the workd where we are */
	warnx("init %d PWM at period %d,duty_cycle %d,polarity %s.", _npwm, _period, _duty_cycle,_polarity?"inversed":"normal");
	return 0;
}


int
PWM::ini_each(int index) {

	char  nor[7] ="normal";
	char  inver[9] ="inversed";
	char buffer[128] = {0};
	int len;
	int fd;
	char pwmname[128] = {0};

	
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/export",index);
	fd = ::open(pwmname, O_WRONLY);
	if (fd < 0) {
		warnx("Failed to open export  %s for writing!\n",pwmname);
		return(-1);
	}
	len = sprintf(buffer, "%d", 0);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to export pwmchip%d pwm0 or aready export!",index);
		//	return -1;
	}
	len = sprintf(buffer, "%d", 1);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to export pwmchip%d pwm1 or aready export!",index);
		//	return -1;
	}
	::close(fd);


	// disable pwm
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm0/enable",index);
	len = sprintf(buffer, "%d", 0);
	fd = ::open(pwmname, O_WRONLY);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to disable pwmchip%d pwm0!",index);
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm1/enable",index);
	fd = ::open(pwmname, O_WRONLY);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to disable pwmchip%d pwm1s!",index);
		return -1;
	}
	::close(fd);

	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm0/period",index);
	fd = ::open(pwmname, O_WRONLY);
	len = sprintf(buffer, "%d", _period);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set pwm0 period %d",_period);
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm0/duty_cycle",index);
	fd = ::open(pwmname, O_WRONLY);
	len = sprintf(buffer, "%d", _duty_cycle);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set pwm0 duty_cycle %d",_duty_cycle);
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm0/polarity",index);
	fd = ::open(pwmname, O_WRONLY);
	if(_polarity==0) {
		if (::write(fd, nor, 7) < 0) {
			warnx("Fail to set pwm0 polarity normal!");
			return -1;
		}
	} else {
		if (::write(fd, inver, 9) < 0) {
			warnx("Fail to set pwm0 polarity inversed!");
			return -1;
		}
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm1/period",index);
	fd = ::open(pwmname, O_WRONLY);
	len = sprintf(buffer, "%d", _period);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set period!");
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm1/duty_cycle",index);
	fd = ::open(pwmname, O_WRONLY);
	len = sprintf(buffer, "%d", _duty_cycle);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set duty_cycle!");
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm1/polarity",index);
	fd = ::open(pwmname, O_WRONLY);
	if(_polarity==0) {
		if (::write(fd, nor, 7) < 0) {
			warnx("Fail to set pwm1 polarity normal !");
			return -1;
		}
	} else {
		if (::write(fd, inver, 9) < 0) {
			warnx("Fail to set pwm1 polarity inversed!");
			return -1;
		}
	}
	::close(fd);
	// enable pwm
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm0/enable",index);
	len = sprintf(buffer, "%d", 1);
	fd = ::open(pwmname, O_WRONLY);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to enable pwmchip%d pwm0!",index);
		return -1;
	}
	::close(fd);
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm1/enable",index);
	fd = ::open(pwmname, O_WRONLY);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to enable pwmchip%d pwm1s!",index);
		return -1;
	}
	::close(fd);
	return 0;

}

int
PWM::probe() {
	// assume the device is too stupid to be discoverable
	return OK;
}
/***************************************
set PWM form 0 to num-1
all period same just set duty_cycle
***************************************/
int
PWM::pwm_set(uint8_t num, uint32_t  duty_cycle) {
#ifndef __PX4_LINUX
	return 1;
#else
	char buffer[128] = {0};
	int len;
	int fd;
	char pwmname[128] = {0};
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",((int)num/2)*2,num%2);
	fd = ::open(pwmname, O_WRONLY);
	if (fd < 0) {
		warnx("failed to open Pwm %s  write!\n",pwmname);
		return -1;
	}
	len = sprintf(buffer, "%d", duty_cycle);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set pwm  %s duty_cycle%d!",pwmname,duty_cycle);
		return -1;
	}
	::close(fd);
	return 0;
#endif
}
/***************************************
get PWM form 0 to num-1
all period same just get duty_cycle
***************************************/
uint16_t
PWM::pwm_get(uint8_t num) {
	char pwmname[128] = {0};
	char buffer[128] = {0};
	uint16_t duty_cycle;
	int fd;
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",((int)num/2)*2,num%2);
	fd = ::open(pwmname, O_RDONLY);
	if (fd < 0) {
		warnx("failed to open Pwm  read!\n");
		return -1;
	}
	if (::read(fd, buffer, 50) < 0) {
		warnx("Fail to get pwm duty_cycle!");
		return -1;
	}
	duty_cycle=atoi(buffer);
	::close(fd);
	return duty_cycle;
}


/* add by duyong, 2016-06-07, ԭ��:

	There are six fd to fetch/set;so we create six methods to get/set
	Then use other methods to combine them for further use

	/sys/class/pwm/pwmchip%d/export
	/sys/class/pwm/pwmchip%d/pwm0/enable
	/sys/class/pwm/pwmchip%d/pwm0/period
	/sys/class/pwm/pwmchip%d/pwm0/duty_cycle
	/sys/class/pwm/pwmchip%d/pwm0/polarity
	/sys/class/pwm/pwmchip%d/pwm0/enable

*/

/*****************************************************************************
 * �� �� ��  : pwm_export
 * �� �� ��  : duyong
 * ��������  : 2016��6��7��
 * ��������  : ��pwmоƬ�еĶ���˿�
 * �������  : chip_id : pwmоƬID
 			   pwm_port_count :pwm�˿� ��0��N
 * �������  : ��
 * �� �� ֵ  : ��
 * ���ù�ϵ  :
 * ��    ��  :

*****************************************************************************/
int
PWM::pwm_export(uint8_t chip_id,uint8_t pwm_port_count) {

	char buffer[32] = {0};
	int len = 0;
	int fd = -1;
	char pwmname[128] = {0};

	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/export",chip_id);
	fd = ::open(pwmname, O_WRONLY);
	if (fd < 0) {
		warnx("Failed to open export for writing!\n");
		return(-1);
	}
	for(uint8_t i = 0 ; i < pwm_port_count ; i++) {
		len = sprintf(buffer, "%d", i);
		if (::write(fd, buffer, len) < 0) {
			warnx("Fail to export pwmchip%d pwm%d or aready export!",chip_id,i);
		}
	}
	::close(fd);
	return 0;
}


/*****************************************************************************
 * �� �� ��  : pwm_enable
 * �� �� ��  : duyong
 * ��������  : 2016��6��7��
 * ��������  :  ʹ��/ֹͣ pwm�˿�
 * �������  : chip_id: pwmоƬID
 				pwm_port:pwm�˿�
 				enable:	true  ʹ��
 						false ֹͣ
 * �������  : ��
 * �� �� ֵ  : 0 :�ɹ� -1:ʧ��
 * ���ù�ϵ  :
 * ��    ��  :

*****************************************************************************/
int
PWM::pwm_enable(uint8_t chip_id,uint8_t pwm_port,bool enable) {
	char buffer[32] = {0};
	int len = 0;
	int fd = -1;
	char pwmname[128] = {0};
	// disable pwm
	sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/enable",chip_id,pwm_port);
	len = sprintf(buffer, "%d", enable);
	fd = ::open(pwmname, O_WRONLY);
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to disable pwmchip%d pwm%d!",chip_id,pwm_port);
		return -1;
	}
	::close(fd);
	return 0;
}

/*****************************************************************************
 * �� �� ��  : pwm_cmd_set
 * �� �� ��  : duyong
 * ��������  : 2016��6��7��
 * ��������  : ���� pwm�˿� ����
 * �������  : chip_id: pwmоƬID
 				pwm_port:pwm�˿�
 				type:	ENABLE = 0,
						PERIOD,
						DUTY_CYCLE,
						POLARITY
				value: ��Ӧ���Ե���ֵ		
 * �������  : ��
 * �� �� ֵ  : 0 :�ɹ� -1:ʧ��
 * ���ù�ϵ  :
 * ��    ��  :

*****************************************************************************/



int
PWM::pwm_cmd_set(uint8_t chip_id,uint8_t pwm_port,int type,uint32_t value) {
	char buffer[32] = {0};
	int len = 0;
	int fd = -1;
	char pwmname[128] = {0};

	switch(type) {
	case 	ENABLE:		
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/enable",chip_id,pwm_port);
		break;
	case 	PERIOD:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/period",chip_id,pwm_port);
		_period = value;
		break;
	case 	DUTY_CYCLE:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",chip_id,pwm_port);
		_duty_cycle = value;
		break;
	case	POLARITY:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/polarity",chip_id,pwm_port);
		_polarity = value;
		break;
	default:
		warnx("pwm cmd illigal: type %d");
		return -1;
	}
	len = sprintf(buffer, "%d", value);
	fd = ::open(pwmname, O_WRONLY);
	if(fd < 0)
	{
		warnx("Fail to open %s %s ,error code %d!",pwmname,buffer,fd);
		return -1;
	}
	if (::write(fd, buffer, len) < 0) {
		warnx("Fail to set %s %s!",pwmname,buffer);
		return -1;
	}
	::close(fd);
	return 0;
}

/*****************************************************************************
 * �� �� ��  : pwm_cmd_get
 * �� �� ��  : duyong
 * ��������  : 2016��6��7��
 * ��������  : ���� pwm�˿� ����
 * �������  : chip_id: pwmоƬID
 				pwm_port:pwm�˿�
 				type:	ENABLE = 0,
						PERIOD,
						DUTY_CYCLE,
						POLARITY
				value: ��Ӧ���Ե���ֵ		
 * �������  : ��
 * �� �� ֵ  : 0 :�ɹ� -1:ʧ��
 * ���ù�ϵ  :
 * ��    ��  :

*****************************************************************************/

int
PWM::pwm_cmd_get(uint8_t chip_id,uint8_t pwm_port,int type,uint32_t &value) {
	char pwmname[128] = {0};
	char buffer[128] = {0};
	int fd;

	switch(type) {
	case 	ENABLE:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/enable",chip_id,pwm_port);
		break;
	case 	PERIOD:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/period",chip_id,pwm_port);

		break;
	case 	DUTY_CYCLE:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",chip_id,pwm_port);
		break;
	case	POLARITY:
		sprintf(pwmname,"/sys/class/pwm/pwmchip%d/pwm%d/polarity",chip_id,pwm_port);
		break;
	default:
		warnx("pwm cmd illigal: type %d",type);
		return -1;
	}	
	fd = ::open(pwmname, O_RDONLY);
	if(fd < 0)
	{
		warnx("Fail to open %s %s ,error code %d!",pwmname,buffer,fd);
		return -1;
	}
	if (::read(fd, buffer, 50) < 0) {
		warnx("Fail to get pwm duty_cycle!");
		return -1;
	}
	value=atoi(buffer);
	::close(fd);
	return 0;
}




} // namespace device
