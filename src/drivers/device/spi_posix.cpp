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
 * @file spi.cpp
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

#include "spi.h"
#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifndef PX4_SIMULATE_SPI
#define PX4_SIMULATE_SPI 0
#endif
static int simulate = PX4_SIMULATE_SPI;



namespace device {

SPI::SPI(const char *name,
         const char *devname,
         int bus,
         enum spi_dev_e device,
         uint32_t mode,
         uint32_t frequency,
         int irq) :
	// base class
	VDev(name, devname),
	// public
	// protected
	// private
	_device(device),
	_mode(mode),
	_frequency(frequency),
	_bits(8),
	_dev(nullptr),
	_bus(bus) {
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

/* add begin by duyong, 2016-07-29, Ô­Òò: */
/*
	note that function transfer used for write one byte only.
	transfer_sel is abandoned.
	easy_transfer_sel used for write one byte and read multi-bytes.
	So before calling these functions ,you'd better read datasheet of each device.
*/
/* add end by duyong, 2016-07-29 */

SPI::~SPI() {
	// XXX no way to let go of the bus...
	if (_fd >= 0) {
#ifndef __PX4_QURT
		::close(_fd);
#endif
		_fd = -1;
	}
}

int
SPI::init() {
	int ret = OK;
	ret = VDev::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("VDev::init failed");
		return ret;
	}
	if (simulate) {
		PX4_WARN("simulate");
		_fd = 10000;
	} else {
#ifndef __PX4_QURT
		// Open the actual I2C device and map to the virtual dev name
		snprintf(_devname,sizeof(_devname),"/dev/spidev%d.%d",_bus,_device);
		warnx("open %s", _devname);
//		_fd = ::open(get_devname(), O_RDWR);
		_fd = ::open(_devname, O_RDWR);
		if (_fd < 0) {
			warnx("could not open %s", _devname);
			px4_errno = errno;
			return PX4_ERROR;
		}
#endif
	}
//	SPI_SELECT(_dev, _device, false);

	ret = ::ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_frequency);
	if (ret == -1)
		warnx("can't set max speed hz");

	/* call the probe function to check whether the device is present */
	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}
	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", _bus, _device, _frequency / 1000);
#if 0
	::close(_fd);
#endif
out:
	return ret;
}

int
SPI::probe() {
	// assume the device is too stupid to be discoverable
	return OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len) {
#ifndef __PX4_LINUX
	return 1;
#else
	int ret = OK;
	ret = ::ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
	if (ret == ERROR) {
		warnx("1can't set spi mode");
		return ERROR;
	}
	struct spi_ioc_transfer tr;
	tr.tx_buf = (unsigned long)send;
	tr.rx_buf = (unsigned long)recv;
	tr.len = len;
	tr.speed_hz = _frequency;
	tr.bits_per_word = _bits;
	ret = ::ioctl(_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1)
	{

		perror("spi::transfer");
		warnx("2can't send spi message,ret %d",ret);
		for(int i = 0 ; i < len; i++)
			printf("%#x ",send[i]);
		printf("\r\n");
		return ERROR;
	}
	return OK;
#endif
}

int
SPI::transfer_sel(uint8_t *send, uint8_t *recv, unsigned len,uint32_t delay) {
#ifndef __PX4_LINUX
	return 1;
#else
	int ret = OK;

	ret = ::ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
	if (ret == -1) {
		warnx("2can't set spi mode");
		return ERROR;
	}
	/*
	 * bits per word
	 */
	ret = ::ioctl(_fd,SPI_IOC_WR_BITS_PER_WORD,&_bits);
	if (ret == -1) {
		warnx("can't set spi bits");
		return ERROR;
	}
	char buf1[8];
	char buf2[8];
//		uint8_t *bp = NULL;
	memset(buf1, 0, sizeof buf1);
	memset(buf2, 0, sizeof buf2);
	struct spi_ioc_transfer tr[2];
	tr[0].tx_buf = (unsigned long)send;
	tr[0].rx_buf = (unsigned long)buf1;
	tr[0].len = 1; /* Length of  command to write*/
	tr[0].cs_change = 0; /* Keep CS activated */
	tr[0].delay_usecs = delay, //delay in us
	tr[0].speed_hz = _frequency, //speed
	tr[0].bits_per_word = _bits, // bites per word 8

	tr[1].tx_buf = (unsigned long)buf2;
	tr[1].rx_buf = (unsigned long)recv;
	tr[1].len = len; /* Length of Data to read */
	tr[1].cs_change = 0; /* Keep CS activated */
	tr[1].delay_usecs = 0;
	tr[1].speed_hz = _frequency;
	tr[1].bits_per_word = _bits;
	ret = ::ioctl(_fd, SPI_IOC_MESSAGE(2), &tr);
	if (ret < (len + 1))
	{
	warnx("1can't send spi message buf[0] addr %x %x freq %d bits %d",send[0],send[1],_frequency,_bits);
	return ERROR;
	}
//	 printf("response(%2d, %2d): ", len, ret);
//	 for (bp = recv; len; len--)
//	 		printf(" %02x", *bp++);
	return OK;
#endif
}



int
SPI::easy_transfer_sel(uint8_t *send, uint8_t *recv, unsigned len) {
#ifndef __PX4_LINUX
	return 1;
#else
	int ret = OK;
	ret = ::ioctl(_fd, SPI_IOC_WR_MODE, &_mode);
	if (ret == -1) {
		warnx("2can't set spi mode");
		return ERROR;
	}

	ret = ::ioctl(_fd,SPI_IOC_WR_BITS_PER_WORD,&_bits);
	if (ret == -1) {
		warnx("can't set spi bits");
		return ERROR;
	}

	struct spi_ioc_transfer tr[2];
	memset(tr, 0, 2*sizeof(struct spi_ioc_transfer));

	tr[0].tx_buf = (unsigned long)send;
	tr[0].len = 1; /* Length of  command to write*/

	tr[1].rx_buf = (unsigned long)recv;
	tr[1].len = len; /* Length of Data to read */
	//now return the true value of the message len
	ret = ::ioctl(_fd, SPI_IOC_MESSAGE(2), &tr);
	if (ret < (len + 1))
	{	warnx("2can't send spi message buf[0] addr %x %x freq %d bits %d",send[0],send[1],_frequency,_bits);
		return ERROR;
	}

//	 printf("\r\nresponse(%2d, %#x, %2d, %2d): ",_fd, *send,len, ret);
//	 for (uint8_t *bp = recv; len; len--)
//	 		printf(" %02x", *bp++);
//	 printf("\r\n");
	return OK;
#endif
}
void
SPI::set_frequency(uint32_t frequency) {
	_frequency = frequency;
}
void
SPI::set_bits(uint32_t setbits) {
	_bits = setbits;
}
} // namespace device
