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
 * @file max1168.cpp
 *
 * Driver for the Invensense MAX1168 connected via SPI.
 *
 * @author yaoling
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

/* Max measurement rate is 200ksps,  using 1000 */
#define MAX1168_CONVERSION_INTERVAL	(1000000 / 1000)	/* microseconds */

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MAX_DEVICE_PATH_ACCEL		"/dev/max1168_accel"
#define MAX_DEVICE_PATH_GYRO		"/dev/max1168_gyro"
#define MAX_DEVICE_PATH_ACCEL_EXT	"/dev/max1168_accel_ext"
#define MAX_DEVICE_PATH_GYRO_EXT	"/dev/max1168_gyro_ext"
// MPU 6000 registers
#define ADVOLT_REF 4.096f
#define ACCEL_REF 3.3f
#define GYRO_REF ADVOLT_REF//4.096f

#ifndef MAX1168_CONF8_CHANNEL
#define MAX1168_CONF_CHANNEL 0x7 // select all channels 7
#endif
#ifndef MAX1168_CONF_SCAN
#define MAX1168_CONF_SCAN 0x1 // scan number of channels selected1
#endif
#ifndef MAX1168_CONF_REF
#define MAX1168_CONF_REF 0x0 // internal ref and no power down
#endif
#ifndef MAX1168_CONF_CLOCK
#define MAX1168_CONF_CLOCK 0x1 // internal clock
#endif

#define MAX1168_CONF_CR ((MAX1168_CONF_CHANNEL<<5)|(MAX1168_CONF_SCAN<<3)|(MAX1168_CONF_REF<<1)|(MAX1168_CONF_CLOCK))

#define MAX1168_ACCEL_DEFAULT_RANGE_G		3
#define MAX1168_ACCEL_DEFAULT_RATE			1000
#define MAX1168_ACCEL_MAX_OUTPUT_RATE		280
#define MAX1168_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	60

#define MAX1168_GYRO_DEFAULT_RANGE_G		3
#define MAX1168_GYRO_DEFAULT_RATE			1000
/* rates need to be the same between accel and gyro */
#define MAX1168_GYRO_MAX_OUTPUT_RATE			MAX1168_ACCEL_MAX_OUTPUT_RATE
#define MAX1168_GYRO_DEFAULT_DRIVER_FILTER_FREQ		200

#define MAX1168_DEFAULT_ONCHIP_FILTER_FREQ		200

#define MAX1168_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  the MAX1168 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
#define MAX1168_BUS_SPEED				10000*1000 /* will be rounded to 10.4 MHz, within margins for MAX1168 */

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MAX1168_TIMER_REDUCTION				200

class MAX1168_gyro;


class MAX1168 : public device::SPI
{
public:
	MAX1168(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~MAX1168();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

	friend class MAX1168_gyro;
	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);


private:
	work_s			_work;
	unsigned		_measure_ticks;
	MAX1168_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	bool			_collect_phase;

	struct hrt_call		_call;
	unsigned _call_interval;

	ringbuffer::RingBuffer *_accel_reports;

	struct accel_calibration_s _accel_scale;
	float _accel_range_scale;
	float _accel_range_m_s2;
	orb_advert_t _accel_topic;
	int _accel_orb_class_instance;
	int _accel_class_instance;

	ringbuffer::RingBuffer *_gyro_reports;

	struct gyro_calibration_s _gyro_scale;
	float _gyro_range_scale;
	float _gyro_range_rad_s;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;
	perf_counter_t		_system_latency_perf;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_accel_int;
	Integrator		_gyro_int;

	enum Rotation		_rotation;


	// use this to avoid processing measurements when in factory
	// self test
	volatile bool		_in_factory_test;

	// last temperature reading for print_info()
	float			_last_temperature;

	// keep last accel reading for duplicate detection
	uint16_t		_last_accel[3];
	bool			_got_duplicate;
	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);
	static void		measure_trampoline(void *arg);
	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	void			cycle();
	/**
	 * Read a register from the MAX1168
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	void			trigmeas();

	uint8_t			read_reg(unsigned reg, uint32_t speed = MAX1168_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the MAX1168
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);


	/**
	 * Swap a 16-bit value read from the MAX1168 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_bus == EXTERNAL_BUS); }

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();
	int				accel_self_test();
	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	 */
	void _set_sample_rate(unsigned desired_sample_rate_hz);


	/* do not allow to copy this class due to pointer data members */
	MAX1168(const MAX1168 &);
	MAX1168 operator=(const MAX1168 &);

#pragma pack(push, 1)
	/**
	 * Report conversation within the MAX1168, including command byte and
	 * interrupt status.
	 */
	struct MAXReport {
		//		uint8_t		cmd;
		uint8_t		gyro_y[2];
		uint8_t		gyro_x[2];
		uint8_t		gyro_z[2];
		uint8_t		accel_z[2];
		uint8_t		accel_y[2];
		uint8_t		accel_x[2];
		uint8_t		accel_x2[2];
		uint8_t		temp[2];
	};
#pragma pack(pop)
};

/**
 * Helper class implementing the gyro driver node.
 */
class MAX1168_gyro : public device::CDev
{
public:
	MAX1168_gyro(MAX1168 *parent, const char *path);
	~MAX1168_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class MAX1168;

	void			parent_poll_notify();

private:
	MAX1168			*_parent;
	orb_advert_t		_gyro_topic;
	int			_gyro_orb_class_instance;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	MAX1168_gyro(const MAX1168_gyro &);
	MAX1168_gyro operator=(const MAX1168_gyro &);
};

/** driver 'main' command */
extern "C" { __EXPORT int max1168_main(int argc, char *argv[]); }

MAX1168::MAX1168(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
			SPI("MAX1168", path_accel, bus, device, SPIDEV_MODE0, MAX1168_BUS_SPEED),
			_work{},
			_measure_ticks(0),
			_gyro(new MAX1168_gyro(this, path_gyro)),
			_product(0),
			_collect_phase(false),
			_call{},
			_call_interval(0),
			_accel_reports(nullptr),
			_accel_scale{},
			_accel_range_scale(0.0f),
			_accel_range_m_s2(0.0f),
			_accel_topic(nullptr),
			_accel_orb_class_instance(-1),
			_accel_class_instance(-1),
			_gyro_reports(nullptr),
			_gyro_scale{},
			_gyro_range_scale(0.0f),
			_gyro_range_rad_s(0.0f),
			_sample_rate(1000),
			_accel_reads(perf_alloc(PC_COUNT, "MAX1168_accel_read")),
			_gyro_reads(perf_alloc(PC_COUNT, "MAX1168_gyro_read")),
			_sample_perf(perf_alloc(PC_ELAPSED, "MAX1168_read")),
			_bad_transfers(perf_alloc(PC_COUNT, "MAX1168_bad_transfers")),
			_bad_registers(perf_alloc(PC_COUNT, "MAX1168_bad_registers")),
			_good_transfers(perf_alloc(PC_COUNT, "MAX1168_good_transfers")),
			_reset_retries(perf_alloc(PC_COUNT, "MAX1168_reset_retries")),
			_duplicates(perf_alloc(PC_COUNT, "MAX1168_duplicates")),
			_system_latency_perf(perf_alloc_once(PC_ELAPSED, "sys_latency")),
			_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
			_register_wait(0),
			_reset_wait(0),
			_accel_filter_x(MAX1168_ACCEL_DEFAULT_RATE, MAX1168_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
			_accel_filter_y(MAX1168_ACCEL_DEFAULT_RATE, MAX1168_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
			_accel_filter_z(MAX1168_ACCEL_DEFAULT_RATE, MAX1168_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
			_gyro_filter_x(MAX1168_GYRO_DEFAULT_RATE, MAX1168_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
			_gyro_filter_y(MAX1168_GYRO_DEFAULT_RATE, MAX1168_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
			_gyro_filter_z(MAX1168_GYRO_DEFAULT_RATE, MAX1168_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
			_accel_int(1000000 / MAX1168_ACCEL_MAX_OUTPUT_RATE),
			_gyro_int(1000000 / MAX1168_GYRO_MAX_OUTPUT_RATE, true),
			_rotation(rotation),
			_in_factory_test(false),
			_last_temperature(0),
			_last_accel{},
			_got_duplicate(false)
			{
				// disable debug() calls
				_debug_enabled = false;

				_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MAX1168;

				/* Prime _gyro with parents devid. */
				_gyro->_device_id.devid = _device_id.devid;
				_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MAX1168;

				// default accel scale factors
				_accel_scale.x_offset = 0;
				_accel_scale.x_scale  = 1.0f;
				_accel_scale.y_offset = 0;
				_accel_scale.y_scale  = 1.0f;
				_accel_scale.z_offset = 0;
				_accel_scale.z_scale  = 1.0f;

				// default gyro scale factors
				_gyro_scale.x_offset = 0;
				_gyro_scale.x_scale  = 1.0f;
				_gyro_scale.y_offset = 0;
				_gyro_scale.y_scale  = 1.0f;
				_gyro_scale.z_offset = 0;
				_gyro_scale.z_scale  = 1.0f;

				memset(&_call, 0, sizeof(_call));
			}

			MAX1168::~MAX1168()
			{
				/* make sure we are truly inactive */
				stop();

				/* delete the gyro subdriver */
				delete _gyro;

				/* free any existing reports */
				if (_accel_reports != nullptr) {
					delete _accel_reports;
				}

				if (_gyro_reports != nullptr) {
					delete _gyro_reports;
				}

				if (_accel_class_instance != -1) {
					unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
				}

				/* delete the perf counter */
				perf_free(_sample_perf);
				perf_free(_accel_reads);
				perf_free(_gyro_reads);
				perf_free(_bad_transfers);
				perf_free(_bad_registers);
				perf_free(_good_transfers);
				perf_free(_reset_retries);
				perf_free(_duplicates);
			}

			int
			MAX1168::init()
			{
				int ret;

				/* do SPI init (and probe) first */
				ret = SPI::init();

				/* if probe/setup failed, bail now */
				if (ret != OK) {
					DEVICE_DEBUG("SPI setup failed");
					return ret;
				}

				/* allocate basic report buffers */
				_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

				if (_accel_reports == nullptr) {
					goto out;
				}

				_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

				if (_gyro_reports == nullptr) {
					goto out;
				}

				/* Initialize offsets and scales */
				_accel_scale.x_offset = 0;
				_accel_scale.x_scale  = 1.0f;
				_accel_scale.y_offset = 0;
				_accel_scale.y_scale  = 1.0f;
				_accel_scale.z_offset = 0;
				_accel_scale.z_scale  = 1.0f;

				_gyro_scale.x_offset = 0;
				_gyro_scale.x_scale  = 1.0f;
				_gyro_scale.y_offset = 0;
				_gyro_scale.y_scale  = 1.0f;
				_gyro_scale.z_offset = 0;
				_gyro_scale.z_scale  = 1.0f;

				// correct gyro scale factors
				// scale to rad/s in SI units
				// 300 deg/s = (300/180)*PI = 5.23598775 rad/s
				// scaling factor:
				// 1/(2^15)*(300/180)*PI
				_gyro_range_scale = (300/ 180.0f) * (float)M_PI / (1.8f * (GYRO_REF / 5.0f)) ;//5v  0.006v =1 deg/s
				_gyro_range_rad_s = (300.0f / 180.0f) * (float)M_PI;

				_accel_range_scale =  MAX1168_ONE_G /( ACCEL_REF/10);    // 3v = 0.3v/g
				_accel_range_m_s2 = 3.0f * MAX1168_ONE_G;

				/* do CDev init for the gyro device node, keep it optional */
				ret = _gyro->init();

				/* if probe/setup failed, bail now */
				if (ret != OK) {
					DEVICE_DEBUG("gyro init failed");
					return ret;
				}

				_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

				trigmeas();
				usleep(500);
				measure();

				/* advertise sensor topic, measure manually to initialize valid report */
				struct accel_report arp;
				_accel_reports->get(&arp);

				/* measurement will have generated a report, publish */
				_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
						&_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

				if (_accel_topic == nullptr) {
					warnx("ADVERT FAIL");
				}


				/* advertise sensor topic, measure manually to initialize valid report */
				struct gyro_report grp;
				_gyro_reports->get(&grp);

				_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
						&_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

				if (_gyro->_gyro_topic == nullptr) {
					warnx("ADVERT FAIL");
				}
				out:
				return ret;
			}
			int
			MAX1168::probe()
			{
				_product=0x01;
				return OK;
			}

			/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
			 */
			void
			MAX1168::_set_sample_rate(unsigned desired_sample_rate_hz)
			{

			}
			ssize_t
			MAX1168::read(struct file *filp, char *buffer, size_t buflen)
			{
				unsigned count = buflen / sizeof(accel_report);

				/* buffer must be large enough */
				if (count < 1) {
					return -ENOSPC;
				}

				/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
				if (_call_interval == 0) {
					_accel_reports->flush();
					//measure();
				}

				/* if no data, error (we could block here) */
				if (_accel_reports->empty()) {
					return -EAGAIN;
				}

				perf_count(_accel_reads);

				/* copy reports out of our buffer to the caller */
				accel_report *arp = reinterpret_cast<accel_report *>(buffer);
				int transferred = 0;

				while (count--) {
					if (!_accel_reports->get(arp)) {
						break;
					}

					transferred++;
					arp++;
				}

				/* return the number of bytes transferred */
				return (transferred * sizeof(accel_report));
			}

			int
			MAX1168::self_test()
			{
				if (perf_event_count(_sample_perf) == 0) {
					measure();
				}

				/* return 0 on success, 1 else */
				return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
			}
			int
			MAX1168::accel_self_test()
			{
				if (self_test()) {
					return 1;
				}

				/* inspect accel offsets */
				if (fabsf(_accel_scale.x_offset) < 0.000001f) {
					return 1;
				}

				if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f) {
					return 1;
				}

				if (fabsf(_accel_scale.y_offset) < 0.000001f) {
					return 1;
				}

				if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f) {
					return 1;
				}

				if (fabsf(_accel_scale.z_offset) < 0.000001f) {
					return 1;
				}

				if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f) {
					return 1;
				}

				return 0;
			}

			ssize_t
			MAX1168::gyro_read(struct file *filp, char *buffer, size_t buflen)
			{
				unsigned count = buflen / sizeof(gyro_report);

				/* buffer must be large enough */
				if (count < 1) {
					return -ENOSPC;
				}
				/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
				if (_call_interval == 0) {
					_gyro_reports->flush();
					//measure();
				}

				/* if no data, error (we could block here) */
				if (_gyro_reports->empty()) {
					return -EAGAIN;
				}

				perf_count(_gyro_reads);

				/* copy reports out of our buffer to the caller */
				gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
				int transferred = 0;

				while (count--) {
					if (!_gyro_reports->get(grp)) {
						break;
					}

					transferred++;
					grp++;
				}

				/* return the number of bytes transferred */
				return (transferred * sizeof(gyro_report));
			}
			int
			MAX1168::ioctl(struct file *filp, int cmd, unsigned long arg)
			{
				switch (cmd) {

				case SENSORIOCSPOLLRATE: {
					switch (arg) {

					/* switching to manual polling */
					case SENSOR_POLLRATE_MANUAL:
						stop();
						_call_interval = 0;
						return OK;

						/* external signalling not supported */
					case SENSOR_POLLRATE_EXTERNAL:

						/* zero would be bad */
					case 0:
						return -EINVAL;

						/* set default/max polling rate */
					case SENSOR_POLLRATE_MAX:
						return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

					case SENSOR_POLLRATE_DEFAULT:
						return ioctl(filp, SENSORIOCSPOLLRATE, MAX1168_ACCEL_DEFAULT_RATE);

						/* adjust to a legal polling interval in Hz */
					default: {
						/* do we need to start internal polling? */
						bool want_start = (_call_interval == 0);

						/* convert hz to hrt interval via microseconds */
						unsigned ticks = 1000000 / arg;

						/* check against maximum sane rate */
						if (ticks < 1000)
							return -EINVAL;

						// adjust filters
						float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
						float sample_rate = 1.0e6f/ticks;

						_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
						_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
						_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


						float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
						_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
						_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
						_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

						/* update interval for next measurement */
						/* XXX this is a bit shady, but no other way to adjust... */
						_call_interval = ticks;
						/*
	                                          set call interval faster then the sample time. We
	                                          then detect when we have duplicate samples and reject
	                                          them. This prevents aliasing due to a beat between the
	                                          stm32 clock and the mpu6000 clock
						 */
						_call.period = _call_interval;
						warnx("_call_interval=%d",_call_interval);
						/* if we need to start the poll state machine, do it */
						if (want_start)
							start();

						return OK;
					}
					}
				}

				case SENSORIOCGPOLLRATE:
					if (_call_interval == 0)
						return SENSOR_POLLRATE_MANUAL;

					return 1000000 / _call_interval;

				case SENSORIOCGQUEUEDEPTH:
					return _accel_reports->size();

				case ACCELIOCGSAMPLERATE:
					return _sample_rate;

				case ACCELIOCSSAMPLERATE:
					_set_sample_rate(arg);
					return OK;

				case ACCELIOCGLOWPASS:
					return _accel_filter_x.get_cutoff_freq();

				case ACCELIOCSLOWPASS:
					// set hardware filtering
					//	_set_dlpf_filter(arg);
					// set software filtering
					_accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					_accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					_accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					return OK;

				case ACCELIOCSSCALE: {
					/* copy scale, but only if off by a few percent */
					struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
					float sum = s->x_scale + s->y_scale + s->z_scale;

					if (sum > 2.0f && sum < 4.0f) {
						memcpy(&_accel_scale, s, sizeof(_accel_scale));
						return OK;

					} else {
						return -EINVAL;
					}
				}

				case ACCELIOCGSCALE:
					/* copy scale out */
					memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
					return OK;

				case ACCELIOCSRANGE:
					//		return set_accel_range(arg);

				case ACCELIOCGRANGE:
					return (unsigned long)((_accel_range_m_s2) / MAX1168_ONE_G + 0.5f);

				case ACCELIOCSELFTEST:
					return accel_self_test();

				default:
					/* give it to the superclass */
					return SPI::ioctl(filp, cmd, arg);
				}
			}
			int
			MAX1168::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
			{

				switch (cmd) {

				/* these are shared with the accel side */
				case SENSORIOCSPOLLRATE:
				case SENSORIOCRESET:
					return ioctl(filp, cmd, arg);

				case SENSORIOCGQUEUEDEPTH:
					return _gyro_reports->size();

				case GYROIOCGSAMPLERATE:
					return _sample_rate;

				case GYROIOCSSAMPLERATE:
					_set_sample_rate(arg);
					return OK;

				case GYROIOCGLOWPASS:
					return _gyro_filter_x.get_cutoff_freq();

				case GYROIOCSLOWPASS:
					// set hardware filtering
					//	_set_dlpf_filter(arg);
					_gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					_gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					_gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
					return OK;

				case GYROIOCSSCALE:
					/* copy scale in */
					memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
					return OK;

				case GYROIOCGSCALE:
					/* copy scale out */
					memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
					return OK;

				case GYROIOCSRANGE:
					/* XXX not implemented */
					// XXX change these two values on set:
					// _gyro_range_scale = xx
					// _gyro_range_rad_s = xx
					return -EINVAL;

				case GYROIOCGRANGE:
					return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

				case GYROIOCSELFTEST:
					return OK;//gyro_self_test();

				default:
					/* give it to the superclass */
					return SPI::ioctl(filp, cmd, arg);
				}
				return OK;
			}

			uint8_t
			MAX1168::read_reg(unsigned reg, uint32_t speed)
			{
				return OK;
			}
			void
			MAX1168::write_reg(unsigned reg, uint8_t value)
			{
				uint8_t	cmd;

				cmd= reg | DIR_WRITE;

				// general register transfer at low clock speed
				//	set_frequency(MAX1168_BUS_SPEED);

				transfer(&cmd, nullptr, 1);
			}

			void
			MAX1168::start()
			{
				/* make sure we are stopped first */
				stop();

				/* discard any stale data in the buffers */
				//	_accel_reports->flush();
				//	_gyro_reports->flush();

				/* reset the report ring and state machine */

				// sensor transfer at high clock speed
				set_frequency(MAX1168_BUS_SPEED);
				set_bits(16);
				/* schedule a cycle to start things */
				//	work_queue(HPWORK, &_work, (worker_t)&MAX1168::cycle_trampoline, this, 1);
				trigmeas();
				hrt_call_every(&_call,
						1800,
						_call_interval,
						(hrt_callout)&MAX1168::cycle_trampoline, this);

			}

			void
			MAX1168::stop()
			{
				//	hrt_cancel(&_call);

				/* reset internal states */
				memset(_last_accel, 0, sizeof(_last_accel));

				/* discard unread data in the buffers */
				_accel_reports->flush();
				_gyro_reports->flush();
			}
			void
			MAX1168::cycle_trampoline(void *arg)
			{
				MAX1168 *dev = reinterpret_cast<MAX1168 *>(arg);

				/* make another measurement */
				dev->cycle();
			}
			void
			MAX1168::trigmeas()
			{
				uint16_t	cmd;

				//cmd[0] = DIR_WRITE;
				cmd = MAX1168_CONF_CR<<8;

				// general register transfer at low clock speed

				transfer_sel((uint8_t *)&cmd, nullptr,2,0);
			}
			void
			MAX1168::cycle()
			{
				//uint64_t timestamp = hrt_absolute_time();
				//if((timestamp-_reset_wait)>500)
				//{
				//_reset_wait=timestamp;
				//
				measure();
				trigmeas();

				//}
			}
			void
			MAX1168::measure()
			{
				//	struct MAXReport max_report;
				static int i=0;
				i++;
				struct Report {
					_uint16_t gyro_y;
					_uint16_t gyro_x;
					_uint16_t gyro_z;
					_uint16_t accel_z;
					_uint16_t accel_y;
					_uint16_t accel_x;
					_uint16_t accel_x2;
					_uint16_t temp;
				} report;
				//uint8_t test[16]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};
				/* start measuring */
				perf_begin(_sample_perf);

				/*
				 * Fetch the full set of measurements from the MAX1168 in one pass.
				 */
				//	max_report.cmd = DIR_READ;
				/*if (OK!= transfer((uint8_t *) &max_report, ((uint8_t *) &max_report),
					sizeof(max_report))) {
						return;
				}*/
				transfer_sel(nullptr, (uint8_t *) &report,8,1);
				/*
				 * Convert from big to little endian
				 */
				//	memcpy(&report,&max_report,16);


				if (report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0
						&& report.temp == 0 && report.gyro_x == 0 && report.gyro_y == 0
						&& report.gyro_z == 0) {
					// all zero data - probably a SPI bus error
					perf_count(_bad_transfers);
					perf_end(_sample_perf);
					// note that we don't call reset() here as a reset()
					// costs 20ms with interrupts disabled. That means if
					// the mpu6k does go bad it would cause a FMU failure,
					// regardless of whether another sensor is available,
					return;
				}

				/*
				 * Swap axes and negate y
				 */
				//	int16_t accel_xt = report.accel_y;
				//	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

				//	int16_t gyro_xt = report.gyro_y;
				//	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

				/*
				 * Apply the swap
				 */
				//	report.accel_x = accel_xt;
				//	report.accel_y = accel_yt;
				//	report.gyro_x = gyro_xt;
				//	report.gyro_y = gyro_yt;
				/*
				 * Report buffers.
				 */
				accel_report arb;
				gyro_report grb;

				/*
				 * Adjust and scale results to m/s^2.
				 */
				grb.timestamp = arb.timestamp = hrt_absolute_time();
				// report the error count as the sum of the number of bad
				// transfers and bad register reads. This allows the higher
				// level code to decide if it should use this sensor based on
				// whether it has had failures
				grb.error_count = arb.error_count = perf_event_count(_bad_transfers)
																																			+ perf_event_count(_bad_registers);

				/*
				 * 1) Scale raw value to SI units using scaling from datasheet.
				 * 2) Subtract static offset (in SI units)
				 * 3) Scale the statically calibrated values with a linear
				 *    dynamically obtained factor
				 *
				 * Note: the static sensor offset is the number the sensor outputs
				 * 	 at a nominally 'zero' input. Therefore the offset has to
				 * 	 be subtracted.
				 *
				 *	 Example: A gyro outputs a value of 74 at zero angular rate
				 *	 	  the offset is 74 from the origin and subtracting
				 *		  74 from all measurements centers them around zero.
				 */

				/* NOTE: Axes have been swapped to match the board a few lines above. */

				arb.x_raw = report.accel_x;
				arb.y_raw = report.accel_y;
				arb.z_raw = report.accel_z;

				float xraw_f = (float)report.accel_x/65536*ADVOLT_REF;
				float yraw_f = (float)report.accel_y/65536*ADVOLT_REF;
				float zraw_f = (float)report.accel_z/65536*ADVOLT_REF;
				//printf("ax=%.3f,ay=%.3f,az=%.3f\n",(double)xraw_f,(double)yraw_f,(double)zraw_f);

				// apply user specified rotation
				rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

				float x_in_new = (((xraw_f-ACCEL_REF/2) * _accel_range_scale) - _accel_scale.x_offset)  * _accel_scale.x_scale;
				float y_in_new = -(((yraw_f-ACCEL_REF/2) * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
				float z_in_new = -(((zraw_f-ACCEL_REF/2) * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
//				if(i%20==0)
//				{printf("ax=%.3f,ay=%.3f,az=%.3f\n",(double)x_in_new,(double)y_in_new,(double)z_in_new);}
				arb.x = _accel_filter_x.apply(x_in_new);
				arb.y = _accel_filter_y.apply(y_in_new);
				arb.z = _accel_filter_z.apply(z_in_new);
				math::Vector < 3 > aval(x_in_new, y_in_new, z_in_new);
				math::Vector < 3 > aval_integrated;

				bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated,
						arb.integral_dt);
				arb.x_integral = aval_integrated(0);
				arb.y_integral = aval_integrated(1);
				arb.z_integral = aval_integrated(2);

				arb.scaling = _accel_range_scale;
				arb.range_m_s2 = _accel_range_m_s2;

				_last_temperature = (report.temp) / 361.0f + 35.0f;

				arb.temperature_raw = report.temp;
				arb.temperature = _last_temperature;

				/* return device ID */
				arb.device_id = _device_id.devid;

				grb.x_raw = report.gyro_x;
				grb.y_raw = report.gyro_y;
				grb.z_raw = report.gyro_z;

				xraw_f = (float)report.gyro_x/65536*ADVOLT_REF;
				yraw_f = (float)report.gyro_y/65536*ADVOLT_REF;
				zraw_f = (float)report.gyro_z/65536*ADVOLT_REF;
				//printf("gx=%.3f,gy=%.3f,gz=%.3f\n",(double)xraw_f,(double)yraw_f,(double)zraw_f);
				// apply user specified rotation
				rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

				//  swap xy and negative y
				float y_gyro_in_new = -(((xraw_f-GYRO_REF/2) * _gyro_range_scale) - _gyro_scale.x_offset)* _gyro_scale.x_scale;
				float x_gyro_in_new = (((yraw_f-GYRO_REF/2) * _gyro_range_scale) - _gyro_scale.y_offset)* _gyro_scale.y_scale;
				float z_gyro_in_new = (((zraw_f-GYRO_REF/2) * _gyro_range_scale) - _gyro_scale.z_offset)* _gyro_scale.z_scale;
				//printf("gx=%.3f,gy=%.3f,gz=%.3f\n",(double)x_gyro_in_new,(double)y_gyro_in_new,(double)z_gyro_in_new);
				grb.x = _gyro_filter_x.apply(x_gyro_in_new);
				grb.y = _gyro_filter_y.apply(y_gyro_in_new);
				grb.z = _gyro_filter_z.apply(z_gyro_in_new);

				math::Vector < 3 > gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
				math::Vector < 3 > gval_integrated;

				bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated,
						grb.integral_dt);
				grb.x_integral = gval_integrated(0);
				grb.y_integral = gval_integrated(1);
				grb.z_integral = gval_integrated(2);

				grb.scaling = _gyro_range_scale;
				grb.range_rad_s = _gyro_range_rad_s;

				grb.temperature_raw = report.temp;
				grb.temperature = _last_temperature;

				/* return device ID */
				grb.device_id = _gyro->_device_id.devid;
				_accel_reports->force(&arb);
				_gyro_reports->force(&grb);

				/* notify anyone waiting for data */
				if (accel_notify) {
					poll_notify(POLLIN);
				}

				if (gyro_notify) {
					_gyro->parent_poll_notify();
				}
				/* advertise sensor topic, measure manually to initialize valid report */
				//struct gyro_report grp;
				//_gyro_reports->get(&grp);
				if (accel_notify && !(_pub_blocked)) {
					/* log the time of this report */
					perf_begin(_controller_latency_perf);
					/* publish it */
					orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
				}

				if (gyro_notify && !(_pub_blocked)) {
					/* publish it */
					orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
				}
				/* stop measuring */
				perf_end(_sample_perf);
			}

			void
			MAX1168::print_info()
			{
				perf_print_counter(_sample_perf);
				perf_print_counter(_accel_reads);
				perf_print_counter(_gyro_reads);
				perf_print_counter(_bad_transfers);
				perf_print_counter(_bad_registers);
				perf_print_counter(_good_transfers);
				perf_print_counter(_reset_retries);
				perf_print_counter(_duplicates);
				_accel_reports->print_info("accel queue");
				_gyro_reports->print_info("gyro queue");

				::printf("temperature: %.1f\n", (double)_last_temperature);
			}

			MAX1168_gyro::MAX1168_gyro(MAX1168 *parent, const char *path) :
			CDev("MAX1168_gyro", path),
			_parent(parent),
			_gyro_topic(nullptr),
			_gyro_orb_class_instance(-1),
			_gyro_class_instance(-1)
			{
			}

			MAX1168_gyro::~MAX1168_gyro()
			{
				if (_gyro_class_instance != -1) {
					unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
				}
			}

			int
			MAX1168_gyro::init()
			{
				int ret;
				// do base class init
				ret = CDev::init();

				/* if probe/setup failed, bail now */
				if (ret != OK) {
					DEVICE_DEBUG("gyro init failed");
					return ret;
				}

				_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

				return ret;
			}

			void
			MAX1168_gyro::parent_poll_notify()
			{
				poll_notify(POLLIN);
			}
			ssize_t
			MAX1168_gyro::read(struct file *filp, char *buffer, size_t buflen)
			{
				return _parent->gyro_read(filp, buffer, buflen);
			}

			int
			MAX1168_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
			{

				switch (cmd) {
				case DEVIOCGDEVICEID:
					return (int)CDev::ioctl(filp, cmd, arg);
					break;

				default:
					return _parent->gyro_ioctl(filp, cmd, arg);
				}
			}

			/**
			 * Local functions in support of the shell command.
			 */
			namespace max1168
			{

			MAX1168	*g_dev_int; // on internal bus
			MAX1168	*g_dev_ext; // on external bus

			void	start(bool, enum Rotation, int range);
			void	stop(bool);
			void	test(bool);
			void	reset(bool);
			void	info(bool);
			void	factorytest(bool);
			void	usage();

			/**
			 * Start the driver.
			 *
			 * This function only returns if the driver is up and running
			 * or failed to detect the sensor.
			 */
			void
			start(bool external_bus, enum Rotation rotation, int range)
			{
				int fd;
				MAX1168 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
				const char *path_accel = external_bus ? MAX_DEVICE_PATH_ACCEL_EXT : MAX_DEVICE_PATH_ACCEL;
				const char *path_gyro  = external_bus ? MAX_DEVICE_PATH_GYRO_EXT : MAX_DEVICE_PATH_GYRO;

				if (*g_dev_ptr != nullptr)
					/* if already started, the still command succeeded */
				{
					errx(0, "already started");
				}

				/* create the driver */
				if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
					*g_dev_ptr = new MAX1168(PX4_SPI_BUS_EXT, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT_MPU, rotation);
#else
					errx(0, "External SPI not available");
#endif

				} else {
					warnx("init already stopped.");
					*g_dev_ptr = new MAX1168(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_GYRO, rotation);
				}

				if (*g_dev_ptr == nullptr) {
					goto fail;
				}

				if (OK != (*g_dev_ptr)->init()) {
					goto fail;
				}

				/* set the poll rate to default, starts automatic data collection */
				fd = open(path_accel, O_RDONLY);

				if (fd < 0) {
					goto fail;
				}

				if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
					goto fail;
				}

				close(fd);

				exit(0);
				fail:

				if (*g_dev_ptr != nullptr) {
					delete(*g_dev_ptr);
					*g_dev_ptr = nullptr;
				}

				errx(1, "driver start failed");
			}

			void
			stop(bool external_bus)
			{
				MAX1168 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

				if (*g_dev_ptr != nullptr) {
					delete *g_dev_ptr;
					*g_dev_ptr = nullptr;

				} else {
					/* warn, but not an error */
					warnx("already stopped.");
				}

				exit(0);
			}

			/**
			 * Perform some basic functional tests on the driver;
			 * make sure we can collect data from the sensor in polled
			 * and automatic modes.
			 */
			void
			test(bool external_bus)
			{
				const char *path_accel = external_bus ? MAX_DEVICE_PATH_ACCEL_EXT : MAX_DEVICE_PATH_ACCEL;
				const char *path_gyro  = external_bus ? MAX_DEVICE_PATH_GYRO_EXT : MAX_DEVICE_PATH_GYRO;
				accel_report a_report;
				gyro_report g_report;
				ssize_t sz;

				/* get the driver */
				int fd = open(path_accel, O_RDONLY);

				if (fd < 0)
					err(1, "%s open failed (try 'MAX1168 start')",
							path_accel);

				/* get the driver */
				int fd_gyro = open(path_gyro, O_RDONLY);

				if (fd_gyro < 0) {
					err(1, "%s open failed", path_gyro);
				}


				/* do a simple demand read */
				sz = read(fd, &a_report, sizeof(a_report));

				if (sz != sizeof(a_report)) {
					warnx("ret: %d, expected: %d", sz, sizeof(a_report));
					err(1, "immediate acc read failed");
				}

				warnx("single read");
				warnx("time:     %lld", a_report.timestamp);
				warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
				warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
				warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
				uint16_t x_raw=a_report.x_raw;
				uint16_t y_raw=a_report.y_raw;
				uint16_t z_raw=a_report.z_raw;
				warnx("acc  x:  \t%8.4fv\traw 0x%0x", (double)((float)x_raw/65536*ADVOLT_REF), (unsigned short)a_report.x_raw);
				warnx("acc  y:  \t%8.4fv\traw 0x%0x", (double)((float)y_raw/65536*ADVOLT_REF), (unsigned short)a_report.y_raw);
				warnx("acc  z:  \t%8.4fv\traw 0x%0x", (double)((float)z_raw/65536*ADVOLT_REF), (unsigned short)a_report.z_raw);
				warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
						(double)(a_report.range_m_s2 / MAX1168_ONE_G));

				/* do a simple demand read */
				sz = read(fd_gyro, &g_report, sizeof(g_report));

				if (sz != sizeof(g_report)) {
					warnx("ret: %d, expected: %d", sz, sizeof(g_report));
					err(1, "immediate gyro read failed");
				}

				warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
				warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
				warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
				x_raw=g_report.x_raw;
				y_raw=g_report.y_raw;
				z_raw=g_report.z_raw;
				warnx("gyro  x:  \t%8.4fv\traw %d", (double)((float)(x_raw)/65536*ADVOLT_REF), x_raw);
				warnx("gyro  y:  \t%8.4fv\traw %d", (double)((float)(y_raw)/65536*ADVOLT_REF), y_raw);
				warnx("gyro  z:  \t%8.4fv\traw %d", (double)((float)(z_raw)/65536*ADVOLT_REF), z_raw);
				warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
						(int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

				warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
				warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);

				/* reset to default polling */
				if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
					err(1, "reset to default polling");
				}

				close(fd);
				close(fd_gyro);

				/* XXX add poll-rate tests here too */

				reset(external_bus);
				errx(0, "PASS");
			}

			/**
			 * Reset the driver.
			 */
			void
			reset(bool external_bus)
			{
				const char *path_accel = external_bus ? MAX_DEVICE_PATH_ACCEL_EXT : MAX_DEVICE_PATH_ACCEL;
				int fd = open(path_accel, O_RDONLY);

				if (fd < 0) {
					err(1, "failed ");
				}

				if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
					err(1, "driver reset failed");
				}

				if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
					err(1, "driver poll restart failed");
				}

				close(fd);

				exit(0);
			}

			/**
			 * Print a little info about the driver.
			 */
			void
			info(bool external_bus)
			{
				MAX1168 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

				if (*g_dev_ptr == nullptr) {
					errx(1, "driver not running");
				}

				printf("state @ %p\n", *g_dev_ptr);
				(*g_dev_ptr)->print_info();

				exit(0);
			}

			/**
			 * Dump the register information
			 */
			void
			factorytest(bool external_bus)
			{
			}

			void
			usage()
			{
				warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset','factorytest'");
				warnx("options:");
				warnx("    -X    (external bus)");
				warnx("    -R rotation");
				warnx("    -a accel range (in g)");
			}

			} // namespace

			int
			max1168_main(int argc, char *argv[])
			{
				bool external_bus = false;
				int ch;
				enum Rotation rotation = ROTATION_NONE;
				int accel_range = 8;

				/* jump over start/off/etc and look at options first */
				while ((ch = getopt(argc, argv, "XR:a:")) != EOF) {
					switch (ch) {
					case 'X':
						external_bus = true;
						break;

					case 'R':
						rotation = (enum Rotation)atoi(optarg);
						break;

					case 'a':
						accel_range = atoi(optarg);
						break;

					default:
						max1168::usage();
						exit(0);
					}
				}

				const char *verb = argv[optind];

				/*
				 * Start/load the driver.

				 */
				if (!strcmp(verb, "start")) {
					max1168::start(external_bus, rotation, accel_range);
				}

				if (!strcmp(verb, "stop")) {
					max1168::stop(external_bus);
				}

				/*
				 * Test the driver/device.
				 */
				if (!strcmp(verb, "test")) {
					max1168::test(external_bus);
				}

				/*
				 * Reset the driver.
				 */
				if (!strcmp(verb, "reset")) {
					max1168::reset(external_bus);
				}

				/*
				 * Print driver information.
				 */
				if (!strcmp(verb, "info")) {
					max1168::info(external_bus);
				}

				if (!strcmp(verb, "factorytest")) {
					max1168::factorytest(external_bus);
				}

				max1168::usage();
				exit(1);
			}
