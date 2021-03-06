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
 * @file ms5611.cpp
 * Driver for the MS5611 barometric pressure sensor connected via I2C or SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>

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

#include <px4_workqueue.h>
#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include "ms5611.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

//#ifndef CONFIG_SCHED_WORKQUEUE
//# error This requires CONFIG_SCHED_WORKQUEUE.
//#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MS5611 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_CONVERSION_INTERVAL	40000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */
#define MS5611_BARO_DEVICE_PATH_EXT	"/dev/ms5611_ext"
#define MS5611_BARO_DEVICE_PATH_INT	"/dev/ms5611_int"

class MS5611 : public device::VDev
{
public:
	MS5611(device::Device *interface, ms5611::prom_u &prom_buf, const char *path);
	~MS5611();

	virtual int		init();

	virtual ssize_t		read(device::file_t *handlep, char *buffer, size_t buflen);
	virtual int		ioctl(device::file_t *handlep, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

	ms5611::prom_s		_prom;

	struct work_s		_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* intermediate temperature values per MS5611 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;
	float			_P;
	float			_T;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5611_main(int argc, char *argv[]);

MS5611::MS5611(device::Device *interface, ms5611::prom_u &prom_buf, const char *path) :
	VDev("MS5611", path),
	_interface(interface),
	_prom(prom_buf.s),
	_measure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5611_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5611_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ms5611_buffer_overflows"))
{
	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

MS5611::~MS5611()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

	delete _interface;
}

int
MS5611::init()
{
	int ret;
	warnx("MS5611::init");

	ret = VDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("VDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr) {
		DEVICE_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}
	
	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	struct baro_report brp;
	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;
	_reports->flush();

	/* this do..while is goto without goto */
	do {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			warnx("temp measure failed");
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			warnx("pressure collect failed");
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			warnx("pressure collect failed");
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		ret = OK;

		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
						  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

		if (_baro_topic == nullptr) {
			warnx("failed to create sensor_baro publication");
		}

		//warnx("sensor_baro publication %ld", _baro_topic);

	} while (0);

out:
	return ret;
}

ssize_t
MS5611::read(device::file_t *handlep, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		_reports->flush();

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(brp)) {
			ret = sizeof(*brp);
		}

	} while (0);

	return ret;
}

int
MS5611::ioctl(device::file_t *handlep, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(MS5611_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if ((unsigned long)ticks < USEC2TICK(MS5611_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start_cycle();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			if (!_reports->resize(arg)) {
				return -ENOMEM;
			}

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return VDev::ioctl(handlep, cmd, arg);
}

void
MS5611::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MS5611::cycle_trampoline, this, 1);
}

void
MS5611::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
MS5611::cycle_trampoline(void *arg)
{
	MS5611 *dev = reinterpret_cast<MS5611 *>(arg);

	dev->cycle();
}

void
MS5611::cycle()
{
	int ret;
	unsigned dummy=0;      //set the value by yaoling

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_interface->ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again */
			start_cycle();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    ((unsigned long)_measure_ticks > USEC2TICK(MS5611_CONVERSION_INTERVAL))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MS5611::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MS5611_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		//DEVICE_LOG("measure error %d", ret);
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start_cycle();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MS5611::cycle_trampoline,
		   this,
		   USEC2TICK(MS5611_CONVERSION_INTERVAL));
}

int
MS5611::measure()
{
	int ret;

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
MS5611::collect()
{
	int ret;
	uint32_t raw;

	perf_begin(_sample_perf);

	struct baro_report report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);
		_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);

		/* temperature compensation */
		if (_TEMP < 2000) {

			int32_t T2 = POW2(dT) >> 31;

			int64_t f = POW2((int64_t)_TEMP - 2000);
			int64_t OFF2 = 5 * f >> 1;
			int64_t SENS2 = 5 * f >> 2;

			if (_TEMP < -1500) {
				int64_t f2 = POW2(_TEMP + 1500);
				OFF2 += 7 * f2;
				SENS2 += 11 * f2 >> 1;
			}

			_TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;
		}

	} else {

		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;
		_P = P * 0.01f;
		_T = _TEMP * 0.01f;

		/* generate a new report */
		report.temperature = _TEMP / 100.0f;
		report.pressure = P / 100.0f;		/* convert to millibar */

		/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
		 */

		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		double p = P / 1000.0;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

		/* publish it */
		if (!(_pub_blocked)) {
			if (_baro_topic != (orb_advert_t)(-1)) {
				/* publish it */
				orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);

			} else {
				printf("MS5611::collect _baro_topic not initialized\n");
			}
		}

		if (_reports->force(&report)) {
			perf_count(_buffer_overflows);
		}

		/* notify anyone waiting for data */
		poll_notify(POLLIN);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
MS5611::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", (long long)_SENS);
	printf("OFF:            %lld\n", (long long)_OFF);
	printf("P:              %.3f\n", (double)_P);
	printf("T:              %.3f\n", (double)_T);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));

	printf("factory_setup             %u\n", _prom.factory_setup);
	printf("c1_pressure_sens          %u\n", _prom.c1_pressure_sens);
	printf("c2_pressure_offset        %u\n", _prom.c2_pressure_offset);
	printf("c3_temp_coeff_pres_sens   %u\n", _prom.c3_temp_coeff_pres_sens);
	printf("c4_temp_coeff_pres_offset %u\n", _prom.c4_temp_coeff_pres_offset);
	printf("c5_reference_temp         %u\n", _prom.c5_reference_temp);
	printf("c6_temp_coeff_temp        %u\n", _prom.c6_temp_coeff_temp);
	printf("serial_and_crc            %u\n", _prom.serial_and_crc);
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

/*
  list of supported bus configurations
 */
struct ms5611_bus_option {
	enum MS5611_BUS busid;
	const char *devpath;
	MS5611_constructor interface_constructor;
	uint8_t busnum;
	uint8_t chipselect;
	MS5611 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MS5611_BUS_SPI_EXTERNAL, "/dev/ms5611_spi_ext", &MS5611_spi_interface, PX4_SPI_BUS_EXT,0, NULL },
#endif
#if 0
#ifdef PX4_SPIDEV_BARO
	{ MS5611_BUS_SPI_INTERNAL, "/dev/ms5611_spi_int", &MS5611_spi_interface, PX4_SPI_BUS_BARO,0, NULL },
#endif
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MS5611_BUS_I2C_INTERNAL, "/dev/i2c-1", &MS5611_i2c_interface, PX4_I2C_BUS_ONBOARD, 0,NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MS5611_BUS_I2C_EXTERNAL, "/dev/i2c-2", &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION,0, NULL },
#endif
	{MS5611_BUS_SPI_COSTOM,"/dev/ms5611_spi_int1",&MS5611_spi_interface,0,0,NULL}
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct ms5611_bus_option &bus);
bool	find_bus(enum MS5611_BUS busid, ms5611_bus_option &bus);
bool set_bus_spi(uint8_t bus_id,uint8_t chip_select);

int	start(enum MS5611_BUS busid);
int	test(enum MS5611_BUS busid);
int	reset(enum MS5611_BUS busid);
int	info();
int	calibrate(unsigned altitude, enum MS5611_BUS busid);
void	usage();

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}


/**
 * Start the driver.
 */
bool
start_bus(struct ms5611_bus_option &bus)
{
	if (bus.dev != nullptr) {
		warnx("bus option already started");
		return false;
	}
	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum,bus.chipselect);
	if (interface->init() != OK) {
	//	delete interface;   //delete will error．ｗｈｙ？
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}
	bus.dev = new MS5611(interface, prom_buf, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}
	int fd = px4_open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		warnx("can't open baro device");
		return false;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		px4_close(fd);
		warnx("failed setting default poll rate");
		return false;
	}

	px4_close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int
start(enum MS5611_BUS busid)
{
	uint8_t i;
	bool started = false;
	for (i = 0; i < NUM_BUS_OPTIONS; i++) {

		if (busid == MS5611_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MS5611_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}
		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		warnx("driver start failed");
		return 1;
	}

	// one or more drivers started OK
	return 0;
}


/**
 * find a bus structure for a busid
 */
bool find_bus(enum MS5611_BUS busid, ms5611_bus_option &bus)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MS5611_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			bus = bus_options[i];
			return true;
		}
	}

	PX4_WARN("bus %u not started", (unsigned)busid);
	return false;
}


bool set_bus_spi(uint8_t bus_id,uint8_t chip_select)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((MS5611_BUS_SPI_COSTOM == bus_options[i].busid) && bus_options[i].dev == NULL) {
			bus_options[i].busnum = bus_id;
			bus_options[i].chipselect = chip_select;
			return true;
		}
	}

	PX4_WARN("bus MS5611_BUS_SPI_COSTOM not found");
	return false;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test(enum MS5611_BUS busid)
{
	struct ms5611_bus_option bus;

	if (!find_bus(busid, bus)) {
		return 1;
	}

	struct baro_report report;

	ssize_t sz;

	int ret;

	int fd;

	fd = px4_open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		warn("open failed (try 'ms5611 start' if the driver is not running)");
		return 1;
	}

	/* do a simple demand read */
	sz = px4_read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		warn("immediate read failed");
		return 1;
	}

	warnx("single read");
	warnx("pressure:    %10.4f", (double)report.pressure);
	warnx("altitude:    %11.4f", (double)report.altitude);
	warnx("temperature: %8.4f", (double)report.temperature);
	warnx("time:        %lld", (long long)report.timestamp);

	/* set the queue depth to 10 */
	if (OK != px4_ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		warnx("failed to set queue depth");
		return 1;
	}

	/* start the sensor polling at 2Hz */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		warnx("failed to set 2Hz poll rate");
		return 1;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		px4_pollfd_struct_t fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			warnx("timed out waiting for sensor data");
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			return 1;
		}

		warnx("periodic read %u", i);
		warnx("pressure:    %10.4f", (double)report.pressure);
		warnx("altitude:    %11.4f", (double)report.altitude);
		warnx("temperature: %8.4f", (double)report.temperature);
		warnx("time:        %lld", (long long)report.timestamp);
	}

	px4_close(fd);
	warnx("PASS");
	return 0;
}

/**
 * Reset the driver.
 */
int
reset(enum MS5611_BUS busid)
{
	struct ms5611_bus_option bus;

	if (!find_bus(busid, bus)) {
		return 1;
	}

	int fd;

	fd = px4_open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		warn("failed ");
		return 1;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		warn("driver reset failed");
		return 1;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		warn("driver poll restart failed");
		return 1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct ms5611_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	return 0;
}

/**
 * Calculate actual MSL pressure given current altitude
 */
int
calibrate(unsigned altitude, enum MS5611_BUS busid)
{
	struct ms5611_bus_option bus;

	if (!find_bus(busid, bus)) {
		return 1;
	}

	struct baro_report report;

	float	pressure;

	float	p1;

	int fd;

	fd = px4_open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		warn("open failed (try 'ms5611 start' if the driver is not running)");
		return 1;
	}

	/* start the sensor polling at max */
	if (OK != px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		warnx("failed to set poll rate");
		return 1;
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		px4_pollfd_struct_t fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 1000);

		if (ret != 1) {
			warnx("timed out waiting for sensor data");
			return 1;
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("sensor read failed");
			return 1;
		}

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (px4_ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK) {
		warn("BAROIOCSMSLPRESSURE");
		return 1;
	}

	px4_close(fd);
	return 0;
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");
	warnx("    -S    (Simulation bus)");
}

} // namespace

int
ms5611_main(int argc, char *argv[])
{
	enum MS5611_BUS busid = MS5611_BUS_ALL;
	uint8_t bus_type = 0;
	uint8_t chip_select = 0;
	int ch;
	int ret;

	if (argc < 2) {
		ms5611::usage();
		return 1;
	}

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "XISsb:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5611_BUS_I2C_EXTERNAL;
			break;
		case 'I':
			busid = MS5611_BUS_I2C_INTERNAL;
			break;
		case 's':
			busid = MS5611_BUS_SPI_INTERNAL;
			break;
		case 'b':
			busid = MS5611_BUS_SPI_COSTOM;
			bus_type = atoi(myoptarg);
		break;		
		case 'c':
			busid = MS5611_BUS_SPI_COSTOM;
			chip_select = atoi(myoptarg);
		break;
		default:
			ms5611::usage();
			return 1;
		}
	}

	const char *verb = argv[myoptind];
	if ( verb == nullptr ) {
		ms5611::usage();
		return 1;
	}
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {

		if(busid != MS5611_BUS_SPI_COSTOM)
		{
			ret = ms5611::start(busid);
		}
		else
		{
			if(ms5611::set_bus_spi(bus_type,chip_select))
			{
				ret = ms5611::start(busid);
			}
			else
			{
				return 1;
			}
			
			
		}
	}

	/*
	 * Test the driver/device.
	 */
	else if (!strcmp(verb, "test")) {
		ret = ms5611::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	else if (!strcmp(verb, "reset")) {
		ret = ms5611::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = ms5611::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	else if (!strcmp(verb, "calibrate")) {
		if (argc < 2) {
			PX4_WARN("missing altitude");
		}

		long altitude = strtol(argv[myoptind + 1], nullptr, 10);

		ret = ms5611::calibrate(altitude, busid);

	} else {
		ms5611::usage();
		warnx("unrecognised command, try 'start', 'test', 'reset' or 'info'");
		return 1;
	}

	return ret;
}
