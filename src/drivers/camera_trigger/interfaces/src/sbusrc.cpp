#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>
#include <px4_config.h>
#include <lib/rc/sbus.h>
#include <board_config.h>
#include "drivers/drv_pwm_trigger.h"
#include <uORB/topics/cammer_rc.h>
#include "sbusrc.h"
#include <fcntl.h>
#include <termios.h>

// TODO : make these parameters later
#define PWM_CAMERA_SHOOT 1800
#define PWM_CAMERA_NEUTRAL 1000
#define	SBUS_FRAME_SIZE 25
CameraInterfaceSBUS::CameraInterfaceSBUS():
	CameraInterface()
{
	_chan = -1;
	_rcs_fd = -1,
	_cammer_rc_sub = -1;
	_enable = false;
	param_t _p_chan;
	// Get parameter handle
	_p_chan = param_find("TRIG_SBUSCHAN");
	param_get(_p_chan, &_chan);
	setup();
}

CameraInterfaceSBUS::~CameraInterfaceSBUS()
{
	work_cancel(LPWORK, &_work);
	close(_rcs_fd);
}

void CameraInterfaceSBUS::setup()
{
#ifdef	GROUNDSTATION_RC_SBUS
//	_cammer_rc_sub = orb_subscribe(ORB_ID(cammer_rc));
	/* open uart */

	fd_open();
#endif

	for(int i=0; i<16 ;i++)
	{
			rcvalues[i]= 1024;
	}
	/*
		 * file descriptor can only be accessed by the process that opened it
		 * so closing here and it will be opened from the High priority kernel
		 * process
		 */
	::close(_rcs_fd);
	_rcs_fd = -1;
	work_queue(HPWORK, &_work, (worker_t)&CameraInterfaceSBUS::cycle_trampoline,
		   this, USEC2TICK(1));
}
int
CameraInterfaceSBUS::fd_open()
{
	_rcs_fd = ::open(GROUNDSTATION_RC_SBUS, O_RDWR | O_NOCTTY |O_NONBLOCK);
	if(_rcs_fd <= 0)
	{
		PX4_INFO("sbus_fd open:%d\r\n",errno);
	}
 	struct termios t;
		/* 100000bps, even parity, two stop bits */
	tcgetattr(_rcs_fd, &t);
	cfsetspeed(&t, 100000);
	t.c_cflag  &= ~CRTSCTS ;
	t.c_cflag |= (CSTOPB | PARENB);
	tcsetattr(_rcs_fd, TCSANOW, &t);
	return _rcs_fd;
}
void
CameraInterfaceSBUS::cycle_trampoline(void *arg)
{
	CameraInterfaceSBUS *dev = reinterpret_cast<CameraInterfaceSBUS *>(arg);

	if (dev->_rcs_fd != -1) {
		dev->cicle();

	} else {
		dev->fd_open();
	}
	work_queue(HPWORK, &(dev->_work), (worker_t)&CameraInterfaceSBUS::cycle_trampoline,
		   dev, USEC2TICK(25 * 1000 ));
}

void CameraInterfaceSBUS::cicle()
{
#ifdef	GROUNDSTATION_RC_SBUS
	   if(_enable)
		{
				rcvalues[_chan-1] = PWM_CAMERA_SHOOT;
			}else
			{
				rcvalues[_chan-1] = PWM_CAMERA_NEUTRAL;
		}
		sbus_output(rcvalues,16);
#endif
}
void CameraInterfaceSBUS::sbus_output(uint16_t *values, uint16_t num_values)
{
	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;

	uint8_t	oframe[SBUS_FRAME_SIZE] = { 0x0f };

	/* 16 is sbus number of servos/channels minus 2 single bit channels.
	 * currently ignoring single bit channels.  */

	for (unsigned i = 0; (i < num_values) && (i < 16); ++i) {
		value =values[i];// (uint16_t)(((values[i] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR) + .5f);

		/*protect from out of bounds values and limit to 11 bits*/
		if (value > 0x07ff) {
			value = 0x07ff;
		}

		while (offset >= 8) {
			++byteindex;
			offset -= 8;
		}

		oframe[byteindex] |= (value << (offset)) & 0xff;
		oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
		oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
		offset += 11;
	}
	int res = ::write(_rcs_fd, oframe, SBUS_FRAME_SIZE);
	if(res <= 0)
	{
		printf("sbus fd %d\r\n",_rcs_fd);
		printf("sbus_fd write %d\r\n",errno);
	}
}

void CameraInterfaceSBUS::trigger(bool enable)
{
	_enable = enable;
}

void CameraInterfaceSBUS::set_cammer_rc(struct cammer_rc_s *camrc)
{
	for(int i=0; i<16 ;i++)
	{
		rcvalues[i]=camrc->values[i];
	}
}
void CameraInterfaceSBUS::set_mount(float pitch,float roll,float yaw)
{
	rcvalues[0]= 1024 + 512 * pitch;
	rcvalues[1]= 1024 + 512 * roll;
	rcvalues[2]= 1024 + 512 * yaw;
}
void CameraInterfaceSBUS::info()
{
#ifdef GROUNDSTATION_RC_SBUS
		PX4_INFO("trigger sbus %s, rc chan 7= : [%d]",GROUNDSTATION_RC_SBUS,rcvalues[_chan-1]);
#endif
}

#endif /* ifdef __PX4_NUTTX */
