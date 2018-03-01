#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>
#include <px4_config.h>
#include <lib/rc/sbus.h>
#include <board_config.h>
#include "drivers/drv_pwm_trigger.h"
#include <uORB/topics/cammer_rc.h>
#include "sbusrc.h"

// TODO : make these parameters later
#define PWM_CAMERA_SHOOT 1800
#define PWM_CAMERA_NEUTRAL 1000

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
}

void CameraInterfaceSBUS::setup()
{
#ifdef	GROUNDSTATION_RC_SBUS
//	_cammer_rc_sub = orb_subscribe(ORB_ID(cammer_rc));
	/* open uart */

	_rcs_fd	=sbus_initrc(GROUNDSTATION_RC_SBUS,false);
#endif

	for(int i=0; i<16 ;i++)
	{
			rcvalues[i]= 1500;
	}

	work_queue(LPWORK, &_work, (worker_t)&CameraInterfaceSBUS::cycle_trampoline,
		   this, USEC2TICK(1));
	//25Hz
	// Precompute the bitmask for enabled channels
//	hrt_call_every(&_trigcall, 0, (25 * 1000),
//		       (hrt_callout)&CameraInterfaceSBUS::cycle_trampoline, this);
}
void
CameraInterfaceSBUS::cycle_trampoline(void *arg)
{
	CameraInterfaceSBUS *dev = reinterpret_cast<CameraInterfaceSBUS *>(arg);

	dev->cicle();
}

void CameraInterfaceSBUS::cicle()
{
#ifdef GROUNDSTATION_RC_SBUS
	   if(_enable)
		{
				rcvalues[_chan-1] = PWM_CAMERA_SHOOT;
			}else
			{
				rcvalues[_chan-1] = PWM_CAMERA_NEUTRAL;
		}
	   _rcs_fd	= sbus_initrc(GROUNDSTATION_RC_SBUS,false);
		sbus1_output(_rcs_fd,rcvalues,16);
		close(_rcs_fd);
		work_queue(LPWORK, &_work, (worker_t)&CameraInterfaceSBUS::cycle_trampoline,
				   this, USEC2TICK(25 * 1000 ));
#endif
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

void CameraInterfaceSBUS::info()
{
#ifdef GROUNDSTATION_RC_SBUS
		PX4_INFO("trigger sbus %s, rc chan 7= : [%d]",GROUNDSTATION_RC_SBUS,rcvalues[_chan-1]);
#endif
}

#endif /* ifdef __PX4_NUTTX */
