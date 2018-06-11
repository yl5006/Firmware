/**
 * @file pwm.h
 *
 * Interface with cameras via pwm.
 *
 */
#pragma once

#ifdef __PX4_NUTTX

#include <drivers/drv_hrt.h>
#include <systemlib/param/param.h>
#include <px4_log.h>
#include <px4_workqueue.h>
#include "camera_interface.h"

class CameraInterfaceSBUS : public CameraInterface
{
public:
	CameraInterfaceSBUS();
	virtual ~CameraInterfaceSBUS();

	void trigger(bool enable);

	void info();
	void set_cammer_rc(struct cammer_rc_s *camrc);
	void set_mount(float pitch,float roll,float yaw);

private:
	int	 _chan;
	int	 _rcs_fd;
	int	_cammer_rc_sub;
	struct work_s	_work;
	struct hrt_call		_trigcall;
	bool _enable;
	void setup();
	uint16_t rcvalues[18];
	static void	cycle_trampoline(void *arg);
	void cicle();
	int fd_open();
	void sbus_output(uint16_t *values, uint16_t num_values);

};

#endif /* ifdef __PX4_NUTTX */
