/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file sp25.c
 *
 * Serial protocol decoder for the Futaba S.bus protocol.
 *
 * Created by duyong 20160616
 */

#include <px4_config.h>
#include <board_config.h>
#include <px4_defines.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "sp25.h"
#include <drivers/drv_hrt.h>

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
#define sp25_udelay(arg) usleep(arg)
#else
#include <nuttx/arch.h>
#define sp25_udelay(arg)    up_udelay(arg)
#endif

//#define SP25_DEBUG

static enum SP25_DECODE_STATE {
	SP25_DECODE_STATE_DESYNC = 0,
	SP25_DECODE_STATE_SBUS_START,
	SP25_DECODE_STATE_SYNC
} sp25_decode_state = SP25_DECODE_STATE_DESYNC;



static int sp25_fd = -1;						/**< File handle to the SP25 UART */
static hrt_abstime sp25_last_rx_time;            /**< Timestamp when we last received data */
//static hrt_abstime sp25_last_frame_time;		/**< Timestamp for start of last valid sp25 frame */
static uint8_t sp25_frame[SP25_BUFFER_SIZE];	/**< SP25 sp25 frame receive buffer */
static uint8_t sp25_buf[SP25_FRAME_SIZE * 2];
static unsigned sp25_partial_frame_count;	/**< Count of bytes received for current sp25 frame */
static unsigned sp25_channel_shift = 0;			/**< Channel resolution, 0=unknown, 1=10 bit, 2=11 bit */
static unsigned sp25_frame_drops = 0;			/**< Count of incomplete SP25 frames */
static uint16_t sp25_chan_count = 0;         /**< SP25 channel count */





int
sp25_config(int fd)
{
	int ret = -1;
	if (fd >= 0) {

		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcflush(fd,TCIFLUSH);

		tcsetattr(fd, TCSANOW, &t);

		/* initialise the decoder */
		sp25_partial_frame_count = 0;
		sp25_last_rx_time = hrt_absolute_time();

		ret = 0;
	}
	warnx("555");
	return ret;
}
int 
sp25_match(uint8_t *frame)
{
	return ( *frame == SP25_HEAD && *(frame+1) == SP25_HEAD && *(frame+12) == SP25_TAIL && *(frame+13) == SP25_TAIL);		
}

/**
 * Initialize the SP25 receive functionality
 *
 * Open the UART for receiving SP25 frames and configure it appropriately
 *
 * @param[in] device Device name of SP25 UART
 */
int
sp25_init(const char *device)
{
	warnx("111 sp25_fd %d",sp25_fd);

	if (sp25_fd < 0) {

		sp25_fd = open(device, O_RDONLY | O_NONBLOCK|O_NOCTTY);
		warnx("xxx sp25_fd %d",sp25_fd);
	}

	sp25_channel_shift = 0;
	sp25_frame_drops = 0;
	sp25_chan_count = 0;
	sp25_decode_state = SP25_DECODE_STATE_DESYNC;
	warnx("222");
	int ret = sp25_config(sp25_fd);
	warnx("333");
	if (!ret) {
		return sp25_fd;

	} else {
		return -1;
	}
}
#if 0
static int get_radar_configuration(char *p)
{
	int _radar_id = 0;
	int _radar_output_type = 0;
	int _radar_output_type_valid = 0;
	int _radar_id_valid = 0;

	char *start_pos = (p + SP25_INFO_POS);
	
	
	_radar_id = (*start_pos) & 0x0f;
	_radar_output_type = (*start_pos) & 0x30;
	_radar_output_type_valid = (*(start_pos + 7)) & 0x01;
	_radar_id_valid = (*(start_pos + 7)) & 0x02;

	printf("get_radar_configuration: _radar_id %x _radar_output_type %x _radar_output_type_valid %x _radar_id_valid %x\r\n",
		_radar_id,_radar_output_type,_radar_output_type_valid,_radar_id_valid);

	return 0;	
}

/*
ACTL_Mode 0 6
0:???
1:????
2:????
3:???
4:???
5:????
RadarSt_RollCount 8 2 0~3
Radar_Cfg_Status 12 4
0:????????
1:?????
2: ID ??
3:??????
4:??????
*/
static int get_radar_status(char *p, uint16_t *status)
{
	int _ACTL_mode = 0;
	int _radarst_roll_count = 0;
	int _radar_config_status = 0;
	
	char *start_pos = (p + SP25_INFO_POS);

	_ACTL_mode = (*start_pos) & 0x3f;
	_radarst_roll_count = (*start_pos) & 0x02;
	_radar_config_status = (*start_pos) & 0xf0;

	printf("get_radar_status :_ACTL_mode %x _radarst_roll_count %x _radar_config_status %x \r\n",
		_ACTL_mode,_radarst_roll_count,_radar_config_status);

	*status = (*start_pos) & 0x3f;
	return 0;	
	
}


/*

NoOfCluster 0 8 0~255
ClusterSt_RollCount 8 3 0~3

*/



static int get_cluster_status(char *p)
{
	int _no_of_cluster = 0;
	int _clusterst_roll_count = 0;

	char *start_pos = (p + SP25_INFO_POS);

	_no_of_cluster = (*start_pos);
	_clusterst_roll_count = (*(start_pos + 1)) & 0x03;
	
	printf("get_cluster_status: _no_of_cluster %x _clusterst_roll_count %x \r\n",
		_no_of_cluster,_clusterst_roll_count);

	return 0;	

}


/*
Cluster_Index 0 8 0~127
Cluster_RCSValue 8 8 Val*0.5-50 0~30
Cluster_Range 16 16 Val*0.01 0~655
Cluster_Azimuth 32 7 Val*2-90 -90~90
Cluster_Vrel 48 11 Val*0.05-35 -35~35
Cluster_RollCount 46 2 0~3
Cluster_SNR 56 8 Val-127 -127~128
*/
static int get_cluster_data(uint8_t *p,uint16_t *range)
{
	int _cluster_index = 0;
	float _cluster_rcsvalue = 0;
	float _cluster_range = 0;
	float _cluster_azimuth = 0;
	float _cluster_vrel = 0;
	int _cluster_rollcount = 0;
	int _snr = 0;
	
	uint8_t *start_pos = (p + SP25_INFO_POS);

	_cluster_index = *start_pos;
	_cluster_rcsvalue = (*(start_pos + 1)) * 0.5 - 50;
	_cluster_range = (((*(start_pos + 2) ) << 8) + (*(start_pos + 3) )) * 0.01;
	_cluster_azimuth = ((*(start_pos + 4)) & 0x7f) * 2 - 90;
	_cluster_vrel = (((*(start_pos + 5) & 0x07 ) << 8) + (*(start_pos + 6) )) * 0.05 -35;
	_cluster_rollcount = (*(start_pos + 5)) & 0xc0;
	_snr = (*(start_pos + 7)) - 127;
	printf("##########%x %x\r\n",*(start_pos + 6),*(start_pos + 6));
	printf("get_cluster_data: _cluster_index %d _cluster_rcsvalue %f  _cluster_range %f  _cluster_azimuth %f   _cluster_vrel %f  _cluster_rollcount %d _snr %d  \r\n",
		_cluster_index,(double)_cluster_rcsvalue,(double)_cluster_range,(double)_cluster_azimuth,(double)_cluster_vrel,_cluster_rollcount,_snr);
	*range = (((*(start_pos + 2) ) << 8) + (*(start_pos + 3) ));
	return 0;	

}




/*
radarDeviceSpeed 8 13 0.02m/s 0 ~163.8
radarDeviceSpeedDirection 16 1
0 -> ??
1-> ??
2 -> ??
3 -> ??
*/
static int get_speed_information(char *p)
{
	float _radar_device_speed = 0;
	int _radar_device_speed_direction = 0;

	char *start_pos = (p + SP25_INFO_POS);
	_radar_device_speed = (((*(start_pos) & 0x1f ) << 8) + (*(start_pos + 1) )) * 0.02;
	_radar_device_speed_direction = (*(start_pos + 2) ) & 0x03;

	printf("_radar_device_speed %f _radar_device_speed_direction %d\r\n",(double)_radar_device_speed,_radar_device_speed_direction);

	return 0;	
	

}

#endif


static int get_cluster_data(uint8_t *p,uint16_t *range)
{
	uint8_t *start_pos = (p + SP25_INFO_POS);
	*range = (((*(start_pos + 2) ) << 8) + (*(start_pos + 3) ));
	return 0;	

}

/**
 * Decode the entire sp25 frame (all contained channels)
 *
 * @param[in] frame_time timestamp when this sp25 frame was received. Used to detect RX loss in order to reset 10/11 bit guess.
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned
 * @return true=SP25 frame successfully decoded, false=no update
 */
bool
sp25_decode(uint8_t *frame, uint16_t *value,uint16_t *value_number)
{
/* check frame boundary markers to avoid out-of-sync cases */
	if (!sp25_match(frame)) {
		sp25_frame_drops++;
#ifdef SP25_DEBUG
		printf("DECODE FAIL: ");

		for (unsigned i = 0; i < SP25_FRAME_SIZE; i++) {
			printf("%0x ", frame[i]);
		}

		printf("\n");
#endif
		sp25_decode_state = SP25_DECODE_STATE_DESYNC;
		return false;
	}

	uint16_t type = (*(frame+3) << 8) + *(frame+2);
//	printf("p+2 %x p+3 %x t1 %x \r\n",*(frame+2),*(frame+3),type);
	switch (type)
	{
		case RadarConfiguration:
			//get_radar_configuration(frame);
			break;
		case RadarStatus:
			//get_radar_status(frame,value);
			//value_number = 1;
			//type = RadarStatus;
			break;
		case ClusterStatus:
			//get_cluster_status(frame);
			break;
		case ClusterData:
			value[0] = (uint16_t)ClusterData;
			get_cluster_data(frame,&value[1]);
			*value_number = 2;
			//type = ClusterData;
			break;
		case SpeedInformation:
			//get_speed_information(frame);
			break;
		case YawRateInformation:
			break;
		case RadarVersion:
			break;
		default:
			break;
	}
	

	return true;
}

/**
 * Called periodically to check for input data from the SP25 UART
 *
 * The SP25* protocol doesn't provide any explicit framing,
 * so we detect sp25 frame boundaries by the inter-sp25 frame delay.
 * The minimum sp25 frame spacing is 11ms; with 16 bytes at 115200bps
 * sp25 frame transmission time is ~1.4ms.
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a sp25 frame.
 * In the case where byte(s) are dropped from a sp25 frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 * Upon receiving a full sp25 frame we attempt to decode it.
 *
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned, high order bit 0:10 bit data, 1:11 bit data
 * @param[out] n_butes number of bytes read
 * @param[out] bytes pointer to the buffer of read bytes
 * @return true=decoded raw channel values updated, false=no update
 */
bool
sp25_input(int fd, uint16_t *values, uint16_t *num_values)
{
	int		ret = 1;
	hrt_abstime	now;

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */

	ret = read(fd, &sp25_buf[0], sizeof(sp25_buf) / sizeof(sp25_buf[0]));

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;

	} else {
		//*n_bytes = ret;
		//*bytes = &sp25_buf[0];
	}

	/*
	 * Try to decode something with what we got
	 */

	return sp25_parse(now, &sp25_buf[0], ret, values, num_values);
}

bool
sp25_parse(uint64_t now, uint8_t *frame, unsigned len,uint16_t *values,
	  uint16_t *num_values)
{

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (sp25_partial_frame_count == sizeof(sp25_frame) / sizeof(sp25_frame[0])) {
			sp25_partial_frame_count = 0;
			sp25_decode_state = SP25_DECODE_STATE_DESYNC;
//#ifdef SP25_DEBUG
			printf("SP25: RESET (BUF LIM)\n");
//#endif
		}

		if (sp25_partial_frame_count == SP25_FRAME_SIZE) {
			sp25_partial_frame_count = 0;
			sp25_decode_state = SP25_DECODE_STATE_DESYNC;
//#ifdef SP25_DEBUG
			printf("SP25: RESET (PACKET LIM)\n");
//#endif
		}


#if 0
		printf("sp25 state: %s%s, count: %d, val: %02x\n",
		       (sp25_decode_state == SP25_DECODE_STATE_DESYNC) ? "SP25_DECODE_STATE_DESYNC" : "",
		       (sp25_decode_state == SP25_DECODE_STATE_SYNC) ? "SP25_DECODE_STATE_SYNC" : "",
		       sp25_partial_frame_count,
		       (unsigned)frame[d]);
#endif


		switch (sp25_decode_state) {
		case SP25_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */

		/* we are de-synced and only interested in the frame marker */
			if (sp25_match(frame+d)) {
				sp25_decode_state = SP25_DECODE_STATE_SBUS_START;
				sp25_partial_frame_count = 0;
				sp25_frame[sp25_partial_frame_count++] = frame[d];
			}
			break;

		case SP25_DECODE_STATE_SBUS_START:
		case SP25_DECODE_STATE_SYNC: {
				sp25_frame[sp25_partial_frame_count++] = frame[d];
//				printf("SP25_FRAME_SIZE %d we come here sp25_frame[%d] %#x",
//						SP25_FRAME_SIZE,sp25_partial_frame_count,frame[d]);
				/* decode whatever we got and expect */
				if (sp25_partial_frame_count < SP25_FRAME_SIZE) {
					break;
				}

				/*
				 * Great, it looks like we might have a frame.  Go ahead and
				 * decode it.
				 */
				decode_ret = sp25_decode(sp25_frame, values, &sp25_chan_count);

				/* we consumed the partial frame, reset */
				sp25_partial_frame_count = 0;

				/* if decoding failed, set proto to desync */
				if (decode_ret == false) {
					sp25_decode_state = SP25_DECODE_STATE_DESYNC;
					sp25_frame_drops++;
				}
			}
			break;

		default:
#ifdef SP25_DEBUG
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}


	if (decode_ret) {
		*num_values = sp25_chan_count;
	}

	sp25_last_rx_time = now;

	/* return false as default */
	return decode_ret;
}


