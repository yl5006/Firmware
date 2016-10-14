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
 * @file cm12jl.c
 *
 * Serial protocol decoder for the cm12jl.
 *
 * Created by duyong 20160616
 */

#include <px4_config.h>
#include <board_config.h>
#include <px4_defines.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "cm12jl.h"
#include <drivers/drv_hrt.h>


#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
#define cm12jl_udelay(arg) usleep(arg)
#else
#include <nuttx/arch.h>
#define cm12jl_udelay(arg)    up_udelay(arg)
#endif

//#define CM12JL_DEBUG

static enum CM12JL_DECODE_STATE {
	CM12JL_DECODE_STATE_DESYNC = 0,
	CM12JL_DECODE_STATE_SBUS_START,
	CM12JL_DECODE_STATE_SYNC
} cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;



static int cm12jl_fd = -1;						/**< File handle to the CM12JL UART */
static hrt_abstime cm12jl_last_rx_time;            /**< Timestamp when we last received data */
//static hrt_abstime cm12jl_last_frame_time;		/**< Timestamp for start of last valid cm12jl frame */
static uint8_t cm12jl_frame[CM12JL_BUFFER_SIZE];	/**< CM12JL cm12jl frame receive buffer */
static uint8_t cm12jl_buf[CM12JL_FRAME_SIZE * 2];
static unsigned cm12jl_partial_frame_count;	/**< Count of bytes received for current cm12jl frame */
static unsigned cm12jl_channel_shift = 0;			/**< Channel resolution, 0=unknown, 1=10 bit, 2=11 bit */
static unsigned cm12jl_frame_drops = 0;			/**< Count of incomplete CM12JL frames */
static uint16_t cm12jl_chan_count = 0;         /**< CM12JL channel count */

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
int
cm12jl_config(int fd) {
	int ret = -1;
	if (fd >= 0) {

	struct termios pNewtio;
	memset(&pNewtio,0, sizeof(struct termios));
//8N1
	pNewtio.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	pNewtio.c_iflag = IGNPAR;
	pNewtio.c_oflag = 0;
	pNewtio.c_lflag = 0; //non ICANON


	pNewtio.c_cc[VINTR] = 0;
	pNewtio.c_cc[VQUIT] = 0;
	pNewtio.c_cc[VERASE] = 0;
	pNewtio.c_cc[VKILL] = 0;
	pNewtio.c_cc[VEOF] = 4;
	pNewtio.c_cc[VTIME] = 5;
	pNewtio.c_cc[VMIN] = 0;
	pNewtio.c_cc[VSWTC] = 0;
	pNewtio.c_cc[VSTART] = 0;
	pNewtio.c_cc[VSTOP] = 0;
	pNewtio.c_cc[VSUSP] = 0;
	pNewtio.c_cc[VEOL] = 0;
	pNewtio.c_cc[VREPRINT] = 0;
	pNewtio.c_cc[VDISCARD] = 0;
	pNewtio.c_cc[VWERASE] = 0;
	pNewtio.c_cc[VLNEXT] = 0;
	pNewtio.c_cc[VEOL2] = 0;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &pNewtio);


	/* initialise the decoder */
	cm12jl_partial_frame_count = 0;
	cm12jl_last_rx_time = hrt_absolute_time();

	ret = 0;
	}
	return ret;
}
#else
int
cm12jl_config(int fd){
	int ret = -1;
	if (fd >= 0) {

		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(fd, TCSANOW, &t);

		/* initialise the decoder */
		cm12jl_partial_frame_count = 0;
		cm12jl_last_rx_time = hrt_absolute_time();

		ret = 0;
	}
	return ret;
}

#endif
bool
cm12jl_checksum(uint8_t *frame) {

	uint16_t checksum = 0;
	int bits = 0;
	for(; bits < (CM12JL_FRAME_SIZE - 2); bits++) {
		checksum += frame[bits];
	}
#ifdef CM12JL_DEBUG
	int test = 0;
	for(test = 0 ; test < CM12JL_FRAME_SIZE ; test++) {
		warnx("frame[%d]: %#x",test,frame[test]);
	}
#endif
	if(((checksum & 0xff) == frame[CM12JL_FRAME_SIZE-1]) && (((checksum >> 8) & 0xff) == frame[CM12JL_FRAME_SIZE-2])) {
		return true;
	}
	warnx("\r\ncm12jl checksum error\r\n");
	return false;
}


int
cm12jl_match(uint8_t *frame) {
	if(*frame == CM12JL_HEAD && *(frame+1) == CM12JL_HEAD_NEXT) {
		return 1;
	} else {
		return 0;
	}
	return 0;
}

/**
 * Initialize the CM12JL receive functionality
 *
 * Open the UART for receiving CM12JL frames and configure it appropriately
 *
 * @param[in] device Device name of CM12JL UART
 */
int
cm12jl_init(const char *device) {

	if (cm12jl_fd < 0) {

		cm12jl_fd = open(device, O_RDONLY | O_NONBLOCK | O_NOCTTY);
		warnx("cm12jl_fd %d",cm12jl_fd);
	}

	cm12jl_channel_shift = 0;
	cm12jl_frame_drops = 0;
	cm12jl_chan_count = 0;
	cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;
	int ret = cm12jl_config(cm12jl_fd);
	if (!ret) {
		return cm12jl_fd;

	} else {
		return -1;
	}
}


static int get_cluster_data(uint8_t *p,uint16_t *range) {
	uint8_t *start_pos = (p + CM12JL_INFO_POS);
	range[0] = (start_pos[0] << 8) + start_pos[1];
	range[1] = (start_pos[2] << 8) + start_pos[3];
	range[2] = (start_pos[4] << 8) + start_pos[5];
	range[3] = (start_pos[6] << 8) + start_pos[7];
	range[4] = (start_pos[8] << 8) + start_pos[9];
//	warnx("range is %d\r\n",*range);
//	printf("range is %d %d %d %d %d\r\n",range[0],range[1],range[2],range[3],range[4]);

	return 0;

}

/**
 * Decode the entire cm12jl frame (all contained channels)
 *
 * @param[in] frame_time timestamp when this cm12jl frame was received. Used to detect RX loss in order to reset 10/11 bit guess.
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned
 * @return true=CM12JL frame successfully decoded, false=no update
 */
bool
cm12jl_decode(uint8_t *frame, uint16_t *value,uint16_t *value_number) {
	/* check frame boundary markers to avoid out-of-sync cases */
	if (!cm12jl_match(frame)) {
		cm12jl_frame_drops++;
#ifdef CM12JL_DEBUG
		printf("DECODE FAIL: ");

		for (unsigned i = 0; i < CM12JL_FRAME_SIZE; i++) {
			printf("%0x ", frame[i]);
		}

		printf("\n");
#endif
		cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;
		return false;
	}

	value[0] = (uint16_t)ClusterData;
	get_cluster_data(frame,&value[1]);
	*value_number = 5;

	return true;
}

/**
 * Called periodically to check for input data from the CM12JL UART
 *
 * The CM12JL* protocol doesn't provide any explicit framing,
 * so we detect cm12jl frame boundaries by the inter-cm12jl frame delay.
 * The minimum cm12jl frame spacing is 11ms; with 16 bytes at 115200bps
 * cm12jl frame transmission time is ~1.4ms.
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a cm12jl frame.
 * In the case where byte(s) are dropped from a cm12jl frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 * Upon receiving a full cm12jl frame we attempt to decode it.
 *
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned, high order bit 0:10 bit data, 1:11 bit data
 * @param[out] n_butes number of bytes read
 * @param[out] bytes pointer to the buffer of read bytes
 * @return true=decoded raw channel values updated, false=no update
 */
bool
cm12jl_input(int fd, uint16_t *values, uint16_t *num_values) {
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


	ret = read(fd, &cm12jl_buf[0], sizeof(cm12jl_buf) / sizeof(cm12jl_buf[0]));

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;

	} else {
		//*n_bytes = ret;
		//*bytes = &cm12jl_buf[0];
	}

	/*
	 * Try to decode something with what we got
	 */
#if 0
	static uint32_t cl = 0;

	int ct;
	for(ct = 0 ; ct < ret ; ct++) {
		cl++;
		printf("%.2x ",cm12jl_buf[ct]);
		if(cl == 14) {
			cl++;
			printf("\r\n");
			cl = 0;
		}
	}
#endif
	return cm12jl_parse(now, &cm12jl_buf[0], ret, values, num_values);
}

bool
cm12jl_parse(uint64_t now, uint8_t *frame, unsigned len,uint16_t *values,
             uint16_t *num_values) {

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (cm12jl_partial_frame_count == sizeof(cm12jl_frame) / sizeof(cm12jl_frame[0])) {
			cm12jl_partial_frame_count = 0;
			cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;
//#ifdef CM12JL_DEBUG
			printf("CM12JL: RESET (BUF LIM)\n");
//#endif
		}

		if (cm12jl_partial_frame_count == CM12JL_FRAME_SIZE) {
			cm12jl_partial_frame_count = 0;
			cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;
//#ifdef CM12JL_DEBUG
			printf("CM12JL: RESET (PACKET LIM)\n");
//#endif
		}

		switch (cm12jl_decode_state) {
		case CM12JL_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */

			/* we are de-synced and only interested in the frame marker */
			if (cm12jl_match(frame+d)) {
				cm12jl_decode_state = CM12JL_DECODE_STATE_SBUS_START;
				cm12jl_partial_frame_count = 0;
				cm12jl_frame[cm12jl_partial_frame_count++] = frame[d];
			}
			break;

		case CM12JL_DECODE_STATE_SBUS_START:
		case CM12JL_DECODE_STATE_SYNC: {
			cm12jl_frame[cm12jl_partial_frame_count++] = frame[d];

			/* decode whatever we got and expect */
			if (cm12jl_partial_frame_count < CM12JL_FRAME_SIZE) {
				break;
			}

			/*
			 * Great, it looks like we might have a frame.  Go ahead and
			 * decode it.
			 */
			if(cm12jl_checksum(cm12jl_frame)) {
				decode_ret = cm12jl_decode(cm12jl_frame, values, &cm12jl_chan_count);
			} else {
				decode_ret = false;
			}
			/* we consumed the partial frame, reset */
			cm12jl_partial_frame_count = 0;

			/* if decoding failed, set proto to desync */
			if (decode_ret == false) {
				cm12jl_decode_state = CM12JL_DECODE_STATE_DESYNC;
				cm12jl_frame_drops++;
			}
		}
		break;

		default:
#ifdef CM12JL_DEBUG
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}


	if (decode_ret) {
		*num_values = cm12jl_chan_count;
	}

	cm12jl_last_rx_time = now;

	/* return false as default */
	return decode_ret;
}


