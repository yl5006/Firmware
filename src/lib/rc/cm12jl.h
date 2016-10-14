/****************************************************************************
 *
 *	Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file cm12jl.h
 *
 * RC protocol definition for Spektrum RC
 *
 * @author DUYONG
 */

#pragma once

#include <stdint.h>
#include <px4_config.h>
#include <board_config.h>
#include <px4_defines.h>

__BEGIN_DECLS

#define CM12JL_FRAME_SIZE		14		/**< CM12JL frame size in bytes */
#define CM12JL_BUFFER_SIZE		(CM12JL_FRAME_SIZE *4 )
#define CM12JL_HEAD 0xA5
#define CM12JL_HEAD_NEXT 0x5A


#define CM12JL_ID_POS 2
#define CM12JL_INFO_POS 2

__EXPORT int	cm12jl_init(const char *device);
__EXPORT int	cm12jl_config(int cm12jl_fd);
__EXPORT bool	cm12jl_input(int fd, uint16_t *values, uint16_t *num_values);

__EXPORT int 
cm12jl_match(uint8_t *frame);
__EXPORT bool
cm12jl_parse(uint64_t now, uint8_t *frame, unsigned len,uint16_t *values,
	  uint16_t *num_values);
__EXPORT bool
cm12jl_decode(uint8_t *frame, uint16_t *value,uint16_t *value_number);
__EXPORT bool
cm12jl_checksum(uint8_t *frame) ;


enum CM12JL_CMD {							/* CM12JL bind states */
	CM12JL_CMD_BIND_POWER_DOWN = 0,
	CM12JL_CMD_BIND_POWER_UP,
	CM12JL_CMD_BIND_SET_RX_OUT,
	CM12JL_CMD_BIND_SEND_PULSES,
	CM12JL_CMD_BIND_REINIT_UART
};

enum cm12jl_type {
	RadarConfiguration = 0x200,
	RadarStatus = 0x60a,
	ClusterStatus = 0x70b,
	ClusterData = 0xa55a,
	SpeedInformation = 0x300,
	YawRateInformation = 0x301,
	RadarVersion = 0x800
};


__END_DECLS

