/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_command_sender.h
 * Mavlink commands sender with support for retransmission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_tasks.h>
#include <px4_sem.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "timestamped_list.h"
#include "mavlink_bridge_header.h"
#include <v2.0/mavlink_types.h>

/**
 * @class MavlinkCommandSender
 */
class MavlinkCommandSender
{
public:
	/**
	 * initialize: call this once on startup (this function is not thread-safe!)
	 */
	static void initialize();

	static MavlinkCommandSender &instance();

	/**
	 * Send a command on a channel and keep it in a queue for retransmission.
	 * thread-safe
	 * @return 0 on success, <0 otherwise
	 */
	int handle_vehicle_command(const struct vehicle_command_s &command, mavlink_channel_t channel);

	/**
	 * Check timeouts to verify if an commands need retransmission.
	 * thread-safe
	 */
	void check_timeout(mavlink_channel_t channel);

	/**
	 * Handle mavlink command_ack.
	 * thread-safe
	 */
	void handle_mavlink_command_ack(const mavlink_command_ack_t &ack, uint8_t from_sysid, uint8_t from_compid);

private:
	MavlinkCommandSender();

	~MavlinkCommandSender();

	static void lock()
	{
		do {} while (px4_sem_wait(&_lock) != 0);
	}

	static void unlock()
	{
		px4_sem_post(&_lock);
	}

	static MavlinkCommandSender *_instance;
	static px4_sem_t _lock;

	// There are MAVLINK_COMM_0 to MAVLINK_COMM_3, so it should be 4.
	static const unsigned MAX_MAVLINK_CHANNEL = 4;

	typedef struct {
		mavlink_command_long_t command = {};
		hrt_abstime timestamp_us = 0;
		hrt_abstime last_time_sent_us = 0;
		int8_t num_sent_per_channel[MAX_MAVLINK_CHANNEL] = { -1, -1, -1, -1};
	} command_item_t;

	TimestampedList<command_item_t> _commands;

	bool _debug_enabled = false;
	static const uint8_t RETRIES = 3;
	static const uint64_t TIMEOUT_US = 500000;

	/* do not allow copying or assigning this class */
	MavlinkCommandSender(const MavlinkCommandSender &) = delete;
	MavlinkCommandSender operator=(const MavlinkCommandSender &) = delete;
};
