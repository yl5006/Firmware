/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file throwtofly.c
 * Parameters for throw to fly.
 *
 * @author yaoling
 */

/**
 * acc time constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @min 0.04
 * @max 1
 * @decimal 3
 * @group Throwtofly
 */
PARAM_DEFINE_FLOAT(THROW_ACC_TIME, 0.12f);

/**
 * throw power acc  constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @min 0.5
 * @max 2
 * @decimal 2
 * @group Throwtofly
 */
PARAM_DEFINE_FLOAT(THROW_POWER, 1.2f);

/**
 * throw derection
 *
 * 0 for xy derection 1 for z derection
 *
 * @min 0
 * @max 2
 * @decimal 1
 * @group Throwtofly
 */
PARAM_DEFINE_INT32(THROW_DERECTION, 1);
/**
 * throw enable
 *
 * throw enable
 *
 * @min 0.0
 * @decimal 2
 * @group Throwtofly
 */
PARAM_DEFINE_INT32(THROW_ENABLE, 1);
