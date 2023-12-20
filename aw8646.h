/* SPDX-License-Identifier: BSD/GPL-2.0*/
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright (c) 2022 Shanghai Awinic Technology Co., Ltd. All Rights Reserved
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _AW8646_H_
#define _AW8646_H_

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#define AW8646_MAX_FREQUENCY			250000
#define AW8646_DEFAULT_FREQUENCY		1000

struct aw8646 {
	struct mutex lock;
	struct device *dev;
	struct hrtimer hrtimer;
	struct platform_device *pdev;

	ktime_t kinterval;
	uint32_t step_num;
	uint32_t timer_cnt;
	uint32_t half_period;
	uint32_t step_frequency;

	int nen_pin;		/* driver enable control input pin */
	int dir_pin;		/* direction input pin */
	int step_pin;		/* step input pin */
	int nsleep_pin;		/* sleep mode input pin */
	int dir_pin_state;
};

#define DRIVER_NAME				"aw8646_step"

#define AW_LOGI(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define AW_LOGE(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)

enum aw_gpio_state {
	AW_GPIO_LOW = 0,
	AW_GPIO_HIGH = 1,
};

#endif
