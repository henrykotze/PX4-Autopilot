/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Owns the ESP32 LEDC hardware: channel/timer allocation bookkeeping and all
 * raw LEDC register access. pwm_servo.c only forwards to the functions here,
 * the same layering used by the STM32/Kinetis/S32K io_pins backends.
 *
 * Only PWM output is implemented - the LEDC peripheral is driven purely as
 * a PWM generator by this backend. PWM input, capture, DShot, LED and
 * OneShot modes are not implemented (OneShot in particular would need a
 * verified LEDC register sequence for immediate one-off triggering that
 * this port does not have).
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include "xtensa.h"

#define DR_REG_DPORT_BASE                       0x3ff00000
#define DPORT_PERIP_CLK_EN_REG          (DR_REG_DPORT_BASE + 0x0C0)
#define DPORT_PERIP_RST_EN_REG          (DR_REG_DPORT_BASE + 0x0C4)

#define DR_REG_LEDC_BASE                0x3ff59000
#define LEDC_LSTIMER0_CONF_REG          (DR_REG_LEDC_BASE + 0x0160)
#define LEDC_LSTIMER1_CONF_REG          (DR_REG_LEDC_BASE + 0x0168)
#define LEDC_LSCH1_CONF0_REG          (DR_REG_LEDC_BASE + 0x00B4)
#define LEDC_LSCH0_CONF0_REG          (DR_REG_LEDC_BASE + 0x00A0)
#define LEDC_LSCH0_CONF1_REG          (DR_REG_LEDC_BASE + 0x00AC)
#define LEDC_LSCH0_HPOINT_REG          (DR_REG_LEDC_BASE + 0x00A4)
#define LEDC_LSCH0_DUTY_REG          (DR_REG_LEDC_BASE + 0x00A8)
#define LEDC_INT_ENA_REG          (DR_REG_LEDC_BASE + 0x0188)
#define LEDC_CONF_REG          (DR_REG_LEDC_BASE + 0x0190)

#define putreg32(v,a)     (*(volatile uint32_t *)(a) = (v))

#define LEDC_SIG_OUT_EN_LSCH0  1 << 2
#define LEDC_PARA_UP_LSCH0  1 << 4
#define DPORT_LEDC_CLK_EN   1 << 11
#define DPORT_LEDC_RST   1 << 11
#define LEDC_LSTIMER0_PAUSE  1 << 23
#define LEDC_LSTIMER0_RST  1 << 24
#define LEDC_TICK_SEL_LSTIMER0  1 << 25
#define LEDC_LSTIMER0_PARA_UP  1 << 26
#define LEDC_DUTY_START_LSCH0  1 << 31

#define LEDC_DIV_NUM_LSTIMER0_S  5
#define LEDC_LSTIMER0_DUTY_RES_S  0

#define b16HALF         0x00008000               /* 0.5 */
#define b16toi(a)       ((a) >> 16)              /* Conversion to integer */

/* LEDC clock resource */
#define LEDC_CLK_RES              (1)         /* APB clock */

/* LEDC timer max reload */
#define LEDC_RELOAD_MAX           (1048576)   /* 2^20 */

/* LEDC timer max clock divider parameter */
#define LEDC_CLKDIV_MAX           (1024)      /* 2^10 */

/* LEDC timer registers mapping */
#define LEDC_TIMER_REG(r, n)      ((r) + (n) * (LEDC_LSTIMER1_CONF_REG - LEDC_LSTIMER0_CONF_REG))

/* LEDC timer channel registers mapping */
#define setbits(bs, a)            modifyreg32(a, 0, bs)
#define resetbits(bs, a)          modifyreg32(a, bs, 0)

#define LEDC_CHAN_REG(r, n)       ((r) + (n) * (LEDC_LSCH1_CONF0_REG - LEDC_LSCH0_CONF0_REG))

#define SET_TIMER_BITS(t, r, b)   setbits(b, LEDC_TIMER_REG(r, t));
#define SET_TIMER_REG(t, r, v)    putreg32(v, LEDC_TIMER_REG(r, t));

#define SET_CHAN_BITS(c, r, b)    setbits(b, LEDC_CHAN_REG(r, c));
#define RESET_CHAN_BITS(c, r, b)  resetbits(b, LEDC_CHAN_REG(r, c));
#define SET_CHAN_REG(c, r, v)     putreg32(v, LEDC_CHAN_REG(r, c));
#define GET_CHAN_REG(c,r)         getreg32(LEDC_CHAN_REG(r, c))

/* Channel mode allocation bitmaps, one bit per logical PX4 channel index.
 * All channels start out NotUsed.
 */

//                                                                NotUsed   PWMOut  PWMIn Capture OneShot Trigger Dshot LED PPS Other
static io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = {
	(io_timer_channel_allocation_t)((1u << MAX_TIMER_IO_CHANNELS) - 1)
};

static io_timer_channel_mode_t timer_allocations[MAX_IO_TIMERS];

/* Shared LEDC timer setup, valid for whichever timer was most recently
 * configured. This backend only ever configures a single active rate at
 * a time, matching every board's current single-LEDC-timer configuration.
 */
static uint32_t reload;
static uint32_t prescaler;
static uint32_t shift;
static uint32_t timer_rate;

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS) ? 0 : -EINVAL;
}

static uint32_t get_timer_channels(unsigned timer)
{
	static uint32_t channels_cache[MAX_IO_TIMERS];
	static bool cache_valid[MAX_IO_TIMERS];

	if (validate_timer_index(timer) != 0) {
		return 0;
	}

	if (!cache_valid[timer]) {
		uint32_t channels = 0;

		for (unsigned chan = 0; chan < MAX_TIMER_IO_CHANNELS; chan++) {
			if (timer_io_channels[chan].gpio_out != 0 && timer_io_channels[chan].timer_index == timer) {
				channels |= 1u << chan;
			}
		}

		channels_cache[timer] = channels;
		cache_valid[timer] = true;
	}

	return channels_cache[timer];
}

int io_timer_validate_channel_index(unsigned channel)
{
	if (channel < MAX_TIMER_IO_CHANNELS &&
	    timer_io_channels[channel].gpio_out != 0 &&
	    validate_timer_index(timer_io_channels[channel].timer_index) == 0) {
		return 0;
	}

	return -EINVAL;
}

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_out;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

int io_timer_allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	irqstate_t flags = px4_enter_critical_section();
	int existing_mode = io_timer_get_channel_mode(channel);
	int ret = -EBUSY;

	if (existing_mode <= IOTimerChanMode_NotUsed || existing_mode == (int)mode) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
		ret = 0;
	}

	px4_leave_critical_section(flags);

	return ret;
}

int io_timer_unallocate_channel(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = io_timer_allocate_channel(channel, mode);
		}
	}

	return rv;
}

int io_timer_allocate_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		if (timer_allocations[timer] == IOTimerChanMode_NotUsed || timer_allocations[timer] == mode) {
			timer_allocations[timer] = mode;
			ret = 0;

		} else {
			ret = -EBUSY;
		}
	}

	return ret;
}

int io_timer_unallocate_timer(unsigned timer)
{
	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		timer_allocations[timer] = IOTimerChanMode_NotUsed;
		ret = 0;
	}

	return ret;
}

/* Compute reload/prescaler/shift for desired_freq and stash them for the
 * next SET_TIMER_REG()/io_timer_set_ccr() calls. Caller must hold the
 * critical section and must not pass desired_freq == 0.
 */
static void get_optimal_timer_setup(uint32_t desired_freq)
{
	uint32_t shifted = 1;
	timer_rate = desired_freq;
	uint64_t pwm_clk = 80000000;
	reload = (pwm_clk * 256 / desired_freq + LEDC_CLKDIV_MAX) / LEDC_CLKDIV_MAX;

	if (reload == 0) {
		reload = 1;

	} else if (reload > LEDC_RELOAD_MAX) {
		reload = LEDC_RELOAD_MAX;
	}

	for (uint32_t c = 2; c <= LEDC_RELOAD_MAX; c *= 2) {
		if (c * 2 > reload) {
			reload = c;
			break;
		}

		shifted++;
	}

	shift = shifted;
	prescaler = (pwm_clk * 256 / reload) / desired_freq;
}

/* Caller must hold the critical section. */
static void timer_set_rate(unsigned timer, unsigned rate)
{
	get_optimal_timer_setup(rate);

	uint32_t regval = (shift << LEDC_LSTIMER0_DUTY_RES_S) | (prescaler << LEDC_DIV_NUM_LSTIMER0_S);
	SET_TIMER_REG(io_timers[timer].base, LEDC_LSTIMER0_CONF_REG, regval);

	/* Setup the timer to use the APB clock (80MHz) */
	SET_TIMER_BITS(io_timers[timer].base, LEDC_LSTIMER0_CONF_REG, LEDC_TICK_SEL_LSTIMER0);

	/* Update clock divide and reload to hardware */
	SET_TIMER_BITS(io_timers[timer].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PARA_UP);
}

int io_timer_init_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	if (validate_timer_index(timer) != 0) {
		return -EINVAL;
	}

	io_timer_channel_mode_t previous_mode = timer_allocations[timer];
	int rv = io_timer_allocate_timer(timer, mode);

	/* Do this only once per timer */
	if (rv == 0 && previous_mode == IOTimerChanMode_NotUsed) {

		irqstate_t flags = px4_enter_critical_section();

		/* Enable and reset the LEDC peripheral, and select its clock source.
		 * Harmless to repeat if more than one timer is ever configured.
		 */
		setbits(DPORT_LEDC_CLK_EN, DPORT_PERIP_CLK_EN_REG);
		resetbits(DPORT_LEDC_RST, DPORT_PERIP_RST_EN_REG);
		putreg32(LEDC_CLK_RES, LEDC_CONF_REG);

		/* pause the timer */
		SET_TIMER_BITS(io_timers[timer].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PAUSE);
		/* reset the timer */
		SET_TIMER_BITS(io_timers[timer].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_RST);

		/* default to updating at 50Hz, like every other io_pins backend */
		timer_set_rate(timer, 50);

		px4_leave_critical_section(flags);
	}

	return rv;
}

int io_timer_set_pwm_rate(unsigned timer, unsigned rate)
{
	if (validate_timer_index(timer) != 0 || timer_allocations[timer] != IOTimerChanMode_PWMOut) {
		return -EINVAL;
	}

	if (rate == 0) {
		/* OneShot triggering is not implemented by this LEDC-based backend */
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();

	timer_set_rate(timer, rate);

	px4_leave_critical_section(flags);

	return OK;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			   channel_handler_t channel_handler, void *context)
{
	if (mode != IOTimerChanMode_PWMOut) {
		/* This LEDC-based backend only implements PWM output channels */
		return -EINVAL;
	}

	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();

	int previous_mode = io_timer_get_channel_mode(channel);
	int rv = allocate_channel(channel, mode);
	unsigned timer = timer_io_channels[channel].timer_index;

	if (rv == 0) {
		/* Try to reserve & initialize the timer - it will only do it once */
		rv = io_timer_init_timer(timer, mode);

		if (rv != 0 && previous_mode == IOTimerChanMode_NotUsed) {
			/* free the channel if it was not used before */
			io_timer_unallocate_channel(channel);
		}
	}

	if (rv == 0) {
		/* Reset this channel's config/duty registers to a known state */
		SET_CHAN_REG(channel, LEDC_LSCH0_CONF0_REG, 0);
		SET_CHAN_REG(channel, LEDC_LSCH0_CONF1_REG, 0);
		SET_CHAN_REG(channel, LEDC_LSCH0_HPOINT_REG, 0);
		SET_CHAN_REG(channel, LEDC_LSCH0_DUTY_REG, 0);
	}

	px4_leave_critical_section(flags);

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	if (mode != IOTimerChanMode_PWMOut) {
		return -EINVAL;
	}

	if (masks == IO_TIMER_ALL_MODES_CHANNELS) {
		masks = channel_allocations[mode];

	} else {
		masks &= channel_allocations[mode];
	}

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (!(masks & (1u << channel))) {
			continue;
		}

		irqstate_t flags = px4_enter_critical_section();

		if (state) {
			/* Reset config 0 & 1 registers */
			SET_CHAN_REG(channel, LEDC_LSCH0_CONF0_REG, 0);
			SET_CHAN_REG(channel, LEDC_LSCH0_CONF1_REG, 0);

			/* Set pulse phase 0 */
			SET_CHAN_REG(channel, LEDC_LSCH0_HPOINT_REG, 0);

			/* Start GPIO output */
			SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);

			/* Start Duty counter */
			SET_CHAN_BITS(channel, LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);

			/* Update duty and phase to hardware */
			SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);

		} else {
			/* Stop just this channel's output; the shared timer keeps
			 * running for any other channel that stays armed.
			 */
			RESET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);
			SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);
		}

		px4_leave_critical_section(flags);
	}

	return OK;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv != 0) {
		return rv;
	}

	if (io_timer_get_channel_mode(channel) != IOTimerChanMode_PWMOut) {
		return -EIO;
	}

	irqstate_t flags = px4_enter_critical_section();

	uint32_t duty = (value * timer_rate) * 0.065536;
	uint64_t scaled = (uint64_t)duty * reload + b16HALF;
	uint32_t regval = b16toi(scaled);

	SET_CHAN_REG(channel, LEDC_LSCH0_CONF0_REG, 0);
	SET_CHAN_REG(channel, LEDC_LSCH0_CONF1_REG, 0);

	/* Set pulse phase 0 */
	SET_CHAN_REG(channel, LEDC_LSCH0_HPOINT_REG, 0);
	SET_CHAN_REG(channel, LEDC_LSCH0_DUTY_REG, regval << 4);

	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);

	/* Start Duty counter */
	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);

	/* Update duty and phase to hardware */
	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);

	px4_leave_critical_section(flags);

	return OK;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0 ||
	    io_timer_get_channel_mode(channel) != IOTimerChanMode_PWMOut ||
	    reload == 0 || timer_rate == 0) {
		return 0;
	}

	uint32_t regval = GET_CHAN_REG(channel, LEDC_LSCH0_DUTY_REG) >> 4;
	uint64_t value_us = ((uint64_t)regval * 1000000ULL) / ((uint64_t)reload * timer_rate);

	return (uint16_t)value_us;
}

void io_timer_trigger(unsigned channels_mask)
{
	/* OneShot mode is not implemented by this backend, so there are never
	 * any channels allocated in IOTimerChanMode_OneShot to trigger.
	 */
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);
}
