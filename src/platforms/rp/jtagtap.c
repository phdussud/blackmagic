/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2023  Black Sphere Technologies Ltd.
 * Written by Patrick Dussud <phdussud@hotmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include "general.h"
#include "platform.h"
#include "jtagtap.h"
#include "gdb_packet.h"
#include "pio_jtag.h"

jtag_proc_s jtag_proc;

pio_jtag_inst_t jtag = {
	.pio = pio0,
	.sm = 0,
	.initted = 0,
};

void jtag_task()
{
	platform_pace_poll();
}

bool jtagtap_initted(void)
{
	return !!jtag.initted;
}

static void jtagtap_reset(void);
static void jtagtap_tms_seq(uint32_t tms_states, size_t ticks);
static void jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static void jtagtap_tdi_seq(bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static bool jtagtap_next(bool tms, bool tdi);
static void jtagtap_cycle(bool tms, bool tdi, size_t clock_cycles);

void jtagtap_init()
{
	if (swdp_initted())
		swdp_deinit();
	platform_target_clk_output_enable(true);
	init_jtag(&jtag, plaform_frequency, JTAG_TCK, JTAG_TDI, JTAG_TDO, JTAG_TMS, JTAG_NRST, JTAG_TRST);
	jtag_proc.jtagtap_reset = jtagtap_reset;
	jtag_proc.jtagtap_next = jtagtap_next;
	jtag_proc.jtagtap_tms_seq = jtagtap_tms_seq;
	jtag_proc.jtagtap_tdi_tdo_seq = jtagtap_tdi_tdo_seq;
	jtag_proc.jtagtap_tdi_seq = jtagtap_tdi_seq;
	jtag_proc.jtagtap_cycle = jtagtap_cycle;
	jtag_proc.tap_idle_cycles = 1;
	/* Ensure we're in JTAG mode */
	for (size_t i = 0; i <= 50U; ++i)
		jtagtap_next(true, false); /* 50 idle cylces for SWD reset */
	jtagtap_tms_seq(0xe73cU, 16U); /* SWD to JTAG sequence */
	jtagtap_soft_reset();
}

void jtagtap_deinit()
{
	jtag_deinit(&jtag);
}

void jtagtap_set_clk_freq(uint32_t clk_freq_Hz)
{
	jtag_set_clk_freq(&jtag, clk_freq_Hz);
}

uint32_t jtagtap_get_clk_freq(void)
{
	return clock_get_hz(clk_sys) / jtag.sm_divider / 4;
}

static void jtagtap_reset(void)
{
#ifdef JTAG_TRST
	if (platform_hwversion() == 0) {
		gpio_put(JTAG_TRST, false);
		platform_delay(1);
		gpio_put(JTAG_TRST, true);
	}
#endif
	jtagtap_soft_reset();
}

static bool jtagtap_next(const bool tms, const bool tdi)
{
	return !!jtag_strobe(&jtag, 1, tms, tdi);
}

static void jtagtap_tms_seq(uint32_t tms_states, const size_t clock_cycles)
{
	bool state = tms_states & 1U;
	for (size_t cycle = 0; cycle < clock_cycles; ++cycle) {
		jtag_strobe(&jtag, 1, state, false);
		tms_states >>= 1U;
		state = tms_states & 1U;
	}
}

static void jtagtap_tdi_tdo_seq(
	uint8_t *const data_out, const bool final_tms, const uint8_t *const data_in, size_t clock_cycles)
{
	if (clock_cycles > 1)
		jtag_transfer(&jtag, clock_cycles - 1, data_in, data_out);
	size_t index = clock_cycles - 1;
	uint8_t res = jtag_strobe(&jtag, 1, final_tms, !!(data_in[index / 8] & (1 << (index & 7))));
	if (res)
		data_out[index / 8] |= 1 << (index & 7);
	else
		data_out[index / 8] &= ~(1 << (index & 7));
}

static void jtagtap_tdi_seq(const bool final_tms, const uint8_t *const data_in, const size_t clock_cycles)
{
	if (clock_cycles == 0)
		return;
	
	if (clock_cycles > 1)
		jtag_transfer(&jtag, clock_cycles - 1, data_in, 0);
	size_t index = clock_cycles - 1;
	jtag_strobe(&jtag, 1, final_tms, !!(data_in[index / 8] & (1 << (index & 7))));
}

static void jtagtap_cycle(const bool tms, const bool tdi, const size_t clock_cycles)
{
	jtag_strobe(&jtag, clock_cycles, tms, tdi);
}
