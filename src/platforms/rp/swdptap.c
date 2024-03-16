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
/* This file implements the SW-DP interface. */

#include "general.h"
#include "platform.h"
#include "timing.h"
#include "swd.h"
#include "gdb_packet.h"


#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "led.h"
#include "swdp.pio.h"
#include "tusb.h"

// Only want to set / clear one gpio per event so go up in powers of 2
enum _dbg_pins
{
	DBG_PIN_WRITE = 1,
	DBG_PIN_WRITE_WAIT = 2,
	DBG_PIN_READ = 4,
	DBG_PIN_PKT = 8,
};

CU_REGISTER_DEBUG_PINS(probe_timing)

typedef enum swdio_status_e
{
	SWDIO_STATUS_FLOAT = 0,
	SWDIO_STATUS_DRIVE
} swdio_status_t;

#define DIV_ROUND_UP(m, n) (((m) + (n)-1) / (n))



// Uncomment to enable debug
// CU_SELECT_DEBUG_PINS(probe_timing)

struct _probe
{
	// PIO offset
	uint offset;
	uint initted;
};

struct _probe probe = {.initted = 0};

bool swdp_initted(void)
{
	return !!probe.initted;
} 

static void probe_set_swclk_freq(uint freq_khz)
{
	uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
	picoprobe_info("Set swclk freq %dKHz sysclk %dkHz\n", freq_khz, clk_sys_freq_khz);
	// Worked out with saleae
	uint32_t divider = clk_sys_freq_khz / freq_khz / 2;
	pio_sm_set_clkdiv_int_frac(pio0, SWDP_SM, divider, 0);
}

static void probe_write_bits(uint bit_count, uint32_t data_byte)
{
	DEBUG_PINS_SET(probe_timing, DBG_PIN_WRITE);
	pio_sm_put_blocking(pio0, SWDP_SM, bit_count - 1);
	pio_sm_put_blocking(pio0, SWDP_SM, data_byte);
	DEBUG_PINS_SET(probe_timing, DBG_PIN_WRITE_WAIT);
	picoprobe_dump("Write %d bits 0x%x\n", bit_count, data_byte);
	// Wait for pio to push garbage to rx fifo so we know it has finished sending
	pio_sm_get_blocking(pio0, SWDP_SM);
	DEBUG_PINS_CLR(probe_timing, DBG_PIN_WRITE_WAIT);
	DEBUG_PINS_CLR(probe_timing, DBG_PIN_WRITE);
}

static uint32_t probe_read_bits(uint bit_count)
{
	DEBUG_PINS_SET(probe_timing, DBG_PIN_READ);
	pio_sm_put_blocking(pio0, SWDP_SM, bit_count - 1);
	uint32_t data = pio_sm_get_blocking(pio0, SWDP_SM);
	uint32_t data_shifted = data;
	if (bit_count < 32)
	{
		data_shifted = data >> (32 - bit_count);
	}

	picoprobe_dump("Read %d bits 0x%x (shifted 0x%x)\n", bit_count, data, data_shifted);
	DEBUG_PINS_CLR(probe_timing, DBG_PIN_READ);
	return data_shifted;
}

static inline swdio_status_t get_probe_mode(void)
{
	return (pio0->dbg_padoe & (1 << SWDP_PIN_SWDIO)) ? SWDIO_STATUS_DRIVE : SWDIO_STATUS_FLOAT;
}

static void probe_read_mode(void)
{
	pio_sm_exec(pio0, SWDP_SM, pio_encode_jmp(probe.offset + swdp_offset_in_posedge));
	while (pio0->dbg_padoe & (1 << SWDP_PIN_SWDIO))
		;
}

static void probe_write_mode(void)
{
	pio_sm_exec(pio0, SWDP_SM, pio_encode_jmp(probe.offset + swdp_offset_out_negedge));
	while (!(pio0->dbg_padoe & (1 << SWDP_PIN_SWDIO)))
		;
}

static uint32_t swdptap_seq_in(size_t clock_cycles) __attribute__((optimize(3)));
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));

static void swdptap_turnaround(const swdio_status_t dir)
{
	swdio_status_t olddir = get_probe_mode();
	/* Don't turnaround if direction not changing */
	if (dir == olddir)
		return;

#ifdef DEBUG_SWD_BITS
	DEBUG("%s", dir ? "\n-> " : "\n<- ");
#endif

	if (dir == SWDIO_STATUS_FLOAT)
	{
		probe_read_mode();
	}

	probe_read_bits(1);

	if (dir == SWDIO_STATUS_DRIVE)
	{
		probe_write_mode();
	}
}

static void probe_gpio_init()
{
	// Funcsel pins
	pio_gpio_init(pio0, SWDP_PIN_SWCLK);
	gpio_set_drive_strength(SWDP_PIN_SWCLK, GPIO_DRIVE_STRENGTH_8MA);
	gpio_set_slew_rate(SWDP_PIN_SWCLK, GPIO_SLEW_RATE_SLOW);
	pio_gpio_init(pio0, SWDP_PIN_SWDIO);
	gpio_set_drive_strength(SWDP_PIN_SWDIO, GPIO_DRIVE_STRENGTH_8MA);
	gpio_set_slew_rate(SWDP_PIN_SWDIO, GPIO_SLEW_RATE_SLOW);
	// Make sure SWDIO has a pullup on it. Idle state is high
	gpio_pull_up(SWDP_PIN_SWDIO);
	gpio_pull_down(SWDP_PIN_SWCLK);
}

static void probe_init()
{
	if (!probe.initted)
	{
		jtagtap_deinit();
		probe_gpio_init();
		// Target reset pin: pull up, input to emulate open drain pin
		gpio_pull_up(SWDP_PIN_NRST);
		// gpio_init will leave the pin cleared and set as input
		gpio_init(SWDP_PIN_NRST);
		uint offset = pio_add_program(pio0, &swdp_program);
		probe.offset = offset;

		pio_sm_config sm_config = swdp_program_get_default_config(offset);

		// Set SWCLK as a sideset pin
		sm_config_set_sideset_pins(&sm_config, SWDP_PIN_SWCLK);

		// Set SWDIO offset
		sm_config_set_out_pins(&sm_config, SWDP_PIN_SWDIO, 1);
		sm_config_set_set_pins(&sm_config, SWDP_PIN_SWDIO, 1);
		sm_config_set_in_pins(&sm_config, SWDP_PIN_SWDIO);

		// Set SWD and SWDIO pins as output to start. This will be set in the sm
		pio_sm_set_consecutive_pindirs(pio0, SWDP_SM, SWDP_PIN_OFFSET, 2, true);
		pio_sm_set_pins_with_mask(pio0, SWDP_SM, SWDP_PIN_SWDIO, SWDP_PIN_SWDIO);

		// shift output right, autopull off, autopull threshold
		sm_config_set_out_shift(&sm_config, true, false, 0);
		// shift input right as swd data is lsb first, autopush off
		sm_config_set_in_shift(&sm_config, true, false, 0);

		// Init SM with config
		pio_sm_init(pio0, SWDP_SM, offset, &sm_config);

		// Set up divisor
		swdtap_set_clk_freq(plaform_frequency);

		// Enable SM
		pio_sm_set_enabled(pio0, SWDP_SM, 1);
		probe.initted = 1;
	}

	// Jump to write program
	probe_write_mode();
}

void swdp_deinit(void)
{
	if (probe.initted)
	{
		probe_read_mode();
		pio_sm_set_enabled(pio0, SWDP_SM, 0);
		pio_remove_program(pio0, &swdp_program, probe.offset);
		probe.initted = 0;
	}
}

swd_proc_s swd_proc;

static uint32_t swdptap_seq_in(size_t clock_cycles)
{
	swdptap_turnaround(SWDIO_STATUS_FLOAT);
	return probe_read_bits(clock_cycles);
}

static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles)
{
	const uint32_t result = swdptap_seq_in(clock_cycles);
	size_t parity = __builtin_popcount(result);

	parity += probe_read_bits(1) ? 1U : 0U;
	*ret = result;
	/* Terminate the read cycle now */
	swdptap_turnaround(SWDIO_STATUS_DRIVE);
	return parity & 1U;
}

static void swdptap_seq_out(const uint32_t tms_states, const size_t clock_cycles)
{
	swdptap_turnaround(SWDIO_STATUS_DRIVE);
	probe_write_bits(clock_cycles, tms_states);
}

static void swdptap_seq_out_parity(const uint32_t tms_states, const size_t clock_cycles)
{
	int parity = __builtin_popcount(tms_states);
	swdptap_seq_out(tms_states, clock_cycles);
	probe_write_bits(1, parity & 1U);
}

void swdptap_init()
{
	swd_proc.seq_in = swdptap_seq_in;
	swd_proc.seq_in_parity = swdptap_seq_in_parity;
	swd_proc.seq_out = swdptap_seq_out;
	swd_proc.seq_out_parity = swdptap_seq_out_parity;
	probe_init();
}
static uint16_t sm_divider = 1;
void swdtap_set_clk_freq(uint32_t clk_freq_Hz)
{
	/* this is for the swdtap*/
	uint clk_sys_freq_hz = clock_get_hz(clk_sys);
	uint32_t divider = clk_sys_freq_hz / clk_freq_Hz / 4;
	if (divider < 2)
		divider = 2; // max reliable freq
	if (divider >= 0xFFFF)
		divider = 0xFFFF;
	sm_divider = divider;
	pio_sm_set_clkdiv_int_frac(pio0, SWDP_SM, divider, 0);
}
uint32_t swdtap_get_clk_freq(void)
{
	return clock_get_hz(clk_sys) / sm_divider / 4;
}
