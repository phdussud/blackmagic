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

/*
 * This file implements capture of the Trace/SWO output using async signalling.boot
 *
 * ARM DDI 0403D - ARMv7M Architecture Reference Manual
 * ARM DDI 0337I - Cortex-M3 Technical Reference Manual
 * ARM DDI 0314H - CoreSight Components Technical Reference Manual
 */

/* TDO/TRACESWO signal comes into the SWOUSART RX pin. */

#include <stdatomic.h>
#include "general.h"
#include "platform.h"
#include "traceswo.h"


#include <stdio.h>
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "tusb.h"

/* For speed this is set to the USB transfer size */
#define FULL_SWO_PACKET (64)

static uint n_checks = 0;
static uint8_t *read_address;
/* Packets arrived from the SWO interface */
volatile static uint8_t trace_rx_buf[NUM_TRACE_PACKETS * FULL_SWO_PACKET];
/* SWO decoding */
static bool decoding = false;
static uint swo_dma_chan; //dma channel
static void dma_handler();
static bool trace_initted = false;
void trace_task()
{
	if (!trace_initted)
		return; // tracing isn't active

	// not recomputing the write address in the loop insures termination
	volatile uint8_t* wa = (uint8_t *)dma_channel_hw_addr(swo_dma_chan)->write_addr;
	if (wa == &trace_rx_buf[NUM_TRACE_PACKETS * FULL_SWO_PACKET])
		wa = &trace_rx_buf[0];
	size_t space = (wa >= read_address) ? (wa - read_address) : (wa + NUM_TRACE_PACKETS * FULL_SWO_PACKET - read_address);
	n_checks++;
	if ((space >= FULL_SWO_PACKET) || ((space != 0) && (n_checks > 4)))
	{
		n_checks = 0;
		uint32_t size_out = MIN(space, FULL_SWO_PACKET);
		if (decoding)
		{
			/* write decoded swo packets to the uart port */
			if (traceswo_decode(NULL, UART_PORT_ITF, read_address, size_out) == 0)
				return;
		}
		else
		{
			/* write raw swo packets to the trace port */
			if (tud_vendor_write_available() >= size_out)
			{
				uint32_t written = tud_vendor_write(read_address, size_out);
				assert(written == size_out);
				tud_task();
			}
			else
				return;
		}
		read_address += size_out;
		if (read_address >= &trace_rx_buf[NUM_TRACE_PACKETS * FULL_SWO_PACKET])
			read_address -= NUM_TRACE_PACKETS * FULL_SWO_PACKET;
	}
}

void traceswo_init(uint32_t baudrate, uint32_t swo_chan_bitmask)
{
	if (!baudrate)
		baudrate = SWO_DEFAULT_BAUD;

	if (trace_initted)
	{
		uart_set_baudrate(SWO_UART, baudrate);
		// clear all errors
		hw_clear_bits(&uart_get_hw(SWO_UART)->rsr, UART_UARTRSR_BITS);
		traceswo_setmask(swo_chan_bitmask);
		decoding = (swo_chan_bitmask != 0);
	}
	else
	{
		// Set up our UART with a basic baud rate.
		uint32_t actual_baud_rate __unused = uart_init(SWO_UART, baudrate);
		gpio_set_function(SWO_UART_RX_PIN, GPIO_FUNC_UART);
		gpio_set_pulls(SWO_UART_RX_PIN, 1, 0);
		uart_set_hw_flow(SWO_UART, false, false);
		uart_set_format(SWO_UART, 8, 1, UART_PARITY_NONE);
		uart_set_fifo_enabled(SWO_UART, true);
		traceswo_setmask(swo_chan_bitmask);
		decoding = (swo_chan_bitmask != 0);

		//clear all errors
		hw_clear_bits(&uart_get_hw(SWO_UART)->rsr, UART_UARTRSR_BITS);
		read_address = (uint8_t*)&trace_rx_buf[0];
		n_checks = 0;
		swo_dma_chan = platform_setup_usart_rx_dma(SWO_UART, read_address, dma_handler, FULL_SWO_PACKET * NUM_TRACE_PACKETS);
		trace_initted = true;
	}
}

static void dma_handler()
{
	if (dma_channel_hw_addr(swo_dma_chan)->transfer_count == 0) //(dma_channel_get_irq0_status(swo_dma_chan))
	{
		dma_channel_set_write_addr(swo_dma_chan, &trace_rx_buf[0], true);
	}
	// Clear the interrupt request.
	dma_hw->ints1 = 1 << swo_dma_chan;
}

