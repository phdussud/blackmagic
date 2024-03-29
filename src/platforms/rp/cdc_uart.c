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

#include <pico/stdlib.h>
#include <hardware/dma.h>
#include <hardware/irq.h>

#include "tusb.h"

#include "platform.h"
/* For speed this is set to the USB transfer size */
#define FULL_SWO_PACKET (64)

#define TX_BUFFER_SIZE (4096) //needs to be a power of 2
#define RX_BUFFER_SIZE (4096)
static uint8_t tx_buf[TX_BUFFER_SIZE] __aligned(TX_BUFFER_SIZE);
volatile static uint8_t rx_buf[RX_BUFFER_SIZE];

static uint rx_dma_channel;
static uint tx_dma_channel;
static volatile uint8_t *tx_write_address;
static uint8_t *rx_read_address;
static uint n_checks;

static void dma_handler();

static uint n_bits(uint n)
{
	int i;
	for (i = 0; i < 31; i++)
	{
		n >>= 1;
		if (n == 0)
		break;
	}
	return i+1;
}

uint setup_usart_tx_dma(uart_inst_t *uart, void *tx_address, uint buffer_size)
{
	uint dma_chan = dma_claim_unused_channel(true);
	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	dma_channel_set_irq1_enabled(dma_chan, true);
	// enable DMA TX
	hw_write_masked(&uart_get_hw(uart)->dmacr, 1 << UART_UARTDMACR_TXDMAE_LSB, UART_UARTDMACR_TXDMAE_BITS);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, uart_get_dreq(uart, true));
	channel_config_set_ring(&c, false, n_bits(buffer_size - 1));
	dma_channel_configure(
		dma_chan,
		&c,
		&uart_get_hw(uart)->dr, // Write Address
		tx_address,             // Read Address
		0,                      // transfer count
		false                   // start
	);
	return dma_chan;
}

uint setup_usart_rx_dma(uart_inst_t *uart, volatile void *rx_address, irq_handler_t handler, uint buffer_size)
{
	uint dma_chan = dma_claim_unused_channel(true);
	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	dma_channel_set_irq1_enabled(dma_chan, true);

	// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
	irq_add_shared_handler(DMA_IRQ_1, handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	irq_set_enabled(DMA_IRQ_1, true);
	// enable DMA RX
	hw_write_masked(&uart_get_hw(uart)->dmacr, 1 << UART_UARTDMACR_RXDMAE_LSB, UART_UARTDMACR_RXDMAE_BITS);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, uart_get_dreq(uart, false));
	hw_clear_bits(&uart_get_hw(uart)->rsr, UART_UARTRSR_BITS); // clear
	dma_channel_configure(
		dma_chan,
		&c,
		rx_address,				// Write Address
		&uart_get_hw(uart)->dr, // Read Address
		buffer_size,			// transfer count
		true					// start
	);
	return dma_chan;
}

void cdc_uart_init(void)
{
	gpio_set_function(USBUSART_TX, GPIO_FUNC_UART);
	gpio_set_function(USBUSART_RX, GPIO_FUNC_UART);
	gpio_set_pulls(USBUSART_TX, 1, 0);
	gpio_set_pulls(USBUSART_RX, 1, 0);
	uart_init(USBUSART_INTERFACE, USBUSART_BAUDRATE);
	uart_set_hw_flow(USBUSART_INTERFACE, false, false);
	uart_set_format(USBUSART_INTERFACE, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(USBUSART_INTERFACE, true);
	tx_dma_channel = setup_usart_tx_dma(USBUSART_INTERFACE, &tx_buf[0], TX_BUFFER_SIZE);
	rx_dma_channel = setup_usart_rx_dma(USBUSART_INTERFACE, &rx_buf[0], dma_handler, RX_BUFFER_SIZE);
	tx_write_address = &tx_buf[0];
	rx_read_address = (uint8_t*)&rx_buf[0];
	n_checks = 0; 
}

// shared between rx_dma_channel and tx_dma_channel

static void dma_handler()
{
	volatile uint32_t ints = dma_hw->ints1;
	if (ints & (1 << rx_dma_channel)) //dma_channel_hw_addr(rx_dma_channel)->transfer_count == 0) // dma_channel_get_irq1_status(rx_dma_channel))
	{
		dma_channel_set_write_addr(rx_dma_channel, &rx_buf[0], true);
	}
	if (ints & (1 << tx_dma_channel)) //(dma_channel_hw_addr(tx_dma_channel)->transfer_count == 0) // (dma_channel_get_irq1_status(tx_dma_channel))
	{
		uint8_t *ra = (uint8_t *)(dma_channel_hw_addr(tx_dma_channel)->read_addr);
		uint8_t *l_tx_write_address = (uint8_t *)tx_write_address;
		size_t space =	(l_tx_write_address >= ra) ? (l_tx_write_address - ra) : (l_tx_write_address + TX_BUFFER_SIZE - ra);
		if (space > 0)
			dma_channel_set_trans_count(tx_dma_channel, space, true);
	}
	dma_hw->ints1 = ints;
}

bool cdc_stopped = false;
void cdc_task(void)
{
	if (cdc_stopped)
	  	return;
	static int was_connected = 0;
	if (tud_cdc_n_connected(UART_PORT_ITF))
	{
		was_connected = 1;
		int written = 0;
		volatile uint8_t *wa = (uint8_t*)(dma_channel_hw_addr(rx_dma_channel)->write_addr);
		if (wa == &rx_buf[RX_BUFFER_SIZE])
		{
			wa = &rx_buf[0];
		}
		uint32_t space = (wa >= rx_read_address) ? (wa - rx_read_address) : (wa + RX_BUFFER_SIZE - rx_read_address);
		n_checks++;
		if ((space >= FULL_SWO_PACKET) || ((space != 0) && (n_checks > 4)))
		{
			n_checks = 0;
			uint32_t capacity = tud_cdc_n_write_available(UART_PORT_ITF);
			uint32_t size_out = MIN(space, capacity);
			if (capacity >= FULL_SWO_PACKET)
			{
				uint32_t written = tud_cdc_n_write(UART_PORT_ITF, rx_read_address, size_out);
				if (space < FULL_SWO_PACKET)
					tud_cdc_n_write_flush(UART_PORT_ITF);
				tud_task();
				rx_read_address += written;
				if (rx_read_address >= &rx_buf[RX_BUFFER_SIZE])
					rx_read_address -= RX_BUFFER_SIZE;
			}
		}
		uint ra = tud_cdc_n_available(UART_PORT_ITF);
		size_t watermark = MIN(ra, &tx_buf[TX_BUFFER_SIZE] - tx_write_address);
		if (watermark > 0)
		{
			size_t tx_len;
			tx_len = tud_cdc_n_read(UART_PORT_ITF, (void*)tx_write_address, watermark);
			//be careful about modifying tx_write_address as it is used in the IRQ handler
			uint8_t *l_tx_write_address = (uint8_t *)(tx_write_address + tx_len);
			if (l_tx_write_address >= &tx_buf[TX_BUFFER_SIZE])
				tx_write_address = l_tx_write_address - TX_BUFFER_SIZE;
			else
				tx_write_address = l_tx_write_address;
			// restart dma if not active
			if (!dma_channel_is_busy(tx_dma_channel))
			{
				uint8_t *ra = (uint8_t *)(dma_channel_hw_addr(tx_dma_channel)->read_addr);
				size_t space = (tx_write_address >= ra) ? (tx_write_address - ra) : (tx_write_address + TX_BUFFER_SIZE - ra);
				if (space > 0)
					dma_channel_set_trans_count(tx_dma_channel, space, true);
			}
		}
	}
	else if (was_connected)
	{
		tud_cdc_n_write_clear(UART_PORT_ITF);
		was_connected = 0;
	}
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* line_coding)
{
	if (itf != UART_PORT_ITF)
	  return;
	uart_deinit(USBUSART_INTERFACE);
	tud_cdc_n_write_clear(itf);
	tud_cdc_n_read_flush(itf);
	uart_init(USBUSART_INTERFACE, line_coding->bit_rate);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	if (itf != UART_PORT_ITF)
		return;
	/* CDC drivers use linestate as a bodge to activate/deactivate the interface.
	* Resume our UART polling on activate, stop on deactivate */
	if (!dtr && !rts)
		cdc_stopped = true;
	else
		cdc_stopped = false;
}

void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{
	for (size_t offset = 0; offset < len;)
	{
		size_t capacity = tud_cdc_n_write_available(UART_PORT_ITF);
		const size_t count = MIN(len - offset, capacity);
		size_t written = tud_cdc_n_write(UART_PORT_ITF, data + offset, count);
		offset += written;
		if (count < FULL_SWO_PACKET)
			tud_cdc_n_write_flush(UART_PORT_ITF);
		tud_task();

	}
}
