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

/* This file provides the platform specific declarations for the mini-401 implementation. */

#ifndef PLATFORMS_RP_PLATFORM_H
#define PLATFORMS_RP_PLATFORM_H

// #include "gpio.h"
#include "timing.h"
// #include "timing_stm32.h"
#include <pico/stdlib.h>
#undef SRAM_BASE
#include <stdint.h>
#include "hardware/gpio.h"

#include <setjmp.h>

#if false
#define picoprobe_info(format, args...) printf(format, ##args)
#else
#define picoprobe_info(format, ...) ((void)0)
#endif

#if false
#define picoprobe_debug(format, args...) printf(format, ##args)
#else
#define picoprobe_debug(format, ...) ((void)0)
#endif

#if false
#define picoprobe_dump(format, args...) printf(format, ##args)
#else
#define picoprobe_dump(format, ...) ((void)0)
#endif

//#define PLATFORM_HAS_TRACESWO
#define PLATFORM_IDENT "(Raspberry pico)"

// SWDP PIO config
#define SWDP_SM 0
#define SWDP_PIN_OFFSET 2
#define SWDP_PIN_SWCLK (SWDP_PIN_OFFSET + 0) // 2
#define SWDP_PIN_SWDIO (SWDP_PIN_OFFSET + 1) // 3
#define SWDP_PIN_NRST 6
bool swdp_initted(void);
void swdp_deinit(void);

//JTAG PIO config
#define JTAG_TCK SWDP_PIN_SWCLK
#define JTAG_TMS SWDP_PIN_SWDIO
#define JTAG_TDI 4
#define JTAG_TDO 5
#define JTAG_NRST SWDP_PIN_NRST
#define JTAG_TRST 7
bool jtagtap_initted(void);
void jtagtap_deinit(void);

// UART config
#define UART_PORT_ITF 1
#define USBUSART_TX 0
#define USBUSART_RX 1
#define USBUSART_INTERFACE uart0
#define USBUSART_BAUDRATE 115200

#define GDB_PORT_ITF 0
#define CDCACM_PACKET_SIZE 64

// LED config
#ifndef LED_IDLE_RUN
#define LED_IDLE_RUN PICO_DEFAULT_LED_PIN
#endif

#define PLATFORM_HAS_TRACESWO 1
#define NUM_TRACE_PACKETS 128U /* This is an 8K buffer */
#define TRACESWO_PROTOCOL 2U   /* 1 = Manchester, 2 = NRZ / async */

#define SWO_UART uart1
#define SWO_UART_RX_PIN 9

//#define DFU_SERIAL_LENGTH (PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1)

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS DMA1
#define SWO_DMA_CLK RCC_DMA1
#define SWO_DMA_CHAN DMA_CHANNEL6
#define SWO_DMA_IRQ NVIC_DMA1_CHANNEL6_IRQ
#define SWO_DMA_ISR(x) dma1_channel6_isr(x)

extern void cdc_task(void);
//extern void tud_task(void);
extern void trace_task(void);

extern bool running_status;

#include "bsp/board.h"
#define SET_RUN_STATE(state)      \
	{                             \
		running_status = (state); \
	}
#define SET_IDLE_STATE(state)           \
	{                                   \
		platform_pace_poll();           \
		gpio_put(LED_IDLE_RUN, !state); \
	}                                  
#define SET_ERROR_STATE(state)          \
	{                                   \
	}

//stores the frequency. 
extern uint32_t plaform_frequency;

void swdtap_set_clk_freq(uint32_t clk_freq_Hz);
uint32_t swdtap_get_clk_freq();
// uart dma services
// returns the dma channel that was allocated and setup
uint platform_setup_usart_rx_dma(uart_inst_t *uart, volatile void *rx_address, irq_handler_t handler, uint buffer_size);

/* Use newlib provided integer-only stdio functions */

#ifdef sscanf
#undef sscanf
#endif
#define sscanf siscanf

#ifdef sprintf
#undef sprintf
#endif
#define sprintf siprintf

#ifdef vasprintf
#undef vasprintf
#endif
#define vasprintf vasiprintf

#ifdef snprintf
#undef snprintf
#endif
#define snprintf sniprintf

#endif /* PLATFORMS_RP_PLATFORM_H */
