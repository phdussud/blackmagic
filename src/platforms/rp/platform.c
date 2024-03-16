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

/* This file implements the platform specific functions for the 96Boards Carbon implementation. */
#include "platform.h"
#include "general.h"
#include "morse.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include <hardware/clocks.h>
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/bootrom.h"
#include "bsp/board.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "cdc_uart.h"
#include "get_serial.h"
#include "jtagtap.h"
#include "adiv5.h"

jmp_buf fatal_error_jmpbuf;
void platform_timing_init(void);

void jtagtap_set_clk_freq(uint32_t clk_freq_Hz);

uint32_t jtagtap_get_clk_freq(void);

void core1_loop()
{
    cdc_uart_init();
    while (1)
    {
        cdc_task();
        trace_task();
    }
}

void platform_init(void)
{
    //    stdio_init_all();
    //    board_init();
    gpio_init(LED_IDLE_RUN);
    gpio_set_dir(LED_IDLE_RUN, true);
    usb_serial_init();
    tusb_init();
    // Target reset pin: pull up, input to emulate open drain pin
    gpio_pull_up(SWDP_PIN_NRST);
    // gpio_init will leave the pin cleared and set as input
    gpio_init(SWDP_PIN_NRST);
    multicore_reset_core1();
    multicore_launch_core1(core1_loop);
    platform_timing_init();
}

void platform_pace_poll(void)
{
	tud_task();
}
int platform_hwversion(void)
{
	return 0;
}


void platform_nrst_set_val(bool assert)
{
    /* Change the direction to out to drive pin to 0 or to in to emulate open drain */
    gpio_set_dir(SWDP_PIN_NRST, assert);
    sleep_ms(1);
}

bool platform_nrst_get_val(void)
{
    return gpio_get(SWDP_PIN_NRST);
}

const char *platform_target_voltage(void)
{
    return NULL;
}

void platform_request_boot(void)
{
    reset_usb_boot(0, 1); //disable mass storage interface
}

void platform_target_clk_output_enable(bool enable)
{
    (void)enable;
}

uint32_t platform_time_ms()
{
    return to_ms_since_boot(get_absolute_time());
}

void platform_delay(uint32_t ms)
{
    sleep_ms(ms);
}

uint32_t plaform_frequency = 1000 * 1000;

void platform_max_frequency_set(uint32_t freq_hz)
{
    plaform_frequency = freq_hz;
    if (jtagtap_initted())
        jtagtap_set_clk_freq(freq_hz);
    else if (swdp_initted())
    {
        swdtap_set_clk_freq(freq_hz);
    }
}


uint32_t platform_max_frequency_get(void)
{
    if (jtagtap_initted())
        return jtagtap_get_clk_freq();
    else if (swdp_initted())
        return swdtap_get_clk_freq();
    else
        return plaform_frequency;
}

bool running_status;

void tud_dfu_runtime_reboot_to_dfu_cb(void)
{
    platform_request_boot();
}

// this is to work around the fact that tinyUSB does not handle setup request automatically
// Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP)
        return true;
    return false;
}

uint platform_setup_usart_rx_dma(uart_inst_t *uart, volatile void *rx_address, irq_handler_t handler, uint buffer_size)
{
    uint dma_chan = dma_claim_unused_channel(true);
    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_add_shared_handler(DMA_IRQ_0, handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
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
        rx_address,             // Write Address
        &uart_get_hw(uart)->dr, // Read Address
        buffer_size,            // transfer count
        true                    // start
    );
    return dma_chan;
}

//right now SPI isn't supported
bool platform_spi_init(const spi_bus_e bus)
{
    (void)bus;
    return false;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
    (void)bus;
    return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
    (void)device_select;
    return false;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
    (void)bus;
    return value;
}