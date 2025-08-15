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

#include <hardware/clocks.h>
#include "pio_jtag.h"
#include "jtag.pio.h"

void jtag_task();//to process USB OUT packets while waiting to finish


void __time_critical_func(pio_jtag_write_blocking)(const pio_jtag_inst_t *jtag, const uint8_t *bsrc, size_t len) 
{
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
    io_rw_32 *rxfifo = (io_rw_32 *) &jtag->pio->rxf[jtag->sm];
    uint32_t x; // scratch local to receive data
    //kick off the process by sending the len to the tx pipeline
    *(io_rw_32*)txfifo = len-1;
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm))
            {
                *txfifo = *bsrc++;
                --tx_remain;
            }
            if ((tx_remain & 3) == 0)
                jtag_task();
            if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm))
            {
                x = *rxfifo;
                --rx_remain;
            }
        }
    }
}

void __time_critical_func(pio_jtag_write_read_blocking)(const pio_jtag_inst_t *jtag, const uint8_t *bsrc, uint8_t *bdst,
                                                         size_t len) 
{
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    uint8_t* rx_last_byte_p = &bdst[byte_length-1];
    io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
    io_rw_32 *rxfifo = (io_rw_32 *) &jtag->pio->rxf[jtag->sm];
    //kick off the process by sending the len to the tx pipeline
    *(io_rw_32*)txfifo = len-1;
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm))
            {
                *txfifo = *bsrc++;
                --tx_remain;
            }
            if ((tx_remain & 3) == 0)
                jtag_task();
            if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm))
            {
                *bdst++ = (*rxfifo) >> 24;
                --rx_remain;
            }
        }
    }
    //fix the last byte
    if (last_shift) 
    {
        *rx_last_byte_p = *rx_last_byte_p >> last_shift;
    }
}

uint8_t __time_critical_func(pio_jtag_write_tms_blocking)(const pio_jtag_inst_t *jtag, bool tdi, bool tms, size_t len)
{
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
    io_rw_32 *rxfifo = (io_rw_32 *) &jtag->pio->rxf[jtag->sm];
    uint8_t x = 0; // scratch local to receive data
    uint8_t tdi_word = tdi ? 0xFF : 0x0;
    gpio_put(jtag->pin_tms, tms);
    //kick off the process by sending the len to the tx pipeline
    *(io_rw_32*)txfifo = len-1;
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm)) 
            {
                *txfifo = tdi_word;
                --tx_remain;
            }
            if ((tx_remain & 3) == 0)
                jtag_task();
            if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm))
            {
                x = (*rxfifo) >> 24;
                --rx_remain;
            }
        }
    }
    //fix the last byte
    if (last_shift)
    {
        x = x >> last_shift;
    }
    return x;
}

static void init_pins(uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst, uint pin_trst)
{
    // emulate open drain with pull up and direction
    gpio_pull_up(pin_rst);
    gpio_clr_mask((1u << pin_tms) | (1u << pin_rst) | (1u << pin_trst));
    gpio_init_mask((1u << pin_tms) | (1u << pin_rst) | (1u << pin_trst));
    gpio_set_dir_masked( (1u << pin_tms) | (1u << pin_trst), 0xffffffffu);
    gpio_set_dir(pin_rst, false);
}

void init_jtag(pio_jtag_inst_t* jtag, uint freq_hz, uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst, uint pin_trst)
{
    if (!jtag->initted)
    {
        init_pins(pin_tck, pin_tdi, pin_tdo, pin_tms, pin_rst, pin_trst);
        jtag->pin_tdi = pin_tdi;
        jtag->pin_tdo = pin_tdo;
        jtag->pin_tck = pin_tck;
        jtag->pin_tms = pin_tms;
        jtag->pin_rst = pin_rst;
        jtag->pin_trst = pin_trst;
        uint16_t clkdiv = 31;  // around 1 MHz @ 125MHz clk_sys
        jtag->offset = pio_jtag_init(jtag->pio, jtag->sm,
                            clkdiv,
                            pin_tck,
                            pin_tdi,
                            pin_tdo
                        );

        jtag_set_clk_freq(jtag, freq_hz);
        jtag->initted = 1;
    }
}

void jtag_deinit(pio_jtag_inst_t *jtag)
{
    if (jtag->initted)
    {
        pio_sm_set_enabled(pio0, 0, 0);
        pio_remove_program(pio0, &djtag_tdo_program, jtag->offset);
        jtag->initted = 0;
    }
}


void jtag_set_clk_freq(pio_jtag_inst_t *jtag, uint freq_hz) {
    uint clk_sys_freq = clock_get_hz(clk_sys);
    uint32_t divider = (clk_sys_freq / freq_hz) / 4;
    divider = (divider < 2) ? 2 : divider; //max reliable freq
    divider = (divider > 0XFFFF) ? 0xFFFF : divider;
    jtag->sm_divider = divider;
    pio_sm_set_clkdiv_int_frac(pio0, jtag->sm, divider, 0);
}

void jtag_transfer(const pio_jtag_inst_t *jtag, uint32_t length, const uint8_t *in, uint8_t *out)
{
    /* set tms to low */
    jtag_set_tms(jtag, false);

    if (out)
        pio_jtag_write_read_blocking(jtag, in, out, length);
    else
        pio_jtag_write_blocking(jtag, in, length);

}

bool jtag_strobe(const pio_jtag_inst_t *jtag, uint32_t length, bool tms, bool tdi)
{
    return !!pio_jtag_write_tms_blocking(jtag, tdi, tms, length);
}



static uint8_t toggle_bits_out_buffer[4];
static uint8_t toggle_bits_in_buffer[4];

void jtag_set_tdi(const pio_jtag_inst_t *jtag, bool value)
{
    toggle_bits_out_buffer[0] = value ? 1 : 0;
}

void jtag_set_clk(const pio_jtag_inst_t *jtag, bool value)
{
    if (value)
    {
        toggle_bits_in_buffer[0] = 0; 
        pio_jtag_write_read_blocking(jtag, toggle_bits_out_buffer, toggle_bits_in_buffer, 1);
    }
}

bool jtag_get_tdo(const pio_jtag_inst_t *jtag)
{
    return !! toggle_bits_in_buffer[0];
}


