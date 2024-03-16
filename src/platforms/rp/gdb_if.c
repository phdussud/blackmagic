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
 * This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented. This implementation for STM32
 * uses the USB CDC-ACM device bulk endpoints to implement the channel.
 */
#include <pico/stdlib.h>
#include "tusb.h"
#include "general.h"
#include "gdb_if.h"
#include "platform.h"

static uint32_t count_out;
static uint32_t count_in;
static uint32_t out_ptr;
static char buffer_out[CFG_TUD_CDC_RX_BUFSIZE];
static char buffer_in[CFG_TUD_CDC_TX_BUFSIZE];
#ifdef STM32F4
static volatile uint32_t count_new;
static char double_buffer_out[CDCACM_PACKET_SIZE];
#endif
static int was_connected = 0;
void gdb_if_putchar(const char c, const int flush)
{
    buffer_in[count_in++] = c;
    if (flush || count_in == CDCACM_PACKET_SIZE)
    {
        /* Refuse to send if USB isn't configured, and
         * don't bother if nobody's listening */
        if (!tud_cdc_n_connected(GDB_PORT_ITF))
        {
            count_in = 0;
            if (was_connected)
            {
                tud_cdc_n_write_clear(GDB_PORT_ITF);
                was_connected = 0;
            }
            return;
        }
        was_connected = 1; 
        while (count_in > 0) 
        {
            uint32_t written = tud_cdc_n_write(GDB_PORT_ITF, buffer_in, count_in);
            count_in -= written;
            platform_pace_poll();
            tud_cdc_n_write_flush(GDB_PORT_ITF);
        }
        if (flush)
        {
            platform_pace_poll();
            tud_cdc_n_write_flush(GDB_PORT_ITF);
        }
    }
}

char gdb_if_getchar(void)
{
    while (out_ptr >= count_out)
    {
        platform_pace_poll();
        if (tud_cdc_n_connected(GDB_PORT_ITF))
        {
            was_connected = 1;
            uint ra = tud_cdc_n_available(GDB_PORT_ITF);
            size_t watermark = MIN(ra, sizeof(buffer_out));
            if (watermark > 0)
            {
                count_out = tud_cdc_n_read(GDB_PORT_ITF, buffer_out, watermark);
                out_ptr = 0;
            }
        } else if (was_connected)
        {
            tud_cdc_n_read_flush(GDB_PORT_ITF);
            was_connected = 0;
            return '\x04';
        }
    }
    return buffer_out[out_ptr++];
}

char gdb_if_getchar_to(const uint32_t timeout)
{
    platform_timeout_s receive_timeout;
    platform_timeout_set(&receive_timeout, timeout);

    if (out_ptr < count_out)
        return buffer_out[out_ptr++];
    else 
    {
        while (tud_cdc_n_available(GDB_PORT_ITF) == 0 && !platform_timeout_is_expired(&receive_timeout))
        {
            platform_pace_poll();
        }
        if (tud_cdc_n_available (GDB_PORT_ITF))
            return gdb_if_getchar();
        else /* XXX: Need to find a better way to error return than this. This provides '\xff' characters. */
            return -1;
    } 
}
