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
#include "general.h"
#include "platform.h"
#include "morse.h"
#include <hardware/gpio.h>

struct repeating_timer timer;
uint morse_tick;
bool repeating_timer_callback(struct repeating_timer *t)
{
    if (morse_tick >= MORSECNT)
    {
        if (running_status)
            gpio_xor_mask(1 << LED_IDLE_RUN);
        SET_ERROR_STATE(morse_update());
        morse_tick = 0;
    }
    else
        ++morse_tick;
    return true;
}

void platform_timing_init(void)
{
    add_repeating_timer_ms(10, repeating_timer_callback, NULL, &timer);
}   