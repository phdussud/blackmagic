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
#include <stdint.h>

#define PICOPROBE_LED PICO_DEFAULT_LED_PIN

#define LED_COUNT_SHIFT 14
#define LED_COUNT_MAX 5 * (1 << LED_COUNT_SHIFT)

static uint32_t led_count;

void led_init(void) {
    led_count = 0;

    gpio_init(PICOPROBE_LED);
    gpio_set_dir(PICOPROBE_LED, GPIO_OUT);
    gpio_put(PICOPROBE_LED, 1);
}



void led_task(void) {
    if (led_count != 0) {
        --led_count;
        gpio_put(PICOPROBE_LED, !((led_count >> LED_COUNT_SHIFT) & 1));
    }
}

void led_signal_activity(uint total_bits) {
    if (led_count == 0) {
        gpio_put(PICOPROBE_LED, 0);
    }

    if (led_count < LED_COUNT_MAX) {
        led_count += total_bits;
    }
}
