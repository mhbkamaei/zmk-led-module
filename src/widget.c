/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/split/bluetooth/peripheral.h>

#include <zephyr/drivers/gpio.h>
#include <zmk_led_widget/widget.h>


#define LED_GPIO_NODE_ID DT_COMPAT_GET_ANY_STATUS_OKAY(gpio_leds)

BUILD_ASSERT(DT_NODE_EXISTS(DT_ALIAS(led_redd)),
             "An alias for a red LED is not found for RGBLED_WIDGET");
BUILD_ASSERT(DT_NODE_EXISTS(DT_ALIAS(led_green)),
             "An alias for a green LED is not found for RGBLED_WIDGET");
BUILD_ASSERT(DT_NODE_EXISTS(DT_ALIAS(led_blue)),
             "An alias for a blue LED is not found for RGBLED_WIDGET");

// GPIO-based LED device and indices of red/green/blue LEDs inside its DT node
static const struct device *led_dev = DEVICE_DT_GET(LED_GPIO_NODE_ID);
static const uint8_t rgb_idx[] = {DT_NODE_CHILD_IDX(DT_ALIAS(led_red)),
                                  DT_NODE_CHILD_IDX(DT_ALIAS(led_green)),
                                  DT_NODE_CHILD_IDX(DT_ALIAS(led_blue))};

// color values as specified by an RGB bitfield
enum led_color_t {
    LED_BLACK,   // 0b000
    LED_RED,     // 0b001
    LED_GREEN,   // 0b010
    LED_YELLOW,  // 0b011
    LED_BLUE,    // 0b100
    LED_MAGENTA, // 0b101
    LED_CYAN,    // 0b110
    LED_WHITE    // 0b111
};

// a blink work item as specified by the color and duration
struct blink_item {
    enum led_color_t color;
    uint16_t duration_ms;
    bool first_item;
    uint16_t sleep_ms;
};

K_MSGQ_DEFINE(led_msgq, sizeof(struct blink_item), 16, 1);


/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

extern void led_process_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    while (true) {
        // wait until a blink item is received and process it
        struct blink_item blink;
        k_msgq_get(&led_msgq, &blink, K_FOREVER);

        // turn appropriate LEDs on
        for (uint8_t pos = 0; pos < 3; pos++) {
            if (BIT(pos) & blink.color) {
                led_on(led_dev, rgb_idx[pos]);
            }
        }

        // wait for blink duration
        k_sleep(K_MSEC(1000));

        // turn appropriate LEDs off
        for (uint8_t pos = 0; pos < 3; pos++) {
            if (BIT(pos) & blink.color) {
                led_off(led_dev, rgb_idx[pos]);
            }
        }

        // wait interval before processing another blink
        k_sleep(K_MSEC(1000));
    }
}

void indicate_battery(void) {
    struct blink_item blink = {.duration_ms = 1000, .first_item = true};
        blink.color = LED_RED;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        blink.color = LED_GREEN;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        blink.color = LED_CYAN;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        blink.color = LED_WHITE;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        blink.color = LED_MAGENTA;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        blink.color = LED_YELLOW;
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
}

extern void led_init_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    indicate_battery();

    // wait until blink should be displayed for further checks
    k_sleep(K_MSEC(2000));
}

// define led_process_thread with stack size 1024, start running it 100 ms after
// boot
K_THREAD_DEFINE(led_process_tid, 1024, led_process_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);
// run init thread on boot for initial battery+output checks
K_THREAD_DEFINE(led_init_tid, 1024, led_init_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 200);
