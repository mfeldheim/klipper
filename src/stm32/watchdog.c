// Watchdog handler on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32H7
#include "internal.h" // IWDG
#include "sched.h" // DECL_TASK
#include "board/misc.h" // timer_read_time

#if CONFIG_MACH_STM32H7 // stm32h7 libraries only define IWDG1 and IWDG2
#define IWDG IWDG1
#endif

static uint32_t last_watchdog_time;

void
watchdog_reset(void)
{
    uint32_t cur_time = timer_read_time();
    uint32_t time_diff = cur_time - last_watchdog_time;

    // Check if watchdog reset is being called too frequently (< 100ms)
    // or too infrequently (> 400ms) - both indicate problems
    if (last_watchdog_time != 0) {
        uint32_t min_interval = timer_from_us(100000); // 100ms
        uint32_t max_interval = timer_from_us(400000); // 400ms

        if (time_diff < min_interval) {
            // Called too frequently - might indicate timing issues
            return; // Skip this reset
        } else if (time_diff > max_interval) {
            // Called too late - system might be overloaded
            // Reset anyway but this indicates a problem
        }
    }

    last_watchdog_time = cur_time;
    IWDG->KR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    IWDG->KR = 0x5555;
    IWDG->PR = 0;
    IWDG->RLR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG->KR = 0xCCCC;
}
DECL_INIT(watchdog_init);
