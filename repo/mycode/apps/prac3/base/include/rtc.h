#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include <stdio.h>

static void rtc_init(void);
uint8_t rtc_set_time_usec(uint64_t);
uint64_t rtc_get_time_usec(void);
char *rtc_get_time_formatted(void);

int cmd_rtc(const struct shell *shell, size_t argc, char *argv[]);

#endif