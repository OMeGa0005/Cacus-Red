#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/shell/shell.h>
#include <time.h>

#include <string.h>

/*
 * RTC (Real-Time Clock) module for managing time in the system.
 * Provides functionality to initialize, set, and retrieve time.
 */

const struct device *const rtcc = DEVICE_DT_GET(DT_ALIAS(rtcc0));
LOG_MODULE_REGISTER(rtc, LOG_LEVEL_DBG);

/* Global state to convert RTCC ticks to an absolute time. */
static time_t base_time = 0;   // Absolute time (Unix epoch seconds) when the time was set
static uint64_t base_usec = 0; // RTCC microseconds count at that same moment

/*
 * @brief Initializes the RTC device and starts the counter.
 */
void rtc_init(void) {

    if (!device_is_ready(rtcc)) {
        LOG_ERR("RTCC device is not ready");
        return;
    }

    counter_start(rtcc);
}

/*
 * @brief Sets the RTC time in microseconds.
 *
 * @param usec Time in microseconds to set.
 * @return 0 on success.
 */
uint8_t rtc_set_time_usec(uint64_t usec) {

    return 0;
}

/*
 * @brief Retrieves the current RTC time in microseconds.
 *
 * @return Current time in microseconds.
 */
uint64_t rtc_get_time_usec(void) {

    uint64_t ticks;
    int errVal = counter_get_value(rtcc, &ticks);

    if (errVal) {
        LOG_ERR("Error while getting RTC counter value");
        return 0;
    }

    uint64_t usec = counter_ticks_to_us(rtcc, ticks);

    return usec;
}

/*
 * @brief Formats the current RTC time as a human-readable string.
 *
 * @return Pointer to a static buffer containing the formatted time string.
 */
char * rtc_get_time_formatted(void) {
    uint64_t current_usec = rtc_get_time_usec();
    uint64_t elapsed_usec = current_usec - base_usec;

    /* Convert elapsed microseconds to seconds. */
    time_t elapsed_sec = elapsed_usec / 1000000;

    /* Compute the current time by adding elapsed seconds to the base time. */
    time_t now = base_time + elapsed_sec;

    struct tm tm_time;
    // Convert Unix timestamp to a time structure
    gmtime_r(&now, &tm_time);

    static char buffer[64];

    /* Format the time string. For example, "YYYY-MM-DD HH:MM:SS" */
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_time);

    return buffer;
}

/*
 * @brief Shell command to read or write RTC time.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success, -EINVAL on invalid arguments.
 */
int cmd_rtc(const struct shell *shell, size_t argc, char *argv[])
{
    // If no optional arguments, retrieve total seconds
    if (argc == 2 | argc == 3 | argc == 4)
    {
        // Get the current time
        if (strcmp(argv[1], "r") == 0)
        {

            uint64_t current_usec = rtc_get_time_usec();
            uint64_t elapsed_usec = current_usec - base_usec;

            /* Convert elapsed microseconds to seconds. */
            time_t elapsed_sec = elapsed_usec / 1000000;

            /* Compute the current time by adding elapsed seconds to the base time. */
            time_t now = base_time + elapsed_sec;

            struct tm tm_time;
            // Convert Unix timestamp to a time structure
            gmtime_r(&now, &tm_time);

            char buffer[64];

            /* Format the time string. For example, "YYYY-MM-DD HH:MM:SS" */
            strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_time);

            LOG_INF("Current time and date: %s", buffer);
        }

        // Set the current time
        else if (strcmp(argv[1], "w") == 0 && argc >= 3)
        {
            char datetime[64] = {0};
            if (argc == 4)
            {
                strcat(datetime, argv[2]);
                strcat(datetime, " ");
                strcat(datetime, argv[3]);
            }
            else if (argc == 3)
            {
                strcpy(&datetime, argv[2]);
            }
            else
            {
                LOG_ERR("Invalid date/time format. Expected: YYYY-MM-DD HH:MM:SS");

                // We should probably return something other than zero for clarity
                // Even though no one is going to read this
                return -EINVAL;
            }

            struct tm tm_time = {0};

            if (strptime(datetime, "%Y-%m-%d %H:%M:%S", &tm_time) == NULL)
            {
                LOG_ERR("Invalid date/time format. Expected: YYYY-MM-DD HH:MM:SS");
                LOG_ERR("\r\nUsage:\r\n\nrtc <command> <time>\r\n\tw (write)\r\n\tr (read)\r\n");
                return -EINVAL;
            }

            // Convert the parsed tm struct into a timestamp
            time_t new_time = timegm(&tm_time);
            base_time = new_time;
            base_usec = rtc_get_time_usec();

            LOG_INF("Set current time and date to: %s", datetime);
        }

        else
        {
            LOG_ERR("Invalid command");
            LOG_ERR("\r\nUsage:\r\n\nrtc <command> <time>\r\n\tw (write)\r\n\tr (read)\r\n");

            // We should probably return something other than zero for clarity
            // Even though no one is going to read this
            return -EINVAL;
        }
    }

    else
    {
        LOG_ERR("Invalid command");
        LOG_ERR("\r\nUsage:\r\n\nrtc <command> <time>\r\n\tw (write)\r\n\tr (read)\r\n");
        return -EINVAL;
    }

    return 0;
}