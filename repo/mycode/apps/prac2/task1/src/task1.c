#include "task1.h"


static int rtc_time(const struct shell *shell, size_t argc, char *argv[]) {
	if(argv[1][0]=='r'){
		get_date_time();
	}
	else if(argv[1][0]=='w'){
		set_date_time(shell, argc, argv);
	}
}

static int set_date_time(const struct shell *shell, size_t argc, char *argv[]) {
	printf("%d\n", argc);
    int year = atoi(argv[2]);
    int mon = atoi(argv[3]);
    int date = atoi(argv[4]);
    int hour = atoi(argv[5]);
    int min = atoi(argv[6]);
    int sec = atoi(argv[7]);
	int ret = 0;
	struct rtc_time tm = {
		.tm_year = year - 1900,
		.tm_mon = mon - 1,
		.tm_mday = date,
		.tm_hour = hour,
		.tm_min = min,
		.tm_sec = sec,
	};

	ret = rtc_set_time(rtc, &tm);
	if (!device_is_ready(rtc)) {
		printf("RTC device is not ready\n");
		return 1;
	}
	if (ret < 0) {
		printf("Cannot write date time: %d\n", ret);
		return 1;
	}
	return 0;
}

static int get_date_time() {
	int ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		printf("Cannot read date time: %d\n", ret);
		return 1;
	}
	printf("<%04d-%02d-%02d %02d:%02d:%02d>\n", tm.tm_year + 1900,
	       tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	return 0;
}
static char* get_date_time_json() {
	static char datetime[64];
	int ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		printf("Cannot read date time: %d\n", ret);
		return 1;
	}
	snprintf(datetime, sizeof(datetime), "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	return datetime;
}
