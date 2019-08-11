#include <stdint.h>
#include <stdio.h>
#include <time.h>
void timestamp(char *currentTime)
{
    char date_buf[80];
    static struct timespec ts;
    uint32_t milli;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_sec = time(NULL);
    milli = ts.tv_nsec / 1000000;
    strftime(date_buf, (size_t) 20, "%Y/%m/%d,%H:%M:%S", localtime(&ts.tv_sec));
    snprintf(currentTime, 84, "%s.%03d", date_buf, milli);
}