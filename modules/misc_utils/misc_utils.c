#include <stdint.h>
#include <stdio.h>
#include <time.h>

void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen)
{
    uint8_t *pt;
    uint32_t x;
    pt = pSrcBufVA;
    printf("%s: %p, len = %d\n\r", str, pSrcBufVA, SrcBufLen);
    for (x = 0; x < SrcBufLen; x++) {
        if (x % 16 == 0) {
            printf("0x%04x : ", x);
        }
        printf("%02x ", ((uint8_t) pt[x]));
        if (x % 16 == 15) {
            printf("\n\r");
        }
    }
    printf("\n\r");
}

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