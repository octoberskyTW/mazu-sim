#include "dcmbus_util.h"
int get_arr_num(int arrary_size, int element_size) {
    if (arrary_size % element_size) {
        fprintf(stderr, "Arrary Size and Element size are not maching\n");
        return -1;
    }
    return (arrary_size/element_size);
}

void debug_hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen) {
    if (DCMBUS_DEBUG_ENABLE == 0)
        return;
    uint8_t *pt;
    uint32_t x;
    pt = pSrcBufVA;
    debug_print("%s: %p, len = %d\n\r", str, pSrcBufVA, SrcBufLen);
    for (x = 0; x < SrcBufLen; x++) {
        if (x % 16 == 0) {
            debug_print("0x%04x : ", x);
        }
        debug_print("%02x ", ((uint8_t)pt[x]));
        if (x % 16 == 15) { debug_print("\n\r"); }
    }
    debug_print("\n\r");
}

static struct timespec g_timestamp;
void clock_get_hw_time(struct timespec *ts) {
    clock_gettime(CLOCK_MONOTONIC, ts);
}

double get_curr_time(void) {
    clock_get_hw_time(&g_timestamp);
    return g_timestamp.tv_sec + (double)g_timestamp.tv_nsec / (double)BILLION;
}