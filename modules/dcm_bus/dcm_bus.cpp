#include "ringbuffer.h"

int dcm_bus_load_cfg (void) {
    struct ringbuffer_t rb;
    rb_init(&rb, 128);
    rb_deinit(&rb);
    printf("[dutsai] %s\n", __FUNCTION__);
    return 0;
}