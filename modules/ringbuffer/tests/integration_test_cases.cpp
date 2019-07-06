#include "ringbuffer.h"

int ringbuffer_basic_test(void)
{
    uint32_t idx = 0;
    uint32_t rc = 0;
    struct ringbuffer_t rb;
    int original[] = {10, 20, 30, 40, 50};
    int *reader[5] = { NULL };

    if(rb_init(&rb, 8))
        rc |= 0x1;

    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        if(rb_push(&rb, &original[idx]) < 0) {
            rc |= 0x2;
            printf("[%s:%d] Test rb_push Error\n", __FUNCTION__, __LINE__);
        }
    }

    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        if(rb_pop(&rb, (void **)&reader[idx]) < 0) {
            printf("[%s:%d] Test rb_pop Error\n", __FUNCTION__, __LINE__);
            rc |= 0x3;
        };
    }

    if(rb_deinit(&rb))
        rc |= 0x4;
    if(rc)
        printf("Test %s FAIL 0x%x\n", __FUNCTION__, rc);
    return 0;
}