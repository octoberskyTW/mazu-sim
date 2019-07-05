#include "ringbuffer.h"

int main(int argc, char const *argv[])
{
    uint32_t idx = 0;
    struct ringbuffer_t rb;
    int original[] = {10, 20, 30, 40, 50};

    rb_init(&rb, sizeof(original)/sizeof(int));
    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        rb.p_cell[idx] = &original[idx];
    }

    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        if(rb.p_cell[idx] != &original[idx]) {
            printf("[%d]Test ringbuffer F.A.I.L\n", __LINE__);
            return -1;
        };
    }
    printf("Test ringbuffer P.A.S.S\n");
    return 0;
}