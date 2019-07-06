#include "ringbuffer.h"

int main(int argc, char const *argv[])
{
    uint32_t idx = 0;
    struct ringbuffer_t rb;
    int original[] = {10, 20, 30, 40, 50};
    int *reader[5] = { NULL };

    rb_init(&rb, 8);

    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        if(rb_push(&rb, &original[idx]) < 0) {
            printf("[%d]Test ringbuffer F.A.I.L\n", __LINE__);
            return -1;
        }
    }

    for (idx = 0; idx < sizeof(original)/sizeof(int); idx++) {
        if(rb_pop(&rb, (void **)&reader[idx])) {
            printf("[%d]Test ringbuffer F.A.I.L\n", __LINE__);
            return -1;
        };
        if (reader[idx])
            printf("%d ", *reader[idx]);
    }
    rb_deinit(&rb);
    printf("Test ringbuffer P.A.S.S\n");
    return 0;
}