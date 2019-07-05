#ifndef __ringbuffer_h__ 
#define __ringbuffer_h__

#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
struct ringbuffer_t {
    uint32_t head;
    uint32_t tail;
    uint32_t ring_size;
    uint32_t full_cnt;
    pthread_mutex_t ring_lock;
    void **p_cell;
};

#ifdef __cplusplus
extern "C" {
#endif
int rb_init(struct ringbuffer_t *rb, size_t size);
int rb_deinit(struct ringbuffer_t *rb);
int rb_push(struct ringbuffer_t *rb, void *payload);
int rb_pop(struct ringbuffer_t *rb, void **payload);
#ifdef __cplusplus
}
#endif


#endif  //  __ringbuffer_h__