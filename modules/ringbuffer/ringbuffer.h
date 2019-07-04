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
    void *pCell;
};

int  rb_init(struct ringbuffer_t *rb, uint32_t size);
void rb_deinit(struct ringbuffer_t *rb);
void rb_push(struct ringbuffer_t *rb, void *payload);
void *rb_pop(struct ringbuffer_t *rb);

#endif  //  __ringbuffer_h__