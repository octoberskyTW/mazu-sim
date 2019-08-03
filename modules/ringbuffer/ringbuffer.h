#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct ringbuffer_t {
    uint32_t head;
    uint32_t tail;
    uint32_t size;
    uint32_t full_cnt;
    pthread_mutex_t ring_lock;
    void **p_data;
};

enum { RB_ERROR = -1, RB_SUCCESS = 0 };

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


#endif  //  __RINGBUFFER_H__