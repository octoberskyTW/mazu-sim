#include "ringbuffer.h"

int rb_init(struct ringbuffer_t *rb, size_t size) {
    int rc = 0;
    rb->head = 0;
    rb->tail = 0;
    rb->size = size;
    rb->full_cnt = 0;
    rb->p_data = NULL;
    if (rb->size & (rb->size - 1)) {
        rc = -1;
        fprintf(stderr, "[%s:%d] Ring size must power of two\n", __FUNCTION__, __LINE__);
        goto rb_init_exit;
    }
    rb->p_data = (void **)calloc(size, sizeof(void *));
    if (!rb->p_data) {
        rc = -1;
        fprintf(stderr, "[%s:%d] calloc Fail\n", __FUNCTION__, __LINE__);
        goto rb_init_exit;
    }

    rc = pthread_mutex_init(&rb->ring_lock, NULL);
    if (rc < 0) {
        fprintf(stderr, "[%s:%d] %s\n", __FUNCTION__, __LINE__, strerror(errno));
        free(rb->p_data);
        goto rb_init_exit;
    }
rb_init_exit:
    return rc;
}

int rb_deinit(struct ringbuffer_t *rb) {
    int rc = 0;
    pthread_mutex_destroy(&rb->ring_lock);
    if (rb->p_data) {
        free(rb->p_data);
        rb->p_data = NULL;
    }
    fprintf(stderr, "[%s] Full count: %d \n", __FUNCTION__, rb->full_cnt);
    return rc;
}


static inline bool rb_is_empty(const struct ringbuffer_t *rb)
{
    return rb->head == rb->tail;
}

static inline bool rb_is_full(const struct ringbuffer_t *rb)
{
    if (rb->head < rb->tail) {
        return  (rb->size - 1) == (rb->tail - rb->head);
    }
    return (rb->head - rb->tail) == 1;
}


int rb_push(struct ringbuffer_t *rb, void *payload) 
{
    int rc = 0;
    if(!rb->p_data)
        return -1;
    pthread_mutex_lock(&rb->ring_lock);
    if (rb_is_full(rb)) {
        //  ring buffer is full
        rb->full_cnt++;
        pthread_mutex_unlock(&rb->ring_lock);
        return rc;
    }
    rb->p_data[rb->tail] = payload;
    rb->tail++;
    rb->tail &= (rb->size - 1);
    pthread_mutex_unlock(&rb->ring_lock);
    return rc;
}

int rb_pop(struct ringbuffer_t *rb, void **payload)
{
    int rc = 0;
    if(!rb->p_data)
        return -1;
    pthread_mutex_lock(&rb->ring_lock);
    if (rb_is_empty(rb)) {
        pthread_mutex_unlock(&rb->ring_lock);
        return rc;
    }

    *payload = rb->p_data[rb->head];
    rb->head++;
    rb->head &= (rb->size - 1);
    pthread_mutex_unlock(&rb->ring_lock);
    return rc;
}



