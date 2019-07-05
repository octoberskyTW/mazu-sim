#include "ringbuffer.h"

int rb_init(struct ringbuffer_t *rb, size_t size) {
    int rc = 0;
    rb->head = 0;
    rb->tail = 0;
    rb->ring_size = size;
    rb->full_cnt = 0;

    rb->p_cell = (void **)calloc(size, sizeof(void *));
    if (!rb->p_cell) {
        rc = -1;
        fprintf(stderr, "[%s:%d] calloc Fail\n", __FUNCTION__, __LINE__);
        return rc;
    }

    rc = pthread_mutex_init(&rb->ring_lock, NULL);
    if (rc < 0) {
        fprintf(stderr, "[%s:%d] %s\n", __FUNCTION__, __LINE__, strerror(errno));
        free(rb->p_cell);
        goto rb_init_exit;
    }
rb_init_exit:
    return rc;
}