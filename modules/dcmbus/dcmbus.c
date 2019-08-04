/* Copyright (c) 2019, Dung-Ru Tsai
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "dcmbus.h"
#include "dcmbus_util.h"

static struct channel_config g_channel_config[8];
static struct ring_config g_ring_config[16];
static struct bind_config g_bind_config[32];
static struct channel_type_enum_t chan_type_enum_tbl[] = {
    { DCMBUS_SOCKET_ETH, "socket_eth" },
    { DCMBUS_SOCKET_CAN, "socket_can" },
    { DCMBUS_DEV_RS422, "dev_rs422" }
};

static void *dcmbus_alloc_mem(size_t size)
{
    return calloc(1, size);
}

static void dcmbus_free_mem(void **ptr)
{
    if (*ptr) {
        free(*ptr);
        *ptr = NULL;
    }
    return;
}

static int dcmbus_load_ring_conf(const char *path)
{
    int numEntries, idx;
    char specifier[] = RING_SPECIFIER;
    memset(g_ring_config, 0, sizeof(g_ring_config));
    read_config(g_ring_config, &numEntries, path, specifier);
    printf("Load ring Entries: %d \n", numEntries);
    printf("%16s %16s %16s %16s\n", RING_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, RING_PRINTF_FORMAT, g_ring_config[idx].rg_name,
                g_ring_config[idx].enable, g_ring_config[idx].direction,
                g_ring_config[idx].ring_size);
    }
    return numEntries;
}

static int dcmbus_load_channel_conf(const char *path)
{
    int numEntries, idx;
    char specifier[] = CHANNEL_SPECIFIER;
    memset(g_channel_config, 0, sizeof(g_channel_config));
    read_config(g_channel_config, &numEntries, path, specifier);
    printf("Load channel Entries: %d \n", numEntries);
    printf("%16s %16s %16s %16s %16s %16s %16s %16s %16s\n",
           CHANNEL_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, CHANNEL_PRINTF_FORMAT, g_channel_config[idx].ch_name,
                g_channel_config[idx].enable, g_channel_config[idx].direction,
                g_channel_config[idx].type, g_channel_config[idx].ifname,
                g_channel_config[idx].netport, g_channel_config[idx].driver_idx,
                g_channel_config[idx].blocking, g_channel_config[idx].options);
    }
    return numEntries;
}

static int dcmbus_load_bind_conf(const char *path)
{
    int numEntries, idx;
    char specifier[] = BIND_SPECIFIER;

    memset(g_bind_config, 0, sizeof(g_bind_config));
    read_config(g_bind_config, &numEntries, path, specifier);
    printf("Load Bind Entries: %d \n", numEntries);
    printf("%16s %16s %16s\n", BIND_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, BIND_PRINTF_FORMAT, g_bind_config[idx].ring,
                g_bind_config[idx].channel, g_bind_config[idx].direction);
    }
    return numEntries;
}

static int dcmbus_get_channel_type_enum(const char *type_name)
{
    int rc = 0;
    int idx;
    for (idx = 0; idx < (int) ARRAY_SIZE(chan_type_enum_tbl); idx++) {
        if (strcmp(chan_type_enum_tbl[idx].type_name, type_name) == 0) {
            rc = chan_type_enum_tbl[idx].enumval;
        }
    }
    return rc;
}

static int dcmbus_bind_ring(struct dcmbus_ctrlblk_t *D,
                            struct dcmbus_channel_blk_t *ch_obj)
{
    int idx;
    struct dcmbus_bind_entry_t *bind_item = NULL, *is_b = NULL;
    struct dcmbus_ring_blk_t *rg_iter = NULL, *is_rg = NULL;
    INIT_LIST_HEAD(&ch_obj->txbind_lhead);
    INIT_LIST_HEAD(&ch_obj->rxbind_lhead);
    for (idx = 0; idx < D->num_binds; ++idx) {
        if (strcmp(g_bind_config[idx].channel, ch_obj->ch_name) == 0) {
            bind_item =
                (struct dcmbus_bind_entry_t *) malloc(sizeof(*bind_item));
            memset(bind_item, 0, sizeof(*bind_item));
            if (!bind_item) {
                fprintf(stderr, "%s:%d: ERROR Binding allocate memory fail.\n",
                        __func__, __LINE__);
                goto error_bind;
            }
            strcpy(bind_item->ring, g_bind_config[idx].ring);
            strcpy(bind_item->channel, g_bind_config[idx].channel);

            if (strcmp(g_bind_config[idx].direction, "TX") == 0) {
                bind_item->dir = DCMBUS_DIR_TX;
                list_for_each_entry_safe(rg_iter, is_rg, &D->ring_lhead, list)
                {
                    if ((strcmp(bind_item->ring, rg_iter->rg_name) == 0) &&
                        (rg_iter->dir == DCMBUS_DIR_TX)) {
                        bind_item->p_data_ring = &rg_iter->data_ring;
                        fprintf(stderr,
                                "Channel %s and TX Ring %s bind at 0x%p.\n",
                                bind_item->channel, bind_item->ring,
                                (void *) bind_item->p_data_ring);
                        list_add_tail(&bind_item->list, &ch_obj->txbind_lhead);
                    }
                }
            } else {
                bind_item->dir = DCMBUS_DIR_RX;
                list_for_each_entry_safe(rg_iter, is_rg, &D->ring_lhead, list)
                {
                    if ((strcmp(bind_item->ring, rg_iter->rg_name) == 0) &&
                        (rg_iter->dir == DCMBUS_DIR_RX)) {
                        bind_item->p_data_ring = &rg_iter->data_ring;
                        fprintf(stderr,
                                "Channel %s and RX Ring %s bind at 0x%p.\n",
                                bind_item->channel, bind_item->ring,
                                (void *) bind_item->p_data_ring);
                        list_add_tail(&bind_item->list, &ch_obj->rxbind_lhead);
                    }
                }
            }
        }
        /*If ring not exists, need do error handling*/
    }  // for loop
    return 0;
error_bind:
    list_for_each_entry_safe(bind_item, is_b, &ch_obj->rxbind_lhead, list)
    {
        list_del(&bind_item->list);
        dcmbus_free_mem((void **) &bind_item);
    }
    list_for_each_entry_safe(bind_item, is_b, &ch_obj->txbind_lhead, list)
    {
        list_del(&bind_item->list);
        dcmbus_free_mem((void **) &bind_item);
    }
    return -1;
}

static int dcmbus_bind_deinit(struct list_head *in_head)
{
    struct dcmbus_bind_entry_t *bind_item = NULL, *is_b = NULL;
    list_for_each_entry_safe(bind_item, is_b, in_head, list)
    {
        list_del(&bind_item->list);
        dcmbus_free_mem((void **) &bind_item);
    }
    return 0;
}

static int dcmbus_channel_create(struct dcmbus_ctrlblk_t *D,
                                 struct channel_config *conf)
{
    struct dcmbus_channel_blk_t *item = NULL;
    item = (struct dcmbus_channel_blk_t *) malloc(sizeof(*item));
    if (!item)
        goto error_malloc;
    memset(item, 0, sizeof(*item));
    item->drv_ops = dcmbus_drivers[conf->driver_idx];
    strcpy(item->ch_name, conf->ch_name);
    strncpy(item->ifname, conf->ifname, 16);
    item->netport = conf->netport;
    item->enable = conf->enable;
    item->type = dcmbus_get_channel_type_enum(conf->type);
    item->blocking = conf->blocking;
    /*Bind operation*/
    dcmbus_bind_ring(D, item);

    list_add_tail(&item->list, &(D->channel_lhead));
    return 0;
error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return -1;
}

static int dcmbus_ring_create(struct dcmbus_ctrlblk_t *D,
                              struct ring_config *conf)
{
    struct dcmbus_ring_blk_t *item = NULL;
    item = (struct dcmbus_ring_blk_t *) malloc(sizeof(*item));
    if (!item)
        goto error_malloc;
    memset(item, 0, sizeof(*item));
    strcpy(item->rg_name, conf->rg_name);
    item->enable = conf->enable;
    item->ring_size = conf->ring_size;
    if (strcmp(conf->direction, "TX") == 0)
        item->dir = DCMBUS_DIR_TX;
    else
        item->dir = DCMBUS_DIR_RX;

    list_add_tail(&item->list, &(D->ring_lhead));
    return 0;
error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return -1;
}

int dcmbus_ctrlblk_init(struct dcmbus_ctrlblk_t *D,
                        const char *path_ring,
                        const char *path_chan,
                        const char *path_bind,
                        int system_type)
{
    int idx, rc = 0;
    struct dcmbus_ring_blk_t *item_r = NULL, *is_r = NULL;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;

    D->system_type = system_type;
    INIT_LIST_HEAD(&D->channel_lhead);
    INIT_LIST_HEAD(&D->ring_lhead);

    D->num_binds = dcmbus_load_bind_conf(path_bind);

    D->num_rings = dcmbus_load_ring_conf(path_ring);
    for (idx = 0; idx < D->num_rings; ++idx) {
        if ((rc = dcmbus_ring_create(D, &g_ring_config[idx])) < 0) {
            fprintf(stderr, "[%s:%d] Ring %d create fail !!\n", __func__,
                    __LINE__, idx);
        }
    }

    list_for_each_entry_safe(item_r, is_r, &D->ring_lhead, list)
    {
        if (item_r->enable) {
            rb_init(&item_r->data_ring, item_r->ring_size);
        }
    }

    D->num_channels = dcmbus_load_channel_conf(path_chan);
    for (idx = 0; idx < D->num_channels; ++idx) {
        if ((rc = dcmbus_channel_create(D, &g_channel_config[idx])) < 0) {
            fprintf(stderr, "[%s:%d] Channel %d create fail !!\n", __func__,
                    __LINE__, idx);
        }
    }

    list_for_each_entry_safe(item, is, &D->channel_lhead, list)
    {
        if (item->enable) {
            item->drv_ops->open_interface(&item->drv_priv_data, item->ifname,
                                          item->netport);
        }
    }

    return 0;
}

int dcmbus_ctrlblk_deinit(struct dcmbus_ctrlblk_t *D)
{
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_ring_blk_t *item_r = NULL, *is_r = NULL;
    list_for_each_entry_safe(item_r, is_r, &D->ring_lhead, list)
    {
        if (item_r->enable) {
            printf("[Ring] %s Closed\n", item_r->rg_name);
            rb_deinit(&item_r->data_ring);
        }
        list_del(&item_r->list);
        free(item_r);
    }

    list_for_each_entry_safe(item, is, &D->channel_lhead, list)
    {
        if (item->enable) {
            printf("[Channel] %s %s Closed\n", item->ch_name, item->ifname);
            item->drv_ops->close_interface(&item->drv_priv_data);
        }
        dcmbus_bind_deinit(&item->txbind_lhead);
        dcmbus_bind_deinit(&item->rxbind_lhead);
        list_del(&item->list);
        free(item);
    }
    return 0;
}

static int dcmbus_l2frame_recv(struct dcmbus_channel_blk_t *item, int buff_size)
{
    struct dcmbus_header_t *rxcell = NULL;
    struct dcmbus_driver_ops *drv_ops = item->drv_ops;
    struct dcmbus_bind_entry_t *bind_item = NULL, *is_b = NULL;
    uint8_t *rx_buffer = NULL;
    int rc = 0;

    rx_buffer = dcmbus_alloc_mem(buff_size);
    if (rx_buffer == NULL) {
        fprintf(stderr, "[%s:%d] dcmbus rx_buffer allocate fail!!\n", __func__,
                __LINE__);
        goto empty;
    }
    if (drv_ops->recv_data(item->drv_priv_data, rx_buffer, buff_size) <= 0)
        goto empty;
    debug_hex_dump("dcmbus_l2frame_recv", (uint8_t *) rx_buffer, buff_size);

    list_for_each_entry_safe(bind_item, is_b, &item->rxbind_lhead, list)
    {
        rxcell = dcmbus_alloc_mem(sizeof(struct dcmbus_header_t));
        if (rxcell == NULL) {
            fprintf(stderr, "[%s:%d] dcmbus rxcell allocate fail!!\n", __func__,
                    __LINE__);
            continue;
        }
        rxcell->frame_full_size = buff_size;
        rxcell->l2frame = dcmbus_alloc_mem(rxcell->frame_full_size);
        if (rxcell->l2frame == NULL) {
            fprintf(stderr, "[%s:%d] dcmbus rxcell l2frame allocate fail!!\n",
                    __func__, __LINE__);
            dcmbus_free_mem((void **) &rxcell);
            continue;
        }
        memcpy(rxcell->l2frame, rx_buffer, rxcell->frame_full_size);
        rb_push(bind_item->p_data_ring, rxcell);
    }
empty:
    dcmbus_free_mem((void **) &rx_buffer);
    return rc;
}

int dcmbus_channel_rx_job(struct dcmbus_ctrlblk_t *D,
                          const char *name,
                          int raw_size)
{
    struct timeval tv;
    int rc = 0;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_driver_ops *drv_ops = NULL;

    tv.tv_sec = 0;
    tv.tv_usec = 100;
    list_for_each_entry_safe(item, is, &D->channel_lhead, list)
    {
        if (item->enable && strcmp(item->ch_name, name) == 0) {
            drv_ops = item->drv_ops;
            switch (item->type) {
            case DCMBUS_SOCKET_CAN:
            case DCMBUS_DEV_RS422:
                drv_ops->fd_zero(item->drv_priv_data);
                drv_ops->fd_set(item->drv_priv_data);
                rc = drv_ops->select(item->drv_priv_data, &tv);
                if (rc < 0)
                    fprintf(stderr, "select error: %d\n", rc);
                if (drv_ops->fd_isset(item->drv_priv_data)) {
                    debug_print("[%lf] RX Received !!\n", get_curr_time());
                    if (dcmbus_l2frame_recv(item, raw_size) < 0)
                        break;
                }
                break;
            case DCMBUS_SOCKET_ETH:
                if (dcmbus_l2frame_recv(item, raw_size) <= 0)
                    break;
                debug_print("[%lf] RX Ethernet Received !!\n", get_curr_time());
                break;
            default:
                fprintf(stderr,
                        "[dcmbus_channel_rx_job] No such RX  device !!!\n");
            }
            break;
        }
    }
    return rc;
}

int dcmbus_channel_tx_job(struct dcmbus_ctrlblk_t *D, const char *ch_name)
{
    struct dcmbus_channel_blk_t *iter = NULL, *is = NULL;
    struct dcmbus_bind_entry_t *bind_iter = NULL, *is_b = NULL;
    struct dcmbus_driver_ops *drv_ops = NULL;
    struct dcmbus_header_t *txcell = NULL;
    uint8_t *tx_buffer = NULL;
    list_for_each_entry_safe(iter, is, &D->channel_lhead, list)
    {
        drv_ops = iter->drv_ops;
        if (iter->enable && strcmp(iter->ch_name, ch_name) == 0) {
            list_for_each_entry_safe(bind_iter, is_b, &iter->txbind_lhead, list)
            {
                rb_pop(bind_iter->p_data_ring, (void **) &txcell);
                if (txcell) {
                    if (iter->drv_priv_data == NULL) {
                        fprintf(stderr,
                                "iter->drv_priv_data: NULL Packet Drop.\n");
                        dcmbus_free_mem(&txcell->l2frame);
                        dcmbus_free_mem((void **) &txcell);
                        continue;
                    }
                    tx_buffer = dcmbus_alloc_mem(txcell->frame_full_size);
                    if (!tx_buffer) {
                        fprintf(stderr,
                                " %s: Tx buffer allocate fail Packet Drop.\n",
                                __func__);
                        dcmbus_free_mem(&txcell->l2frame);
                        dcmbus_free_mem((void **) &txcell);
                        continue;
                    }
                    memcpy(tx_buffer, (uint8_t *) txcell->l2frame,
                           txcell->frame_full_size);
                    int out_frame_size = txcell->frame_full_size;
                    dcmbus_free_mem(&txcell->l2frame);
                    dcmbus_free_mem((void **) &txcell);
                    drv_ops->send_data(iter->drv_priv_data, tx_buffer,
                                       out_frame_size);
                    debug_hex_dump("dcmbus_channel_tx_job", tx_buffer,
                                   out_frame_size);
                    dcmbus_free_mem((void **) &tx_buffer);
                }
            }  // bind ring loop
        }
    }
    return 0;
}

int dcmbus_ring_dequeue(struct dcmbus_ctrlblk_t *D,
                        const char *rg_name,
                        void *payload)
{
    struct dcmbus_ring_blk_t *iter = NULL, *is = NULL;
    struct dcmbus_header_t *rxcell = NULL;
    int size = 0;
    list_for_each_entry_safe(iter, is, &D->ring_lhead, list)
    {
        if (strcmp(iter->rg_name, rg_name) == 0) {
            rb_pop(&iter->data_ring, (void **) &rxcell);
            break;
        }
    }

    if (rxcell == NULL)
        goto empty;
    size = rxcell->frame_full_size;
    memcpy(payload, rxcell->l2frame, size);
    dcmbus_free_mem(&rxcell->l2frame);
    dcmbus_free_mem((void **) &rxcell);
    debug_hex_dump("dcmbus_ring_dequeue", payload, size);
    return size;
empty:
    return 0;
}

int dcmbus_ring_enqueue(struct dcmbus_ctrlblk_t *D,
                        const char *rg_name,
                        void *payload,
                        uint32_t size)
{
    struct dcmbus_ring_blk_t *iter = NULL, *is = NULL;
    struct dcmbus_header_t *txcell = NULL;
    txcell = dcmbus_alloc_mem(sizeof(struct dcmbus_header_t));
    if (txcell == NULL) {
        fprintf(stderr, "[%s:%d] dcmbus txcell allocate fail!!\n", __func__,
                __LINE__);
        goto empty;
    }
    txcell->frame_full_size = size;
    txcell->l2frame = dcmbus_alloc_mem(txcell->frame_full_size);
    if (txcell->l2frame == NULL) {
        fprintf(stderr, "[%s:%d] dcmbus txcell->l2frame allocate fail!!\n",
                __func__, __LINE__);
        goto empty;
    }

    memcpy(txcell->l2frame, (uint8_t *) payload, size);
    list_for_each_entry_safe(iter, is, &D->ring_lhead, list)
    {
        if (strcmp(iter->rg_name, rg_name) == 0) {
            rb_push(&iter->data_ring, txcell);
        }
    }
empty:
    return 0;
}

int dcmbus_tx_direct(struct dcmbus_ctrlblk_t *D,
                     const char *name,
                     void *payload,
                     uint32_t size)
{
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_driver_ops *drv_ops = NULL;

    frame_full_size = size;
    // if (drv_ops->get_header_size) {
    //     frame_full_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
    // }
    list_for_each_entry_safe(item, is, &D->channel_lhead, list)
    {
        if (item->enable && strcmp(item->ch_name, name) == 0) {
            drv_ops = item->drv_ops;
            tx_buffer = dcmbus_alloc_mem(frame_full_size);
            memcpy(tx_buffer + offset, (uint8_t *) payload, size);
            drv_ops->send_data(item->drv_priv_data, tx_buffer, frame_full_size);
            debug_hex_dump("dcmbus_tx_direct", tx_buffer, frame_full_size);
            dcmbus_free_mem((void **) &tx_buffer);
            break;
        }
    }

    // if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
    //     drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload,
    //     size); offset += drv_ops->header_copy(ctrlport->drv_priv_data,
    //     tx_buffer);
    // }

    return 0;
}
