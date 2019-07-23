#include "dcmbus.h"
#include "dcmbus_util.h"

static struct channel_config g_channel_config[8];
static struct ring_config g_ring_config[16];
static struct channel_type_enum_t chan_type_enum_tbl[]= {
    {DCMBUS_SOCKET_ETH, "socket_eth"},
    {DCMBUS_SOCKET_CAN, "socket_can"},
    {DCMBUS_DEV_RS422,  "dev_rs422"}
};

static int dcmbus_load_ring_conf(const char *path) {
    int numEntries,idx;
    char specifier[] = RING_SPECIFIER;
    memset(g_ring_config, 0, sizeof(g_ring_config));
    read_config(g_ring_config, &numEntries, path, specifier);
    printf("Load ring Entries: %d \n", numEntries);
    printf("%16s %16s %16s %16s\n", RING_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, RING_PRINTF_FORMAT,
                        g_ring_config[idx].name,
                        g_ring_config[idx].enable,
                        g_ring_config[idx].direction,
                        g_ring_config[idx].ring_size);
    }
    return numEntries;
}

static int dcmbus_load_channel_conf(const char *path) {
    int numEntries,idx;
    char specifier[] = CHANNEL_SPECIFIER;
    memset(g_channel_config, 0, sizeof(g_channel_config));
    read_config(g_channel_config, &numEntries, path, specifier);
    printf("Load channel Entries: %d \n", numEntries);
    printf("%16s %16s %16s %16s %16s %16s %16s %16s %16s\n", CHANNEL_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, CHANNEL_PRINTF_FORMAT,
                        g_channel_config[idx].name,
                        g_channel_config[idx].enable,
                        g_channel_config[idx].direction,
                        g_channel_config[idx].type,
                        g_channel_config[idx].ifname,
                        g_channel_config[idx].netport,
                        g_channel_config[idx].driver_idx,
                        g_channel_config[idx].blocking,
                        g_channel_config[idx].options);
    }
    return numEntries;
}



static int dcmbus_get_channel_type_enum(const char *type_name) {
    int rc = 0;
    int idx;
    for(idx = 0; idx < get_arr_num(sizeof(chan_type_enum_tbl), sizeof(struct channel_type_enum_t)); idx++) {
        if (strcmp(chan_type_enum_tbl[idx].type_name, type_name) == 0) {
            rc = chan_type_enum_tbl[idx].enumval;
        }
    }
    return rc;
}

static int dcmbus_ring_create(struct dcmbus_ctrlblk_t* D, struct ring_config* conf) {
    struct dcmbus_ring_blk_t *item = NULL;
    item = (struct dcmbus_ring_blk_t *) malloc(sizeof(*item));
    if(!item)
        goto error_malloc;
    memset(item, 0, sizeof(*item));
    strncpy(item->name, conf->name, 16);
    item->enable = conf->enable;
    item->ring_size = conf->ring_size;
    list_add_tail(&item->list, &(D->ring_lhead));
    return 0;
error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return -1;
}

static int dcmbus_channel_create(struct dcmbus_ctrlblk_t* D, struct channel_config* conf) {
    struct dcmbus_channel_blk_t *item = NULL;
    item = (struct dcmbus_channel_blk_t *) malloc(sizeof(*item));
    if(!item)
        goto error_malloc;
    memset(item, 0, sizeof(*item));
    item->drv_ops = dcmbus_drivers[conf->driver_idx];
    strncpy(item->name, conf->name, 16);
    strncpy(item->ifname, conf->ifname, 16);
    item->netport = conf->netport;
    item->enable = conf->enable;
    item->type = dcmbus_get_channel_type_enum(conf->type);
    item->blocking = conf->blocking;
    list_add_tail(&item->list, &(D->channel_lhead));
    return 0;
error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return -1;
}

int dcmbus_ctrlblk_init(struct dcmbus_ctrlblk_t* D,
                        const char *path_ring,
                        const char *path_chan,
                        int system_type) {

    int num_rings;
    int num_channels;
    int idx, rc = 0;
    struct dcmbus_ring_blk_t *item_r = NULL, *is_r = NULL;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    
    D->system_type = system_type;
    INIT_LIST_HEAD(&D->channel_lhead);
    INIT_LIST_HEAD(&D->ring_lhead);
    
    num_rings = dcmbus_load_ring_conf(path_ring);
    for (idx = 0; idx < num_rings; ++idx) {
        if ((rc = dcmbus_ring_create(D, &g_ring_config[idx])) < 0) {
            fprintf(stderr, "[%s:%d] Ring %d create fail !!\n", __func__, __LINE__, idx);
        }
    }

    list_for_each_entry_safe(item_r, is_r, &D->ring_lhead, list) {
        if (item_r->enable) {
            rb_init(&item_r->data_ring, item_r->ring_size);
        }
    }


    num_channels = dcmbus_load_channel_conf(path_chan);
    for (idx = 0; idx < num_channels; ++idx) {
        if ((rc = dcmbus_channel_create(D, &g_channel_config[idx])) < 0) {
            fprintf(stderr, "[%s:%d] Channel %d create fail !!\n", __func__, __LINE__, idx);
        }
    }

    list_for_each_entry_safe(item, is, &D->channel_lhead, list) {
        if (item->enable) {
            item->drv_ops->open_interface(&item->drv_priv_data, item->ifname, item->netport);
        }
    }
    return 0;
}

int dcmbus_ctrlblk_deinit(struct dcmbus_ctrlblk_t* D) {
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_ring_blk_t *item_r = NULL, *is_r = NULL;
    list_for_each_entry_safe(item_r, is_r, &D->ring_lhead, list) {
        if (item->enable) {
            printf("[Ring] %s Closed\n", item->name);
            rb_deinit(&item_r->data_ring);
        }
        list_del(&item->list);
        free(item);
    }

    list_for_each_entry_safe(item, is, &D->channel_lhead, list) {
        if (item->enable) {
            printf("[Channel] %s %s Closed\n", item->name, item->ifname);
            item->drv_ops->close_interface(&item->drv_priv_data);
        }
        list_del(&item->list);
        free(item);
    }
    return 0;
}

static void *dcmbus_alloc_mem(size_t size) {
    return calloc(1, size);
}

static void dcmbus_free_mem(void **ptr) {
    if (*ptr) {
        free(*ptr);
        *ptr = NULL;
    }
    return;
}

static int dcmbus_l2frame_recv(struct dcmbus_channel_blk_t *item, int buff_size) {
    struct dcmbus_header_t *rxcell = NULL;
    struct dcmbus_driver_ops *drv_ops = item->drv_ops;
    int rc = 0;

    rxcell = dcmbus_alloc_mem(sizeof(struct dcmbus_header_t));
    if (rxcell == NULL) {
        fprintf(stderr, "[%s:%d] dcmbus cell allocate fail!!\n", __func__, __LINE__);
        goto empty;
    }
    rxcell->frame_full_size = buff_size;
    rxcell->l2frame = dcmbus_alloc_mem(rxcell->frame_full_size);
    if (rxcell->l2frame == NULL) {
        fprintf(stderr, "[%s:%d] dcmbus l2frame allocate fail!!\n", __func__, __LINE__);
        goto empty;
    }
    if (drv_ops->recv_data(item->drv_priv_data, (uint8_t *)rxcell->l2frame, rxcell->frame_full_size) < 0)
        goto empty;
    debug_hex_dump("dcmbus_l2frame_recv", (uint8_t *)rxcell->l2frame, rxcell->frame_full_size);
    // qidx = icf_dispatch_rx_frame(C->system_type, rxcell->l2frame, ctrlport->hw_port_idx);
    // if (qidx == EGSE_EMPTY_SW_QIDX)
    //     goto empty;
    // ctrlqueue = C->ctrlqueue[qidx];
    // rb_push(&ctrlqueue->data_ring, rxcell);
    return rc;
empty:
    dcmbus_free_mem(&rxcell->l2frame);
    dcmbus_free_mem((void **)&rxcell);
    return rc;
}


int dcmbus_channel_rx_job(struct dcmbus_ctrlblk_t* D, const char *name, int raw_size) {
    struct timeval tv;
    int rc = 0;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_driver_ops *drv_ops = NULL;

    tv.tv_sec = 0;
    tv.tv_usec = 100;
    list_for_each_entry_safe(item, is, &D->channel_lhead, list) {
        if (item->enable && strcmp(item->name, name) == 0) {
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
                        if (dcmbus_l2frame_recv(item, raw_size) < 0)
                            break;
                        debug_print("[%lf] RX Received !!\n", get_curr_time());
                    }
                    break;
                case DCMBUS_SOCKET_ETH:
                        if (dcmbus_l2frame_recv(item, raw_size) < 0)
                            break;
                        debug_print("[%lf] RX Ethernet Received !!\n", get_curr_time());
                    break;
                default:
                    fprintf(stderr, "[dcmbus_channel_rx_job] No such RX  device !!!\n");
            }
            break;
        }
        
    }

    return rc;
}

#if 0
int dcmbus_channel_tx_job(struct dcmbus_ctrlblk_t* D, const char *name, const char *name) {
    // int rc = 0;
    // uint8_t *tx_buffer = NULL;
    // struct ringbuffer_cell_t *txcell = NULL;
    // struct ringbuffer_t *whichring = NULL;
    // struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
    // struct icf_ctrl_port *ctrlport = C->ctrlqueue[qidx]->port;
    // struct icf_driver_ops *drv_ops = ctrlport->drv_priv_ops;
    // uint32_t out_frame_size;
    // uint32_t offset = 0;
    // whichring = &ctrlqueue->data_ring;
    // txcell = (uint8_t *)rb_pop(whichring);
    // if (txcell) {
    //     out_frame_size = txcell->frame_full_size;
    //     if (ctrlport->drv_priv_data == NULL) {
    //         icf_free_mem(&txcell->l2frame);
    //         icf_free_mem(&txcell);
    //         return ICF_STATUS_SUCCESS;
    //     }
    //     // if (drv_ops->get_header_size) {
    //     //     out_frame_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
    //     // }
    //     tx_buffer = icf_alloc_mem(out_frame_size);

    //     // if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
    //     //     drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) txcell->l2frame, txcell->frame_full_size);
    //     //     offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
    //     // }
    //     memcpy(tx_buffer + offset, (uint8_t *) txcell->l2frame, txcell->frame_full_size);
    //     icf_free_mem(&txcell->l2frame);
    //     icf_free_mem(&txcell);
    //     drv_ops->send_data(ctrlport->drv_priv_data, tx_buffer, out_frame_size);
    //     debug_hex_dump("icf_tx_ctrl_job", tx_buffer, out_frame_size);
    //     icf_free_mem(&tx_buffer);
    // }
    // return ICF_STATUS_SUCCESS;
    return 0;
}

int dcmbus_ring_dequeue(struct dcmbus_ctrlblk_t* D, const char *name, void *payload, uint32_t size) {
//     struct icf_ctrl_queue *ctrlqueue = C->ctrlqueue[qidx];
//     struct ringbuffer_cell_t *rxcell = NULL;
//     rxcell = (struct ringbuffer_cell_t *)rb_pop(&ctrlqueue->data_ring);
//     if (rxcell == NULL)
//         goto empty;
//     memcpy(payload, rxcell->l2frame, size);
//     icf_free_mem(&rxcell->l2frame);
//     icf_free_mem(&rxcell);
//     debug_hex_dump("icf_rx_dequeue", payload, size);
//     return 1;
// empty:
     return 0;
}
#endif

int dcmbus_tx_direct(struct dcmbus_ctrlblk_t* D, const char *name, void *payload, uint32_t size) {
    uint8_t *tx_buffer = NULL;
    uint32_t frame_full_size;
    uint32_t offset = 0;
    struct dcmbus_channel_blk_t *item = NULL, *is = NULL;
    struct dcmbus_driver_ops *drv_ops = NULL;

    frame_full_size = size;
    // if (drv_ops->get_header_size) {
    //     frame_full_size += drv_ops->get_header_size(ctrlport->drv_priv_data);
    // }
    list_for_each_entry_safe(item, is, &D->channel_lhead, list) {
        if (item->enable && strcmp(item->name, name) == 0) {
            drv_ops = item->drv_ops;
            tx_buffer = dcmbus_alloc_mem(frame_full_size);
            memcpy(tx_buffer + offset, (uint8_t *) payload, size);
            drv_ops->send_data(item->drv_priv_data, tx_buffer, frame_full_size);
            debug_hex_dump("icf_tx_direct", tx_buffer, frame_full_size);
            dcmbus_free_mem((void **)&tx_buffer);
            break;
        }
        
    }

    // if (drv_ops->get_header_size(ctrlport->drv_priv_data)) {
    //     drv_ops->header_set(ctrlport->drv_priv_data, (uint8_t *) payload, size);
    //     offset += drv_ops->header_copy(ctrlport->drv_priv_data, tx_buffer);
    // }

    return 0;
}

