#include "dcmbus.h"

int dcmbus_load_cfg (void) {
    struct ringbuffer_t rb;
    rb_init(&rb, 128);
    rb_deinit(&rb);
    printf("[dutsai] %s\n", __func__);
    return 0;
}


static int dcmbus_channel_create(struct dcmbus_ctrlblk_t* D, int driver_idx) {
    struct dcmbus_channel_t *item = NULL;
    item = (struct dcmbus_channel_t *) malloc(sizeof(*item));
    if(!item)
        goto error_malloc;
    item->drv_ops = dcmbus_drivers[driver_idx];
    list_add_tail(&item->list, &(D->channel_lhead));

error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return 0;
}

int dcmbus_ctrlblk_init(struct dcmbus_ctrlblk_t* D, int system_type) {

    D->system_type = system_type;
    INIT_LIST_HEAD(&(D->channel_lhead));


    dcmbus_channel_create(D, DCMBUS_DRIVER_TCP);
    // for (idx = 0; idx < get_arr_num(port_tbl_size, sizeof(struct icf_ctrl_port)); idx++) {
    //     ctrlport = &which_port_tbl[idx];
    //     C->ctrlport[ctrlport->hw_port_idx] = ctrlport;
    //     ctrlport->drv_priv_ops = icf_drivers[icf_pidx_to_drivers_id(ctrlport->hw_port_idx, system_type)];
    //     drv_ops = ctrlport->drv_priv_ops;
    //     if (ctrlport->enable == 0)
    //         continue;
    //     drv_ops->open_interface(&ctrlport->drv_priv_data, ctrlport->ifname, ctrlport->netport);
    // }
    return 0;
}