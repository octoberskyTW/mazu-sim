#include "ringbuffer.h"
#include "dcmbus.h"

int dcmbus_load_cfg (void) {
    struct ringbuffer_t rb;
    rb_init(&rb, 128);
    rb_deinit(&rb);
    printf("[dutsai] %s\n", __FUNCTION__);
    return 0;
}


int dcmbus_ctrlblk_init(struct dcmbus_ctrlblk_t* D, int system_type) {

    struct dcmbus_channel_t *dchannel;
    struct dcmbus_driver_ops *drv_ops;
    int idx;

    D->system_type = system_type;

    // for (idx = 0; idx < get_arr_num(port_tbl_size, sizeof(struct dcmbus_channel_t)); idx++) {
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