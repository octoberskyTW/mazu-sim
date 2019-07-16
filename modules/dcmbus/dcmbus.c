#include "dcmbus.h"

static struct channel_config g_channel_config[8];

static int dcmbus_load_channel_conf(const char *path) {
    int numEntries,idx;
    char specifier[] = CHANNEL_SPECIFIER;
    memset(g_channel_config, 0, sizeof(g_channel_config));
    read_config(g_channel_config, &numEntries, path, specifier);
    printf("[%s] Entries: %d \n", __func__, numEntries);
    printf("%16s %16s %16s %16s %16s %16s %16s %16s %16s\n", CHANNEL_FIELDS_NAME);
    for (idx = 0; idx < numEntries; ++idx) {
        fprintf(stdout, CHANNEL_PRINTF_FORMAT,
                        g_channel_config[idx].ch_name,
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


static int dcmbus_channel_create(struct dcmbus_ctrlblk_t* D, struct channel_config* conf) {
    struct dcmbus_channel_blk_t *item = NULL;
    item = (struct dcmbus_channel_blk_t *) malloc(sizeof(*item));
    if(!item)
        goto error_malloc;
    item->drv_ops = dcmbus_drivers[conf->driver_idx];
    strncpy(item->ch_name, conf->ch_name, 16);
    strncpy(item->ifname, conf->ifname, 16);
    item->netport = conf->netport;
    item->enable = conf->enable;
    list_add_tail(&item->list, &(D->channel_lhead));
    return 0;
error_malloc:
    fprintf(stderr, "[%s:%d] Allocate Fail\n", __func__, __LINE__);
    return -1;
}

int dcmbus_ctrlblk_init(struct dcmbus_ctrlblk_t* D, const char *path, int system_type) {

    int num_channels;
    int idx, rc = 0;
    D->system_type = system_type;
    INIT_LIST_HEAD(&(D->channel_lhead));

    num_channels = dcmbus_load_channel_conf(path);
    for (idx = 0; idx < num_channels; ++idx) {
        if ((rc = dcmbus_channel_create(D, &g_channel_config[idx])) < 0) {
            fprintf(stderr, "[%s:%d] Channel %d create fail !!\n", __func__, __LINE__, idx);
        }
    }
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