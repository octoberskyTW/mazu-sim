#ifndef __DCMBUS_H__ 
#define __DCMBUS_H__

#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "dcmbus_driver.h"

typedef enum _ENUM_DCMBUS_CHANNEL_TYPE {
    DCMBUS_TCP_SERVER = 0x1,
    DCMBUS_TCP_CLIENT = 0x2,
    DCMBUS_UDP_SERVER = 0x3,
    DCMBUS_UDP_CLIENT = 0x4,
    DCMBUS_SOCKET_CAN = 0x5,
    DCMBUS_CHARDEV_RS422 = 0x6,
    DCMBUS_NULL_CHANNEL_TYPE
}ENUM_DCMBUS_CHANNEL_TYPE;

struct dcmbus_channel_t {
    int fd;
    int channel_type;
    char mode[16];
    char channel_name[16];
    void *drv_priv_data;
    struct dcmbus_driver_ops *drv_ops;
};

struct dcmbus_ctrlblk_t {
    int system_type;
    struct dcmbus_channel_t *dcmbus_chan[16];
};

#ifdef __cplusplus
extern "C" {
#endif
    int dcmbus_load_cfg(void);
#ifdef __cplusplus
}
#endif




#endif  //  __DCMBUS_H__