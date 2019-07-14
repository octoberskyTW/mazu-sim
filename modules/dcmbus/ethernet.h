#ifndef __ETHERNET_H__
#define __ETHERNET_H__
#include "dcmbus_driver_intf.h"
#include "dcmbus_util.h"
#include "linux_common.h"

struct ethernet_device_info_t {
    char ifname[IF_NAMESIZE];
    int server_enable;
    int server_fd;
    int client_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int client_addr_len;
    uint32_t header_size;
    int sock_type;
};

#endif  // __ETHERNET_H__