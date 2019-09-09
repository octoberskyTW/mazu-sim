#ifndef __NON_BLOCKING_IO_H__
#define __NON_BLOCKING_IO_H__
#include "dcmbus_driver_intf.h"
#include "dcmbus_util.h"
#include "linux_common.h"

struct non_blocking_io_info_t {
    char ifname[IF_NAMESIZE];
    int server_enable;
    int server_fd;
    int client_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int client_addr_len;
    int sock_type;
    struct timeval timeout;
    fd_set readfd_set;
    fd_set writefd_set;
};

#endif  // __NON_BLOCKING_IO_H__