#include "ethernet.h"
#include <net/if.h>
static int ethernet_tcp_socket_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    int optval = 1; /* prevent from address being taken */
    if ((dev_info->server_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        fprintf(stderr, "[%s:%d] Socket create fail. status: %s\n", __func__, __LINE__, strerror(errno));
    }
    memset(&dev_info->server_addr, 0, sizeof(dev_info->server_addr));
    dev_info->server_addr.sin_family = AF_INET;
    dev_info->server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->server_addr.sin_port = htons(net_port);
    err = setsockopt(dev_info->server_fd, SOL_SOCKET,  SO_REUSEADDR, &optval, sizeof(int));
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("ethernet_tcp_socket_server: setsockopt() failed");
    }

    err = bind(dev_info->server_fd, (struct sockaddr *)&dev_info->server_addr, sizeof(dev_info->server_addr));
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("ethernet_tcp_socket_server: bind() failed");
    }

    err = listen(dev_info->server_fd, 5);
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("ethernet_tcp_socket_server: listen() failed");
    }

    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    fprintf(stderr, "ethernet_tcp_socket_server: Waiting for the client ...\n");
    dev_info->client_fd = accept(dev_info->server_fd, (struct sockaddr*) &(dev_info->client_addr), (socklen_t*)&dev_info->client_addr_len);
    if (dev_info->client_fd < 0) {
        fprintf(stderr, "ethernet_tcp_socket_server: Accept Fail ... %s:%d\n", ifname, net_port);
        goto error;
    }
    fprintf(stderr, "ethernet_tcp_socket_server: Accept on ... %s:%d\n", ifname, net_port);
    return 0;
error:
    return -1;
}

static int ethernet_udp_socket_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    if ((dev_info->server_fd = socket(AF_INET , SOCK_DGRAM , 0)) < 0) {
        errExit("[ethernet_udp_socket_server] Error while opening socket");
    }
    memset(&dev_info->server_addr, 0, sizeof(dev_info->server_addr));
    dev_info->server_addr.sin_family = AF_INET;
    dev_info->server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->server_addr.sin_port = htons(net_port);

    err = bind(dev_info->server_fd, (struct sockaddr *)&dev_info->server_addr, sizeof(dev_info->server_addr));
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[ethernet_udp_socket_server] bind() failed");
    }
    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    fprintf(stderr, "ethernet_udp_socket_server: Accept on ... %s:%d\n", ifname, net_port);
    return 0;
}


static int ethernet_create_server(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 1;
    switch (dev_info->sock_type) {
        case SOCK_DGRAM:
            err = ethernet_udp_socket_server(dev_info, ifname, net_port);
            break;
        case SOCK_STREAM:
            err = ethernet_tcp_socket_server(dev_info, ifname, net_port);
            break;
        default:
            fprintf(stderr, "[%s:%d] Unknown socket type: %d\n", __func__, __LINE__, dev_info->sock_type);
            err = -1;
    }
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[ethernet_create_server] Failed then Abort !!");
    }
    return 0;
}

static int ethernet_tcp_socket_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    int retry_cnt = 0;
    if ((dev_info->client_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->client_addr.sin_family = AF_INET;
    dev_info->client_addr.sin_addr.s_addr = inet_addr(ifname);
    dev_info->client_addr.sin_port = htons(net_port);
    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    while (retry_cnt < 5) {
        err = connect(dev_info->client_fd, (struct sockaddr *)&dev_info->client_addr, sizeof(dev_info->client_addr));
        if (err == 0)
            break;
        if (err < 0) {
            sleep(3);
            retry_cnt++;
            fprintf(stderr, "ethernet_tcp_socket_client: Connection error retry...%d\n", retry_cnt);
        }
    }
    if (err < 0) {
        fprintf(stderr, "ethernet_tcp_socket_client: Connection error !!\n");
        return -1;
    }
    fprintf(stderr, "ethernet_tcp_socket_client: Connection %s:%d\n", ifname, net_port);
    return 0;
}

static int ethernet_udp_socket_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err = 0;
    if ((dev_info->client_fd = socket(AF_INET , SOCK_DGRAM , 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->client_addr.sin_family = AF_INET;
    dev_info->client_addr.sin_port = htons(net_port);
    if (inet_pton(AF_INET, ifname, &dev_info->client_addr.sin_addr.s_addr) == 0) {
        fprintf(stderr, "Invalid IP adress\n");
        err = -1;
    }

    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    if (err < 0) {
        fprintf(stderr, "ethernet_udp_socket_client: UDP client not Ready !!\n");
        return -1;
    }
    fprintf(stderr, "ethernet_udp_socket_client: UDP client Ready !! %s:%d\n", ifname, net_port);
    return 0;
}

static int ethernet_create_client(struct ethernet_device_info_t *dev_info, char *ifname, int net_port) {
    int err;
    dev_info->server_enable = 0;
    switch (dev_info->sock_type) {
        case SOCK_DGRAM:
            err = ethernet_udp_socket_client(dev_info, ifname, net_port);
            break;
        case SOCK_STREAM:
            err = ethernet_tcp_socket_client(dev_info, ifname, net_port);
            break;
        default:
            fprintf(stderr, "[%s:%d] Unknown Socket type: %d\n", __func__, __LINE__, dev_info->sock_type);
            err = -1;
    }
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[ethernet_create_client] Failed then Abort !!");
    }
    return 0;
}

int ethernet_tcp_init(void **priv_data, char *ifname, int netport) {
    struct ethernet_device_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct ethernet_device_info_t));

    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __func__, __LINE__, strerror(errno));
        goto error;
    }
     /* TCP Blocking Socket*/
    dev_info->sock_type = SOCK_STREAM;
    strncpy(dev_info->ifname, ifname, IF_NAMESIZE);
    if (strstr(ifname, "_server")) {
        if (ethernet_create_server(dev_info, ifname, netport) < 0)
            errExit("ethernet_tcp_init :Error create server");
    } else {
        if (ethernet_create_client(dev_info, ifname, netport) < 0)
            errExit("ethernet_tcp_init :Error create client");
    }
    dev_info->header_size = 0;
    *priv_data = dev_info;
    return 0;
error:
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return -1;
}

int ethernet_deinit(void **priv_data) {
    struct ethernet_device_info_t *dev_info = *priv_data;
    if (dev_info->client_fd > 0)
        close(dev_info->client_fd);
    if (dev_info->server_fd > 0)
        close(dev_info->server_fd);
    fprintf(stderr, "Closing %s \n", dev_info->ifname);
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return 0;
}

int ethernet_udp_init(void **priv_data, char *ifname, int netport) {
    struct ethernet_device_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct ethernet_device_info_t));

    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __func__, __LINE__, strerror(errno));
        goto error;
    }
     /* TCP Blocking Socket*/
    dev_info->sock_type = SOCK_DGRAM;
    strncpy(dev_info->ifname, ifname, IF_NAMESIZE);
    if (strstr(ifname, "_server")) {
        if (ethernet_create_server(dev_info, ifname, netport) < 0)
            errExit("ethernet_udp_init :Error create server");
    } else {
        if (ethernet_create_client(dev_info, ifname, netport) < 0)
            errExit("ethernet_udp_init :Error create client");
    }
    dev_info->header_size = 0;
    *priv_data = dev_info;
    return 0;
error:
    if (dev_info) {
        free(dev_info);
        dev_info = NULL;
    }
    *priv_data = NULL;
    return -1;
}

int ethernet_data_recv(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;

    while (offset < buff_size) {
        if ((rdlen = recv(dev_info->client_fd, rx_buff + offset, buff_size - offset, 0)) < 0) {
            if (errno == EINTR) {
                rdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __func__, strerror(errno));
                return -1;
            }
        } else if (rdlen == 0) {
            break;
        }
        offset += rdlen;
    }
    return offset;
}

int ethernet_data_send(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    while (offset < frame_len) {
        if ((wdlen = send(dev_info->client_fd, payload + offset, frame_len - offset, 0)) < 0) {
            if (wdlen < 0 && errno == EINTR) {
                wdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __func__, strerror(errno));
                return -1;
            }
        }
        offset += wdlen;
    }
    return offset;
}

int ethernet_udp_data_recvfrom(void *priv_data, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    int source_fd;
    source_fd = dev_info->server_enable ? dev_info->server_fd : dev_info->client_fd;

    while (offset < buff_size) {
        if ((rdlen = recvfrom(source_fd, rx_buff + offset, buff_size - offset, 0,
                              (struct sockaddr*)&dev_info->client_addr, (socklen_t *)&dev_info->client_addr_len)) < 0) {
            if (errno == EINTR) {
                rdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __func__, strerror(errno));
                return -1;
            }
        } else if (rdlen == 0) {
            break;
        }
        offset += rdlen;
    }
    return offset;
}

int ethernet_udp_data_sendto(void *priv_data, uint8_t *payload, uint32_t frame_len) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    struct ethernet_device_info_t *dev_info = priv_data;
    int source_fd;
    source_fd = dev_info->server_enable ? dev_info->server_fd : dev_info->client_fd;
    while (offset < frame_len) {
        if ((wdlen = sendto(source_fd, payload + offset, frame_len - offset, 0,
                            (struct sockaddr*)&dev_info->client_addr, sizeof(dev_info->client_addr))) < 0) {
            if (wdlen < 0 && errno == EINTR) {
                wdlen = 0;
            } else {
                fprintf(stderr, "%s: %s\n", __func__, strerror(errno));
                return -1;
            }
        }
        offset += wdlen;
    }
    return offset;
}


uint32_t ethernet_get_header_size(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return dev_info->header_size;
}

int ethernet_is_server(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return dev_info->server_enable;
}
int ethernet_get_client_fd(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    return dev_info->client_fd;
}

int ethernet_accept(void *priv_data) {
    struct ethernet_device_info_t *dev_info = priv_data;
    dev_info->client_fd = accept(dev_info->server_fd, (struct sockaddr*) &(dev_info->client_addr), (socklen_t *)&(dev_info->client_addr_len));
    return dev_info->client_fd;
}

struct dcmbus_driver_ops dcmbus_driver_ethernet_ops = {
    .open_interface = ethernet_tcp_init,
    .recv_data = ethernet_data_recv,
    .send_data = ethernet_data_send,

    .header_set = NULL,
    .header_copy = NULL,
    .get_header_size = ethernet_get_header_size,
    .is_server = ethernet_is_server,
    .get_client_fd = ethernet_get_client_fd,
    .accept = ethernet_accept,
    .close_interface = ethernet_deinit,
};

struct dcmbus_driver_ops dcmbus_driver_ethernet_udp_ops = {
    .open_interface = ethernet_udp_init,
    .recv_data = ethernet_udp_data_recvfrom,
    .send_data = ethernet_udp_data_sendto,

    .header_set = NULL,
    .header_copy = NULL,
    .get_header_size = ethernet_get_header_size,
    .is_server = ethernet_is_server,
    .get_client_fd = ethernet_get_client_fd,
    .accept = ethernet_accept,
    .close_interface = ethernet_deinit,
};
