#include "non_blocking_io.h"

static int set_sock_non_block(int fd)
{
    int flags, rc = 0;
    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        fprintf(stderr, "fcntl(F_GETFL) failed");
        rc = -1;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "fcntl(F_SETFL) failed");
        rc = -1;
    }
    return rc;
}

static int non_blocking_tcp_sock_server(struct non_blocking_io_info_t *dev_info,
                                        char *ifname,
                                        int net_port)
{
    int err;
    int optval = 1; /* prevent from address being taken */
    if ((dev_info->server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        fprintf(stderr, "[%s:%d] Socket create fail. status: %s\n", __func__,
                __LINE__, strerror(errno));
        errExit("non_blocking_tcp_sock_server: socket() failed");
    }
    memset(&dev_info->server_addr, 0, sizeof(dev_info->server_addr));
    dev_info->server_addr.sin_family = AF_INET;
    dev_info->server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dev_info->server_addr.sin_port = htons(net_port);
    err = setsockopt(dev_info->server_fd, SOL_SOCKET, SO_REUSEADDR, &optval,
                     sizeof(int));

    if (err < 0) {
        fprintf(stderr, "[%s:%d] setsockopt() failed. status: %s\n", __func__,
                __LINE__, strerror(errno));
        goto error;
    }

    err = set_sock_non_block(dev_info->server_fd);
    if (err < 0) {
        fprintf(stderr, "[%s:%d] set_sock_non_block failed. status: %s\n",
                __func__, __LINE__, strerror(errno));
        goto error;
    }

    err = bind(dev_info->server_fd, (struct sockaddr *) &dev_info->server_addr,
               sizeof(dev_info->server_addr));
    if (err < 0) {
        fprintf(stderr, "[%s:%d] bind() failed. status: %s\n", __func__,
                __LINE__, strerror(errno));
        goto error;
    }

    err = listen(dev_info->server_fd, 32);
    if (err < 0) {
        fprintf(stderr, "[%s:%d] listen() failed. status: %s\n", __func__,
                __LINE__, strerror(errno));
        goto error;
    }
    fprintf(stderr, "Listen on ... %s:%d\n", ifname, net_port);
    FD_ZERO(&dev_info->master_set);
    dev_info->max_sd = dev_info->server_fd;
    FD_SET(dev_info->server_fd, &dev_info->master_set);
    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    dev_info->timeout.tv_sec = 0;
    dev_info->timeout.tv_usec = 0;
    dev_info->end_server = FALSE;

    return 0;
error:
    return -1;
}


static int non_blocking_server_create(struct non_blocking_io_info_t *dev_info,
                                      char *ifname,
                                      int net_port)
{
    int err;
    dev_info->server_enable = 1;
    switch (dev_info->sock_type) {
    case SOCK_DGRAM:
        err = 0;
        break;
    case SOCK_STREAM:
        err = non_blocking_tcp_sock_server(dev_info, ifname, net_port);
        break;
    default:
        fprintf(stderr, "[%s:%d] Unknown socket type: %d\n", __func__, __LINE__,
                dev_info->sock_type);
        err = -1;
    }
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[non_blocking_server_create] Failed then Abort !!");
    }
    return 0;
}

static int non_blocking_tcp_sock_client(struct non_blocking_io_info_t *dev_info,
                                        char *ifname,
                                        int net_port)
{
    int err;
    int retry_cnt = 0;
    if ((dev_info->client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        errExit("Error while opening socket");
        return -1;
    }
    dev_info->client_addr.sin_family = AF_INET;
    dev_info->client_addr.sin_addr.s_addr = inet_addr(ifname);
    dev_info->client_addr.sin_port = htons(net_port);
    dev_info->client_addr_len = sizeof(dev_info->client_addr);
    while (retry_cnt < 5) {
        err = connect(dev_info->client_fd,
                      (struct sockaddr *) &dev_info->client_addr,
                      sizeof(dev_info->client_addr));
        if (err == 0)
            break;
        if (err < 0) {
            sleep(3);
            retry_cnt++;
            fprintf(stderr, "%s: Connection error retry...%d\n", __func__,
                    retry_cnt);
        }
    }
    if (err < 0) {
        fprintf(stderr, "%s: Connection error !!\n", __func__);
        return -1;
    }
    fprintf(stderr, "%s: Connection %s:%d\n", ifname, __func__, net_port);
    return 0;
}


static int non_blocking_client_create(struct non_blocking_io_info_t *dev_info,
                                      char *ifname,
                                      int net_port)
{
    int err;
    dev_info->server_enable = 0;
    switch (dev_info->sock_type) {
    case SOCK_DGRAM:
        err = 0;
        break;
    case SOCK_STREAM:
        err = non_blocking_tcp_sock_client(dev_info, ifname, net_port);
        break;
    default:
        fprintf(stderr, "[%s:%d] Unknown Socket type: %d\n", __func__, __LINE__,
                dev_info->sock_type);
        err = -1;
    }
    if (err < 0) {
        close(dev_info->server_fd);
        errExit("[non_blocking_client_create] Failed then Abort !!");
    }
    return 0;
}

int non_blocking_tcp_init(void **priv_data, char *ifname, int netport)
{
    struct non_blocking_io_info_t *dev_info = NULL;
    dev_info = malloc(sizeof(struct non_blocking_io_info_t));

    if (dev_info == NULL) {
        fprintf(stderr, "[%s:%d]Memory allocate fail. status: %s\n", __func__,
                __LINE__, strerror(errno));
        goto error;
    }
    /* TCP Blocking Socket*/
    dev_info->sock_type = SOCK_STREAM;
    strncpy(dev_info->ifname, ifname, IF_NAMESIZE);
    if (strstr(ifname, "_server")) {
        if (non_blocking_server_create(dev_info, ifname, netport) < 0)
            errExit("non_blocking_server_create :Error create server");
    } else {
        if (non_blocking_client_create(dev_info, ifname, netport) < 0)
            errExit("non_blocking_client_create :Error create client");
    }
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

int non_blocking_deinit(void **priv_data)
{
    struct non_blocking_io_info_t *dev_info = *priv_data;
    int ii = 0;
    for (ii = 0; ii <= dev_info->max_sd; ++ii) {
        if (FD_ISSET(ii, &dev_info->master_set))
            close(ii);
    }
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


int non_blocking_data_recv(void *priv_data,
                           uint8_t *rx_buff,
                           uint32_t buff_size)
{
    uint32_t offset = 0;
    int32_t rdlen = 0;
    int rc = 0, ii;
    struct non_blocking_io_info_t *dev_info = priv_data;
    if (dev_info->end_server) {
        return rc;
    }
    memcpy(&dev_info->rx_working_set, &dev_info->master_set, sizeof(dev_info->master_set));
    rc = select(dev_info->max_sd + 1, &dev_info->rx_working_set, NULL, NULL,
                &dev_info->timeout);

    if (rc < 0) {
        fprintf(stderr, "select() failed: %d\n", rc);
        return rc;
    }
    dev_info->desc_ready = rc;
    for (ii = 0; ii <= dev_info->max_sd && dev_info->desc_ready > 0; ++ii) {
        if (FD_ISSET(ii, &dev_info->rx_working_set)) {
            dev_info->desc_ready -= 1;
            if (ii == dev_info->server_fd) {
                printf("  Listening socket is readable\n");
                do {
                    dev_info->new_sd = accept(dev_info->server_fd, NULL, NULL);
                    if (dev_info->new_sd < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        } else {
                            fprintf(stderr, "  accept() failed  %s\n",
                                    strerror(errno));
                            dev_info->end_server = TRUE;
                            break;
                        }
                    }
                    printf("  New incoming connection - %d\n",
                           dev_info->new_sd);
                    FD_SET(dev_info->new_sd, &dev_info->master_set);
                    if (dev_info->new_sd > dev_info->max_sd)
                        dev_info->max_sd = dev_info->new_sd;
                } while (dev_info->new_sd != -1);
            } else {
                printf("  Descriptor %d is readable\n", ii);
                dev_info->close_conn = FALSE;
                while (offset < buff_size) {
                    if ((rdlen = recv(ii, rx_buff + offset, buff_size - offset, 0)) < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            rdlen = 0;
                        } else {
                            fprintf(stderr, "%s: %s\n", __func__,
                                    strerror(errno));
                            dev_info->close_conn = TRUE;
                            break;
                        }
                    } else if (rdlen == 0) {
                        break;
                    }
                    offset += rdlen;
                }
                if (dev_info->close_conn) {
                    close(ii);
                    FD_CLR(ii, &dev_info->master_set);
                    if (ii == dev_info->max_sd) {
                        while (FD_ISSET(dev_info->max_sd,
                                        &dev_info->master_set) == FALSE)
                            dev_info->max_sd -= 1;
                    }
                }
            } /* End of existing connection is readable */
        }     /* End of if (FD_ISSET(i, &rx_working_set)) */
    }         /* End of loop through selectable descriptors */
    return offset;
}

int non_blocking_data_send(void *priv_data,
                           uint8_t *payload,
                           uint32_t frame_len)
{
    uint32_t offset = 0;
    int32_t wdlen = 0;
    int rc = 0, ii;
    struct non_blocking_io_info_t *dev_info = priv_data;

    if (dev_info->end_server) {
        return rc;
    }
    memcpy(&dev_info->tx_working_set, &dev_info->master_set, sizeof(dev_info->master_set));
    rc = select(dev_info->max_sd + 1, NULL, &dev_info->tx_working_set, NULL,
                &dev_info->timeout);

    if (rc < 0) {
        fprintf(stderr, "select() failed: %d\n", rc);
        return rc;
    }
    dev_info->desc_ready = rc;
    for (ii = 0; ii <= dev_info->max_sd && dev_info->desc_ready > 0; ++ii) {
        if (FD_ISSET(ii, &dev_info->tx_working_set)) {
            dev_info->desc_ready -= 1;
            if (ii == dev_info->server_fd) {
                printf("  Listening socket is writeable\n");
                do {
                    dev_info->new_sd = accept(dev_info->server_fd, NULL, NULL);
                    if (dev_info->new_sd < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        } else {
                            fprintf(stderr, "  accept() failed  %s\n",
                                    strerror(errno));
                            dev_info->end_server = TRUE;
                            break;
                        }
                    }
                    printf("  New incoming connection - %d\n",
                           dev_info->new_sd);
                    FD_SET(dev_info->new_sd, &dev_info->master_set);
                    if (dev_info->new_sd > dev_info->max_sd)
                        dev_info->max_sd = dev_info->new_sd;
                } while (dev_info->new_sd != -1);
            } else {
                printf("  Descriptor %d is writeable\n", ii);
                dev_info->close_conn = FALSE;
                while (offset < frame_len) {
                    if ((wdlen = send(ii, payload + offset, frame_len - offset, 0)) < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            wdlen = 0;
                        } else {
                            fprintf(stderr, "%s: %s\n", __func__,
                                    strerror(errno));
                            dev_info->close_conn = TRUE;
                            break;
                        }
                    } else if (wdlen == 0) {
                        break;
                    }
                    offset += wdlen;
                }
                if (dev_info->close_conn) {
                    close(ii);
                    FD_CLR(ii, &dev_info->master_set);
                    if (ii == dev_info->max_sd) {
                        while (FD_ISSET(dev_info->max_sd,
                                        &dev_info->master_set) == FALSE)
                            dev_info->max_sd -= 1;
                    }
                }
            } /* End of existing connection is readable */
        }     /* End of if (FD_ISSET(i, &tx_working_set)) */
    }         /* End of loop through selectable descriptors */
    return offset;
}

struct dcmbus_driver_ops dcmbus_driver_non_blocking_tcp_ops = {
    .open_interface = non_blocking_tcp_init,
    .recv_data = non_blocking_data_recv,
    .send_data = non_blocking_data_send,
    .close_interface = non_blocking_deinit,
};