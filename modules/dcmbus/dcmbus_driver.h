#ifndef __DCMBUS_DRIVER_H__
#define __DCMBUS_DRIVER_H__
struct dcmbus_driver_ops {
    int (*open_interface)(void **priv_data, char *ifname, int netport);
    int (*recv_data)(void *priv_data, uint8_t *rx_buff, uint32_t buff_size);
    int (*send_data)(void *priv_data, uint8_t *payload, uint32_t frame_len);
    int (*header_set)(void *priv_data, const uint8_t *payload, const uint32_t data_len);
    int (*header_copy)(void *priv_data, uint8_t *out_buff);
    uint32_t (*get_header_size)(void *priv_data);
    int  (*select)(void *priv_data, struct timeval *timeout);
    void (*fd_clr)(void *priv_data);
    int  (*fd_isset)(void *priv_data);
    void (*fd_set)(void *priv_data);
    void (*fd_zero)(void *priv_data);
    int (*is_server)(void *priv_data);
    int (*close_interface)(void **priv_data);
    int (*get_client_fd)(void *priv_data);
    int (*accept)(void *priv_data);
};

//extern struct dcm_driver_ops *dcm_drivers[];

#endif  //  __DCMBUS_DRIVER_H__