#ifndef __PACKET_FORMAT_H__
#define __PACKET_FORMAT_H__

typedef struct __attribute__((packed)) {
    /* accel */
    double accel_FSPCB[3];
    /* gyro */
    double gyro_WBICB[3];
} refactor_uplink_packet_t;

typedef struct __attribute__((packed)) {
    double theta_a_cmd;
    double theta_b_cmd;
    double theta_c_cmd;
    double theta_d_cmd;
    double throttle_cmd;
    int beco_flag;
    uint64_t flight_event_code;
} refactor_downlink_packet_t;

typedef struct __attribute__((packed)) {
    double guidance_UTBC[3];
} guidnace_packet_t;

#endif  // __PACKET_FORMAT_H__
