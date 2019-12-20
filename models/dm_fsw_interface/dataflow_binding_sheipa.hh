#ifndef __DATAFLOW_BINDING_SHEPIA_HH__
#define __DATAFLOW_BINDING_SHEPIA_HH__

/*
 *  IMU raw packet
 *
 */
 typedef struct __attribute__((packed)) {  /* TODO for FC*/
    uint16_t gyroX_high[10];
    uint16_t gyroX_low[10];
    uint16_t gyroY_high[10];
    uint16_t gyroY_low[10];
    uint16_t gyroZ_high[10];
    uint16_t gyroZ_low[10];
    uint16_t accelX_high[10];
    uint16_t accelX_low[10];
    uint16_t accelY_high[10];
    uint16_t accelY_low[10];
    uint16_t accelZ_high[10];
    uint16_t accelZ_low[10];
} imu_packet_t;

/*
 *  Navigation raw packet Input wrapper
 *  Calibration Input
 */
typedef struct __attribute__((packed)) {
    double gyro0[3];
    double gyro1[3];
    double gyro2[3];
    double gyro3[3];
    double gyro4[3];
    double gyro5[3];
    double gyro6[3];
    double gyro7[3];
    double gyro8[3];
    double gyro9[3];
    double acce0[3];
    double acce1[3];
    double acce2[3];
    double acce3[3];
    double acce4[3];
    double acce5[3];
    double acce6[3];
    double acce7[3];
    double acce8[3];
    double acce9[3];
} navi_ingress_packet_t;

/**
 * Calibration Output
 * Low pass filter input
 */
typedef struct __attribute__((packed)) {
    arma::vec3 gyro[10];
    arme::vec3 accel[10];
} calib_to_filter_data_t;

/**
 * INS Data Input
 * Low pass filter Output
 * 
 */
typedef struct __attribute__((packed)) {
    // calibration
    arma::vec3 body_ang_rate_last;
    arma::vec3 body_accel_last;
    arma::vec3 body_delta_ang;
    arma::vec3 body_delta_vel;
    arma::vec3 prev_body_delta_ang;
    arma::vec3 prev_body_delta_vel;
    
    // INS
    double lat;
    double prev_lat;
    double lnt;
    double prev_lnt;
    double rkt_height;
    double prev_rkt_height;
    double time;

    arma::vec3 rkt_attitude;

    arma::vec3 r1;
    arma::vec3 r;

    arma::vec3 v1;
    arma::vec3 v;

    arma::vec3 dv_L1;
    arma::vec3 dv_L;

    arma::vec4 q_bL1;
    arma::vec4 q_bL;

    arma::vec4 q_Le1;
    arma::vec4 q_Le;

    arma::mat33 C_bL1;
    arma::mat33 C_bL;
} ins_data_t; //Low Pass filter output, INS input

/**
 * INS output (Navigation output)
 * Control module Input
 */
typedef struct navi_egress_packet_t __attribute__((packed)) {
    double rkt_attitude[3];
    double rkt_angular_rate[3]
    double rkt_acc[3];
    double rkt_vel[3];
    double rkt_pos[3];
} nav_to_ctl_t; // nav_to_ctl_t;

#endif  // __DATAFLOW_BINDING_SHEPIA_HH__