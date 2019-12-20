#ifndef __DATAFLOW_BINDING_SHEPIA_HH__
#define __DATAFLOW_BINDING_SHEPIA_HH__
#include <stdint.h>

#include <armadillo>
/*
 *  IMU raw packet
 *
 */
typedef struct { /* TODO for FC*/
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
typedef struct {
    double gyro[10][3];
    double accel[10][3];
} navi_ingress_packet_t;

/**
 * Calibration Output
 * Low pass filter input
 */
typedef struct {
    arma::vec3 gyro[10];
    arma::vec3 accel[10];
} calib_to_filter_data_t;

/**
 * INS Data Input
 * Low pass filter Output
 *
 */
typedef struct {
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
    double temper;

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

    //fix term
    arma::mat33 C_Lb1;
    arma::mat33 C_Lb;

    arma::mat33 C_Le1;
    arma::mat33 C_Le;
    arma::vec3 rkt_accel_last;
    arma::vec3 rkt_ang_rate_last;

    // verify
    arma::vec3 beta;
    arma::vec3 scul;
} ins_data_t;  //Low Pass filter output, INS input

/**
 * INS output (Navigation output)
 * Control module Input
 */
typedef struct {
    arma::vec3 rkt_attitude;
    arma::vec3 rkt_angular_rate;
    arma::vec3 rkt_acc;
    arma::vec3 rkt_vel;
    arma::vec3 rkt_pos;
} navi_egress_packet_t;  // nav_to_ctl_t;

#endif  // __DATAFLOW_BINDING_SHEPIA_HH__
