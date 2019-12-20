#ifndef __PACKET_FORMAT_H__
#define __PACKET_FORMAT_H__


typedef struct __attribute__((packed)) {
    /* gyro */
    double gyro_WBICB[3];
    double gyro_EWBIB[3];

    /* SDT */
    double sdt_phi[3];
    double sdt_delta_vel[3];
} refactor_trick_dirty_data_t;

typedef struct __attribute__((packed)) {
    /* accel */
    double accel_FSPCB[3];
    double accel_EFSPB[3];
    /* GPS Constellation */
    uint32_t gps_con_gps_update;
    refactor_trick_dirty_data_t trick_data;
} refactor_uplink_packet_t;

typedef struct __attribute__((packed)) {
    double theta_a_cmd;
    double theta_b_cmd;
    double theta_c_cmd;
    double theta_d_cmd;
    double e_roll;
    double e_pitch;
    double e_yaw;
    int beco_flag;
    uint64_t flight_event_code;
} refactor_downlink_packet_t;

typedef struct __attribute__((packed)) {
    int flag_ins_clear_gps;

    double ins_dvbec;
    double ins_thtvdcx;
    double ins_thtbdcx;
    double ins_TBDQ[4];
    double ins_TBD[3][3];
    double ins_TBICI[3][3];
    double ins_TBIC[3][3];
    double ins_WBECB[3];
    double ins_ABICB[3];
    double ins_SBIIC[3];
    double ins_VBIIC[3];
    double ins_VBECD[3];

    double ins_alphacx;
    double ins_psibdcx;
    double ins_phibdcx;
    double ins_altc;
    double ins_alppcx;
    double ins_phipcx;

    double accel_FSPCB[3];
    refactor_trick_dirty_data_t trick_data;
} refactor_ins_to_ctl_t;

typedef struct __attribute__((packed)) {
    double guidance_UTBC[3];
} guidnace_packet_t;

#endif  // __PACKET_FORMAT_H__
