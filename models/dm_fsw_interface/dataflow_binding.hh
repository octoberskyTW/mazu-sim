#ifndef __DATAFLOW_BINDING_HH__
#define __DATAFLOW_BINDING_HH__


#include "packet_format.h"
#include "guidance.hh"
#include "ins.hh"
#include "control.hh"
#define DM_SAVE_decl()                                                       \
  void DM_SaveOutData(refactor_uplink_packet_t &dm_ins_db) {                 \
    STORE_VEC(dm_ins_db.accel_FSPCB, (&V1)->Sensor->FSPCB);                  \
    STORE_VEC(dm_ins_db.accel_EFSPB, (&V1)->Sensor->EFSPB);                  \
    STORE_VEC(dm_ins_db.trick_data.gyro_WBICB, (&V1)->Sensor->WBICB);        \
    STORE_VEC(dm_ins_db.trick_data.gyro_EWBIB, (&V1)->Sensor->EWBIB);        \
    STORE_VEC(dm_ins_db.trick_data.sdt_phi, (&V1)->Sensor->PHI);             \
    STORE_VEC(dm_ins_db.trick_data.sdt_delta_vel, (&V1)->Sensor->DELTA_VEL); \
  }
// #define GPS_LINK_decl()                                                 \
//   void GPSLinkInData(GPS_FSW &gps, refactor_uplink_packet_t &dm_ins_db, \
//                      INS &ins) {                                        \
//     gps.grab_SBIIC = LINK(ins, get_SBIIC);                              \
//     gps.grab_VBIIC = LINK(ins, get_VBIIC);                              \
//     gps.grab_WBICI = LINK(ins, get_WBICI);                              \
//     gps.grab_SBEEC = LINK(ins, get_SBEEC);                              \
//     gps.grab_VBEEC = LINK(ins, get_VBEEC);                              \
//     gps.grab_TEIC = LINK(ins, get_TEIC);                                \
//   }

#define INS_LINK_decl()                                                  \
  void INSLinkInData(INS &ins, refactor_uplink_packet_t &dm_ins_db) {   \
    ins.grab_computed_WBIB = GRAB_VEC3(dm_ins_db.trick_data.gyro_WBICB); \
    ins.grab_error_of_computed_WBIB =                                    \
        GRAB_VEC3(dm_ins_db.trick_data.gyro_EWBIB);                      \
    ins.grab_computed_FSPB = GRAB_VEC3(dm_ins_db.accel_FSPCB);           \
    ins.grab_error_of_computed_FSPB = GRAB_VEC3(dm_ins_db.accel_EFSPB);  \
    ins.grab_PHI = GRAB_VEC3(dm_ins_db.trick_data.sdt_phi);              \
    ins.grab_DELTA_VEL = GRAB_VEC3(dm_ins_db.trick_data.sdt_delta_vel);  \
    ins.grab_gps_update = GRAB_VAR(dm_ins_db.gps_con_gps_update);        \
  }

#define CONTROL_LINK_decl()                                                   \
  void ControlLinkInData(Control &control, refactor_ins_to_ctl_t &ins_ctl_db, \
                         refactor_uplink_packet_t &dm_ins_db) {               \
    control.grab_dvbec = GRAB_VAR(ins_ctl_db.ins_dvbec);                      \
    control.grab_thtvdcx = GRAB_VAR(ins_ctl_db.ins_thtvdcx);                  \
    control.grab_thtbdcx = GRAB_VAR(ins_ctl_db.ins_thtbdcx);                  \
    control.grab_phibdcx = GRAB_VAR(ins_ctl_db.ins_phibdcx);                  \
    control.grab_psibdcx = GRAB_VAR(ins_ctl_db.ins_psibdcx);                  \
    control.grab_TBDQ = GRAB_VAR(arma::vec4(ins_ctl_db.ins_TBDQ));            \
    control.grab_TBD = GRAB_MAT33(ins_ctl_db.ins_TBD);                        \
    control.grab_alphacx = GRAB_VAR(ins_ctl_db.ins_alphacx);                  \
    control.grab_TBICI = GRAB_MAT33(ins_ctl_db.ins_TBICI);                    \
    control.grab_TBIC = GRAB_MAT33(ins_ctl_db.ins_TBIC);                      \
    control.grab_FSPCB = GRAB_VEC3(dm_ins_db.accel_FSPCB);                    \
    control.grab_computed_WBIB =                                              \
        GRAB_VAR(arma::vec3(ins_ctl_db.trick_data.gyro_WBICB));               \
    control.grab_altc = GRAB_VAR(ins_ctl_db.ins_altc);                        \
    control.grab_WBECB = LINK(ins, get_WBECB);                                \
    control.grab_phipcx = GRAB_VAR(ins_ctl_db.ins_phipcx);                    \
    control.grab_alppcx = GRAB_VAR(ins_ctl_db.ins_alppcx);                    \
    control.grab_ABICB = GRAB_VAR(ins_ctl_db.ins_ABICB);                      \
    control.grab_VBECD = GRAB_VAR(ins_ctl_db.ins_VBECD);                      \
  }

#define Guidance_LINK_decl()                                 \
  void GuidanceLinkData(Guidance &guidance,                  \
                        refactor_ins_to_ctl_t &ins_ctl_db) { \
    guidance.grab_thtvdx = GRAB_VAR(ins_ctl_db.ins_thtvdcx); \
    guidance.grab_VBIIC = GRAB_VEC3(ins_ctl_db.ins_VBIIC);   \
    guidance.grab_SBIIC = GRAB_VEC3(ins_ctl_db.ins_SBIIC);   \
    guidance.grab_TBIC = GRAB_MAT33(ins_ctl_db.ins_TBIC);    \
    guidance.grab_FSPCB = GRAB_VEC3(ins_ctl_db.accel_FSPCB); \
  }

#define RCS_LINK_decl()                                                     \
  void RcsLinkInData(RCS_FC &rcs_fc, refactor_ins_to_ctl_t &ins_ctl_db,     \
                     guidnace_packet_t &guidance_rcs_db) {                  \
    rcs_fc.grab_WBICB =                                                     \
        GRAB_VAR(arma::vec3(ins_ctl_db.trick_data.gyro_WBICB));             \
    rcs_fc.grab_phibdcx = GRAB_VAR(ins_ctl_db.ins_phibdcx);                 \
    rcs_fc.grab_thtbdcx = GRAB_VAR(ins_ctl_db.ins_thtbdcx);                 \
    rcs_fc.grab_psibdcx = GRAB_VAR(ins_ctl_db.ins_psibdcx);                 \
    rcs_fc.grab_UTBC = GRAB_VAR(arma::vec3(guidance_rcs_db.guidance_UTBC)); \
  }

#define Guidance_SAVE_decl()                                         \
  void GuidanceSaveOutData(Guidance &guidance,                       \
                           guidnace_packet_t &guidance_rcs_db,       \
                           refactor_downlink_packet_t &ctl_tvc_db) { \
    STORE_VEC(guidance_rcs_db.guidance_UTBC, guidance.get_UTBC());   \
    ctl_tvc_db.beco_flag = guidance.get_beco_flag();                 \
  }

#define INS_SAVE_decl()                                               \
  void INS_SaveOutData(INS &ins, refactor_uplink_packet_t &dm_ins_db, \
                       refactor_ins_to_ctl_t &ins_ctl_db) {           \
    ins_ctl_db.ins_dvbec = ins.get_dvbec();                           \
    ins_ctl_db.ins_thtvdcx = ins.get_thtvdcx();                       \
    STORE_MAT33(ins_ctl_db.ins_TBD, ins.get_TBD());                   \
    STORE_MAT33(ins_ctl_db.ins_TBICI, ins.get_TBICI());               \
    STORE_MAT33(ins_ctl_db.ins_TBIC, ins.get_TBIC());                 \
    STORE_VEC(ins_ctl_db.ins_TBDQ, ins.get_TBDQ());                   \
    ins_ctl_db.ins_alphacx = ins.get_alphacx();                       \
    ins_ctl_db.ins_thtbdcx = ins.get_thtbdcx();                       \
    ins_ctl_db.ins_phibdcx = ins.get_phibdcx();                       \
    ins_ctl_db.ins_psibdcx = ins.get_psibdcx();                       \
    ins_ctl_db.ins_altc = ins.get_altc();                             \
    memcpy(&ins_ctl_db.accel_FSPCB, &ins_ctl_db.accel_FSPCB,          \
           3 * sizeof(double));                                       \
    ins_ctl_db.trick_data = dm_ins_db.trick_data;                     \
    STORE_VEC(ins_ctl_db.ins_WBECB, ins.get_WBECB());                 \
    ins_ctl_db.ins_phipcx = ins.get_phipcx();                         \
    ins_ctl_db.ins_alppcx = ins.get_alppcx();                         \
    STORE_VEC(ins_ctl_db.ins_ABICB, ins.get_ABICB());                 \
    STORE_VEC(ins_ctl_db.ins_SBIIC, ins.get_SBIIC());                 \
    STORE_VEC(ins_ctl_db.ins_VBIIC, ins.get_VBIIC());                 \
    STORE_VEC(ins_ctl_db.ins_VBECD, ins.get_VBECD());                 \
  }

#define CONTROL_SAVE_decl()                                          \
  void Control_SaveOutData(Control &control,                         \
                           refactor_downlink_packet_t &ctl_tvc_db) { \
    ctl_tvc_db.theta_a_cmd = control.get_theta_a_cmd();              \
    ctl_tvc_db.theta_b_cmd = control.get_theta_b_cmd();              \
    ctl_tvc_db.theta_c_cmd = control.get_theta_c_cmd();              \
    ctl_tvc_db.theta_d_cmd = control.get_theta_d_cmd();              \
  }

#define RCS_SAVE_decl()                                          \
  void RCS_SaveOutData(RCS_FC &rcs_fc,                           \
                       refactor_downlink_packet_t &ctl_tvc_db) { \
    ctl_tvc_db.e_roll = rcs_fc.get_e_roll();                     \
    ctl_tvc_db.e_pitch = rcs_fc.get_e_pitch();                   \
    ctl_tvc_db.e_yaw = rcs_fc.get_e_yaw();                       \
  }


#endif
