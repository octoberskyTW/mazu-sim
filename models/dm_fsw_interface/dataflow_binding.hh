#ifndef __DATAFLOW_BINDING_HH__
#define __DATAFLOW_BINDING_HH__


#include "packet_format.h"
#include "control.hh"

#define CONTROL_SAVE_decl()                                          \
  void Control_SaveOutData(Control &control,                         \
                           refactor_downlink_packet_t &ctl_tvc_db) { \
    ctl_tvc_db.theta_a_cmd = control.get_theta_a_cmd();              \
    ctl_tvc_db.theta_b_cmd = control.get_theta_b_cmd();              \
    ctl_tvc_db.theta_c_cmd = control.get_theta_c_cmd();              \
    ctl_tvc_db.theta_d_cmd = control.get_theta_d_cmd();              \
    ctl_tvc_db.throttle_cmd = control.get_throttle_cmd(); \
  }

#define DM_SAVE_decl() void DM_SaveOutData(refactor_uplink_packet_t &dm_ins_db) { \
    STORE_VEC(dm_ins_db.accel_FSPCB, accelerometer->get_computed_FSPB()); \
    STORE_VEC(dm_ins_db.trick_data.gyro_WBICB, gyro->get_computed_WBIB()); \
}
#endif
