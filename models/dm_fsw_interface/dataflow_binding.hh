#ifndef __DATAFLOW_BINDING_HH__
#define __DATAFLOW_BINDING_HH__


#include "packet_format.h"
#include "control.hh"

#define CONTROL_SAVE_decl()                                          \
  void Control_SaveOutData(Control &control,                         \
                           refactor_downlink_packet_t &ctl_tvc_db) { \
    ctl_tvc_db.act_cmd = control.get_ACT_CMD();              \
    ctl_tvc_db.throttle_cmd = control.get_throttle_cmd(); \
  }
#endif
