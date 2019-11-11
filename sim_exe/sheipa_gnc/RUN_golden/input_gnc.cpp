#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/exec_proto.h"
#include "trick/external_application_c_intf.h"
#include "trick/framelog_proto.h"
#include "trick/realtimesync_proto.h"

#include "data_record_fsw.h"
#include "fsw_init_sequence.h"

extern "C" int run_me()
{
    //real_time_enable();
    exec_set_software_frame(0.005);
    //exec_set_lock_memory(1);
    // exec_set_thread_priority(0, 1);
    exec_set_thread_cpu_affinity(0, 1);
    frame_log_on();
    fprintf(stderr, "%s\n", real_time_clock_get_name());
    fprintf(stderr, "S_main_Linux_*.exe Process ID : %d\n", getpid());

    record_gps_slave();
    // realtime();
    slave_init_time(&fc);
    /* INS */
    slave_init_ins_variable(&fc);
    /* GPS */
    // slave_init_gps_fc_variable(&fc);

    slave_init_stage1_control(&fc);
    /* events */
    flight_events_trigger_configuration(&fc);
    return 0;
}
