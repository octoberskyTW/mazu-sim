#include <cstdlib>
#include <exception>

#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/exec_proto.h"
#include "trick/external_application_c_intf.h"
#include "trick/framelog_proto.h"
#include "trick/realtimesync_proto.h"

#include "../../models/jit_input/data_record_golden.h"
#include "../../models/jit_input/data_record_gps.h"
#include "../../models/jit_input/dm_init_sequence.h"

extern "C" int run_me()
{
    record_gps();
    record_golden();
    //real_time_enable();
    exec_set_software_frame(0.005);
    //exec_set_lock_memory(1);
    //exec_set_thread_priority(0, 1);
    exec_set_thread_cpu_affinity(0, 1);
    frame_log_on();
    fprintf(stderr, "%s\n", real_time_clock_get_name());
    fprintf(stderr, "S_main_Linux_*.exe Process ID : %d\n", getpid());
    master_startup(&dyn);
    fprintf(stderr, "time_tic_value = %d tics per seconds\n", exec_get_time_tic_value());
    fprintf(stderr, "software_frame = %lf second per frame.\n", exec_get_software_frame());
    fprintf(stderr, "software_frame_tics = %lld tics per_frame\n", exec_get_software_frame_tics());
    master_model_configuration(&dyn);
    master_init_time(&dyn);
    master_init_environment(&dyn);
    master_init_slv(&dyn);
    master_init_aerodynamics(&dyn);
    master_init_propulsion(&dyn);
    master_init_sensors(&dyn);
    master_init_tvc(&dyn);
    master_init_rcs(&dyn);
    flight_events_handler_configuration(&dyn);
    return 0;
}
