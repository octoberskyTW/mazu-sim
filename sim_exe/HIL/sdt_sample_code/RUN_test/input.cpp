#include <cstdlib>
#include <exception>
#include "../S_source.hh"
#include "trick/CheckPointRestart_c_intf.hh"
#include "trick/external_application_c_intf.h"
#include "trick/realtimesync_proto.h"
#include "trick/framelog_proto.h"
#include "trick/exec_proto.h"
extern "C" int run_me() {

    real_time_enable();
    exec_set_software_frame(0.005);
    //exec_set_lock_memory(1);
    //exec_set_thread_priority(0, 1);
    exec_set_thread_cpu_affinity(0, 1);
    frame_log_on();
    fprintf(stderr, "%s\n", real_time_clock_get_name());
    fprintf(stderr, "S_main_Linux_*.exe Process ID : %d\n", getpid());

    exec_set_terminate_time(100.0);
    return 0;
}
