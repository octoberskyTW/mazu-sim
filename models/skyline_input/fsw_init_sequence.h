#ifndef __FSW_INIT_SEQUENCE_H__
#define __FSW_INIT_SEQUENCE_H__
#include "jit_utils.h"
#include "mission_events_define.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern FlightSoftware_SimObject fc;

const double LONX = 120.49;  //  Vehicle longitude - deg  module newton
const double LATX = 24.68;    //  Vehicle latitude  - deg  module newton
const double ALT = 0.0;     //  Vehicle altitude  - m  module newton
const double PHIBDX = 0.0;    //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX = 0.0;   //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX = -0.0;  //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X = 0;     // Initial angle-of-attack   - deg  module newton
const double BETA0X = 0;      // Initial sideslip angle    - deg  module newton
const double DVBE = 0.0;      // Vehicle geographic speed  - m/s  module newton

/* S1 */
const double S1_XCG_0 = 2.35;            //  vehicle initial xcg
const double S1_XCG_1 = 2.35;            //  vehicle final xcg
const double S1_MOI_ROLL_0 = 12.8289;    //  vehicle initial moi in roll direction
const double S1_MOI_ROLL_1 = 9.6727;     //  vehicle final moi in roll direction
const double S1_MOI_PITCH_0 = 275.5836;  //  vehicle initial transverse moi
const double S1_MOI_PITCH_1 = 232.3524;  //  vehicle final transverse moi
const double S1_MOI_YAW_0 = 275.5382;    //  vehicle initial transverse moi
const double S1_MOI_YAW_1 = 232.327;     //  vehicle final transverse moi
const double S1_vmass0 = 300.0;          //  Vehicle init mass
const double S1_fmass0 = 240.0;          //  Vehicle init fuel mass
const double S1_RP = -3.322;             //  reference point
const double S1_ENG_NUM = 1.0;

extern "C" int event_liftoff(void)
{
    fc.ins.set_liftoff(1);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_LIFTOFF;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_LIFTOFF", fc.ctl_tvc_db.flight_event_code);
    fc.rcs_fc.enable_rcs();
    fc.rcs_fc.set_mode(1);
    return 0;
}

extern "C" int slave_init_stage1_control(FlightSoftware_SimObject *fc)
{
    /* Control variable Stage2 */
    fc->control.set_controller_var(S1_VMASS0, S1_FUEL_FLOW_RATE, S1_FMASS0, S1_XCG_1, S1_XCG_0, 0.0, 0.0);
    fc->control.set_IBBB0(S1_MOI_ROLL_0, S1_MOI_PITCH_0, S1_MOI_YAW_0);
    fc->control.set_IBBB1(S1_MOI_ROLL_1, S1_MOI_PITCH_1, S1_MOI_YAW_1);
    fc->control.set_reference_point(S1_RP);
    fc->control.set_engnum(S1_ENG_NUM);
}

extern "C" int slave_init_ins_variable(FlightSoftware_SimObject *fc)
{
    fc->ins.load_location(LONX, LATX, ALT);
    fc->ins.load_angle(PSIBDX, PHIBDX, THTBDX);
    fc->ins.load_geodetic_velocity(ALPHA0X, BETA0X, DVBE);
    fc->ins.set_ideal();
    //  fc->ins.set_non_ideal();
    uint32_t gpsupdate = 0;
    fc->ins.set_gps_correction(gpsupdate);
    return 0;
}

extern "C" int slave_init_time(FlightSoftware_SimObject *fc)
{
    uint32_t Year = 2020;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    fc->time->load_start_time(Year, DOY, Hour, Min, Sec);
    return 0;
}

extern "C" void flight_events_trigger_configuration(FlightSoftware_SimObject *fc)
{
    /* events */
    jit_add_read(0.001 + fc->stand_still_time, "event_liftoff");
    exec_set_terminate_time(30.001 + fc->stand_still_time);
}
#endif  //  __FSW_INIT_SEQUENCE_H__
