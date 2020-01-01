#ifndef __FSW_INIT_SEQUENCE_H__
#define __FSW_INIT_SEQUENCE_H__
#include "jit_utils.h"
#include "mission_events_define.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern FlightSoftware_SimObject fc;

const double LONX = -120.49;  //  Vehicle longitude - deg  module newton
const double LATX = 34.68;    //  Vehicle latitude  - deg  module newton
const double ALT = 100.0;     //  Vehicle altitude  - m  module newton
const double PHIBDX = 0.0;    //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX = 90.0;   //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX = -83.0;  //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X = 0;     // Initial angle-of-attack   - deg  module newton
const double BETA0X = 0;      // Initial sideslip angle    - deg  module newton
const double DVBE = 1.0;      // Vehicle geographic speed  - m/s  module newton
const double RCS_TAU = 1.0;   // Slope of the switching function - sec RCS
/* S1 */
const double S1_XCG_0 = 10.53;           //  vehicle initial xcg
const double S1_XCG_1 = 6.76;            //  vehicle final xcg
const double S1_MOI_ROLL_0 = 21.94e3;    //  vehicle initial moi in roll direction
const double S1_MOI_ROLL_1 = 6.95e3;     //  vehicle final moi in roll direction
const double S1_MOI_PITCH_0 = 671.62e3;  //  vehicle initial transverse moi
const double S1_MOI_PITCH_1 = 158.83e3;  //  vehicle final transverse moi
const double S1_MOI_YAW_0 = 671.62e3;    //  vehicle initial transverse moi
const double S1_MOI_YAW_1 = 158.83e3;    //  vehicle final transverse moi
const double S1_SPI = 279.2;             //  Specific impusle
const double S1_FUEL_FLOW_RATE = 514.1;  //  fuel flow rate
const double S1_XCP = 8.6435;            //  Xcp location
const double S1_refa = 3.243;            //  Aerodynamics reference area
const double S1_refd = 2.032;            //  Aerodynamics reference length
const double S1_VMASS0 = 48984.0;        //  Vehicle init mass
const double S1_FMASS0 = 31175.0;        //  Vehicle init fuel mass
const double S1_RP = 16.84;              //  reference point
const double S1_ROLL_CMD = 0.0;          //  Roll command - deg
const double S1_PITCH_CMD = 80.0;
const double S1_YAW_CMD = -83.0;
const double S1_ANCOMX = -0.15;
const double S1_ALCOMX = 0.0;
const double S1_GAINP = 0.0;
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
    fc->control.set_controller_var(S1_VMASS0, S1_FUEL_FLOW_RATE, S1_FMASS0, S1_XCG_1, S1_XCG_0, S1_SPI, 0.0);
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
    unsigned int Year = 2017;
    unsigned int DOY = 81;
    unsigned int Hour = 2;
    unsigned int Min = 0;
    unsigned int Sec = 0;
    fc->time->load_start_time(Year, DOY, Hour, Min, Sec);
    return 0;
}

extern "C" void flight_events_trigger_configuration(FlightSoftware_SimObject *fc)
{
    /* events */
    jit_add_read(0.001 + fc->stand_still_time, "event_liftoff");
    exec_set_terminate_time(185.001 + fc->stand_still_time);
}
#endif  //  __FSW_INIT_SEQUENCE_H__
