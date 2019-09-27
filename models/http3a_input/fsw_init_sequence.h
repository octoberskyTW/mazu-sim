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
/* S2 */
const double S2_XCG_0 = 5.91;            //  vehicle initial xcg
const double S2_XCG_1 = 4.17;            //  vehicle final xcg
const double S2_MOI_ROLL_0 = 5.043e3;    //  vehicle initial moi in roll direction
const double S2_MOI_ROLL_1 = 2.047e3;    //  vehicle final moi in roll direction
const double S2_MOI_PITCH_0 = 51.91e3;   //  vehicle initial transverse moi
const double S2_MOI_PITCH_1 = 15.53e3;   //  vehicle final transverse moi
const double S2_MOI_YAW_0 = 51.91e3;     //  vehicle initial transverse moi
const double S2_MOI_YAW_1 = 15.53e3;     //  vehicle final transverse moi
const double S2_SPI = 285.0;             //  Specific impusle
const double S2_FUEL_FLOW_RATE = 189.1;  //  fuel flow rate
const double S2_XCP = 5.0384;            //  Xcp location
const double S2_refa = 3.243;            //  Aerodynamics reference area
const double S2_refd = 2.032;            //  Aerodynamics reference length
const double S2_VMASS0 = 15490.0;        //  Vehicle init mass
const double S2_FMASS0 = 9552.0;         //  Vehicle init fuel mass
const double S2_RP = 9.749;
const double S2_ENG_NUM = 1.0;
/* S3 */
const double S3_XCG_0 = 2.6651;          //  vehicle initial xcg
const double S3_XCG_1 = 2.66507;         //  vehicle final xcg
const double S3_MOI_ROLL_0 = 1.519e3;    //  vehicle initial moi in roll direction
const double S3_MOI_ROLL_1 = 0.486e3;    //  vehicle final moi in roll direction
const double S3_MOI_PITCH_0 = 5.158e3;   //  vehicle initial transverse moi
const double S3_MOI_PITCH_1 = 2.394e3;   //  vehicle final transverse moi
const double S3_MOI_YAW_0 = 5.158e3;     //  vehicle initial transverse moi
const double S3_MOI_YAW_1 = 2.394e3;     //  vehicle final transverse moi
const double S3_SPI = 284.0;             //  Specific impusle
const double S3_FUEL_FLOW_RATE = 44.77;  //  fuel flow rate
const double S3_XCP = 3.2489;            //  Xcp location
const double S3_refa = 3.243;            //  Aerodynamics reference area
const double S3_refd = 2.032;            //  Aerodynamics reference length
const double S3_VMASS0 = 5024.0;         //  Vehicle init mass
const double S3_FMASS0 = 3291.0;         //  Vehicle init fuel mass
const double S3_RP = 3.86;
const double S3_ENG_NUM = 1.0;
/* Controller setting */
const double ZACLP = 1.0;      // Damping of accel close loop complex pole - ND
const double ZACLY = 1.0;      // Damping of accel close loop complex pole - ND
const double FACTWACLP = 0.5;  // Factor to mod 'waclp': waclp*(1+factwacl) - ND
const double FACTWACLY = 0.5;  // Factor to mod 'wacly': wacly*(1+factwacl) - ND
const double ALPHACOMX = 0.0;  // AOA command - deg
const double BETACOMX = 0.0;   // Side slip angle command - deg
const double ROLLCOMX = 0.0;   // Roll angle command - deg

/* Guidance setting */
const double LTG_STEP = 0.1;
const int NUM_STAGES = 2;
const double DBI_DESIRED = 6470e3;
const double DVBI_DESIRED = 6600.0;
const double THTVDX_DESIRED = 1.0;
const double DELAY_IGNITION = 0.1;
const double AMIN = 3.0;
const double LAMD_LIMIT = 0.028;
const double EXHAUST_VEL1 = 2795.0;
const double EXHAUST_VEL2 = 2785.0;
const double BURNOUT_EPOCH1 = 51.5;
const double BURNOUT_EPOCH2 = 126.0;
const double CHAR_TIME1 = 81.9;
const double CHAR_TIME2 = 112.2;


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

extern "C" int event_acc_on(void)
{
    fc.control.set_acc_control();
    fc.rcs_fc.set_mode(0);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S1_CONTROL_ON;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S1_CONTROL_ON);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S1_CONTROL_ON", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_UECO(void)
{
    if (fc.guidance.get_beco_flag()) {
        if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_UECO, fc.egse_flight_event_trigger_bitmap, FLIGHT_EVENT_CODE_UECO))
            return 0;
    } else {
        return 0;
    }

    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_UECO;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_UECO);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_UECO", fc.ctl_tvc_db.flight_event_code);
    fc.rcs_fc.disable_rcs();

    return 0;
}

extern "C" int event_s1_seperation(void)
{
    fc.control.set_controller_var(S2_VMASS0, S2_FUEL_FLOW_RATE, S2_FMASS0, S2_XCG_1, S2_XCG_0, S2_SPI, 0.0);
    fc.control.set_IBBB0(S2_MOI_ROLL_0, S2_MOI_PITCH_0, S2_MOI_YAW_0);
    fc.control.set_IBBB1(S2_MOI_ROLL_1, S2_MOI_PITCH_1, S2_MOI_YAW_1);
    fc.control.set_reference_point(S2_RP);
    // fc.control.set_engine_d(0.0);
    fc.control.set_NO_CONTROL();
    fc.guidance.set_guidance_var(LTG_STEP, NUM_STAGES, DBI_DESIRED, DVBI_DESIRED, THTVDX_DESIRED, DELAY_IGNITION, AMIN, LAMD_LIMIT, EXHAUST_VEL1, EXHAUST_VEL2, BURNOUT_EPOCH1, BURNOUT_EPOCH2, CHAR_TIME1, CHAR_TIME2);
    fc.guidance.set_ltg_guidance();
    fc.rcs_fc.set_mode(2);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S1_SEPERATION;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S1_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S1_SEPERATION", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int event_s2_seperation(void)
{
    fc.control.set_controller_var(S3_VMASS0, S3_FUEL_FLOW_RATE, S3_FMASS0, S3_XCG_1, S3_XCG_0, S3_SPI, 0.0);
    fc.control.set_IBBB0(S3_MOI_ROLL_0, S3_MOI_PITCH_0, S3_MOI_YAW_0);
    fc.control.set_IBBB1(S3_MOI_ROLL_1, S3_MOI_PITCH_1, S3_MOI_YAW_1);
    fc.control.set_reference_point(S3_RP);
    // fc.control.set_engine_d(0.0);
    // fc.control.set_NO_CONTROL();
    // fc.guidance.set_guidance_var(LTG_STEP, NUM_STAGES, DBI_DESIRED, DVBI_DESIRED, THTVDX_DESIRED, DELAY_IGNITION
    //                             , AMIN, LAMD_LIMIT, EXHAUST_VEL1, EXHAUST_VEL2, BURNOUT_EPOCH1
    //                             , BURNOUT_EPOCH2, CHAR_TIME1, CHAR_TIME2);
    // fc.guidance.set_ltg_guidance();
    // fc.rcs_fc.set_mode(2);
    fc.ctl_tvc_db.flight_event_code = FLIGHT_EVENT_CODE_S2_SEPERATION;
    fc.egse_flight_event_trigger_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_S2_SEPERATION);
    PRINT_FLIGHT_EVENT_MESSAGE("FC", exec_get_sim_time(), "FLIGHT_EVENT_CODE_S2_SEPERATION", fc.ctl_tvc_db.flight_event_code);
    return 0;
}

extern "C" int slave_init_stage1_control(FlightSoftware_SimObject *fc)
{
    /* Control variable Stage2 */
    fc->control.set_controller_var(S1_VMASS0, S1_FUEL_FLOW_RATE, S1_FMASS0, S1_XCG_1, S1_XCG_0, S1_SPI, 0.0);
    fc->control.set_IBBB0(S1_MOI_ROLL_0, S1_MOI_PITCH_0, S1_MOI_YAW_0);
    fc->control.set_IBBB1(S1_MOI_ROLL_1, S1_MOI_PITCH_1, S1_MOI_YAW_1);
    fc->rcs_fc.set_rcs_tau(RCS_TAU);
    fc->rcs_fc.set_phibdcomx(S1_ROLL_CMD);
    fc->rcs_fc.set_thtbdcomx(S1_PITCH_CMD);
    fc->rcs_fc.set_psibdcomx(S1_YAW_CMD);

    fc->control.set_ancomx(S1_ANCOMX);
    fc->control.set_alcomx(S1_ALCOMX);
    fc->control.load_aerotable("../../tables/aero_table_slv3.txt");
    fc->control.atmosphere_use_public();
    fc->control.set_close_loop_pole(ZACLP, ZACLY);
    fc->control.set_factor(FACTWACLP, FACTWACLY);
    fc->control.set_aero_coffe(S1_refd, S1_refa, S1_XCP);
    fc->control.set_feedforward_gain(S1_GAINP);
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

// extern "C" int slave_init_gps_fc_variable(FlightSoftware_SimObject *fc)
// {
//     /* GPS */
//     double pr_bias_default[4] = { 0, 0, 0, 0 };
//     double pr_noise_default[4] = { 0.25, 0.25, 0.25, 0.25 };
//     double dr_noise_default[4] = { 0.03, 0.03, 0.03, 0.03 };
//     //  User clock frequency error - m/s MARKOV  module gps
//     fc->gps.ucfreq_noise = 0.1;
//     // User clock bias error - m GAUSS  module gps
//     fc->gps.ucbias_error = 0;
//     // Pseudo-range bias - m GAUSS  module gps
//     memcpy(fc->gps.PR_BIAS, pr_bias_default, sizeof(pr_bias_default));
//     //  Pseudo-range noise - m MARKOV  module gps
//     memcpy(fc->gps.PR_NOISE, pr_noise_default, sizeof(pr_noise_default));
//     //  Delta-range noise - m/s MARKOV  module gps
//     memcpy(fc->gps.DR_NOISE, dr_noise_default, sizeof(dr_noise_default));
//     //  Factor to modifiy initial P-matrix P(1+factp)=module gps
//     double gpsr_factp = 0;
//     //  Init 1sig clock bias error of state cov matrix - m=module gps
//     double gpsr_pclockb = 3;
//     //  Init 1sig clock freq error of state cov matrix - m/s=module gps
//     double gpsr_pclockf = 1;
//     fc->gps.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf);

//     //  Factor to modifiy the Q-matrix Q(1+factq)=module gps
//     double gpsr_factq = 0;
//     //  1sig clock bias error of process cov matrix - m=module gps
//     double gpsr_qclockb = 0.5;
//     //  1sig clock freq error of process cov matrix - m/s=module gps
//     double gpsr_qclockf = 0.1;
//     fc->gps.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf);

//     //  User clock correlation time constant - s=module gps
//     double gpsr_uctime_cor = 100;
//     fc->gps.setup_fundamental_dynamic_matrix(gpsr_uctime_cor);

//     //  Init 1sig pos values of state cov matrix - m=module gps
//     fc->gps.ppos = 5;
//     //  Init 1sig vel values of state cov matrix - m/s=module gps
//     fc->gps.pvel = 0.2;
//     //  1sig pos values of process cov matrix - m=module gps
//     fc->gps.qpos = 0.1;
//     //  1sig vel values of process cov matrix - m/s=module gps
//     fc->gps.qvel = 0.01;
//     //  1sig pos value of meas cov matrix - m=module gps
//     fc->gps.rpos = 1;
//     //  1sig vel value of meas cov matrix - m/s=module gps
//     fc->gps.rvel = 0.1;
//     //  Factor to modifiy the R-matrix R(1+factr)=module gps
//     fc->gps.factr = 0;
//     return 0;
// }

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
    jit_add_read(10.001 + fc->stand_still_time, "event_acc_on");
    jit_add_read(61.6 + fc->stand_still_time, "event_s1_seperation");
    jit_add_read(113.14 + fc->stand_still_time, "event_s2_seperation");
    jit_add_event("event_UECO", "UECO", 0.05);
    exec_set_terminate_time(185.001 + fc->stand_still_time);
}
#endif  //  __FSW_INIT_SEQUENCE_H__
