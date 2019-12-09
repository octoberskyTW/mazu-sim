#ifndef __DM_INIT_SEQUENCE_H__
#define __DM_INIT_SEQUENCE_H__
#include "jit_utils.h"
#include "mission_events_define.h"
#include "trick/exec_proto.h"
#include "trick/jit_input_file_proto.hh"
extern Rocket_SimObject dyn;
const double LONX = -120.49;        //  Vehicle longitude - deg  module newton
const double LATX = 34.68;          //  Vehicle latitude  - deg  module newton
const double ALT = 1.0;           //  Vehicle altitude  - m  module newton
const double PHIBDX = 0.0;          //  Rolling  angle of veh wrt geod coord - deg  module kinematics
const double THTBDX = 90.0;         //  Pitching angle of veh wrt geod coord - deg  module kinematics
const double PSIBDX = -83.0;        //  Yawing   angle of veh wrt geod coord - deg  module kinematics
const double ALPHA0X = 0;           // Initial angle-of-attack   - deg  module newton
const double BETA0X = 0;            // Initial sideslip angle    - deg  module newton
const double DVBE = 1.0;            // Vehicle geographic speed  - m/s  module newton
const unsigned int CYCLE = 2;
/* S1 */
const double S1_XCG_0 = 2.35;           //  vehicle initial xcg
const double S1_XCG_1 = 2.35;            //  vehicle final xcg
const double S1_MOI_ROLL_0 = 12.8289;    //  vehicle initial moi in roll direction
const double S1_MOI_ROLL_1 = 9.6727;     //  vehicle final moi in roll direction
const double S1_MOI_PITCH_0 = 275.5836;  //  vehicle initial transverse moi
const double S1_MOI_PITCH_1 = 232.3524;  //  vehicle final transverse moi
const double S1_MOI_YAW_0 = 275.5382;    //  vehicle initial transverse moi
const double S1_MOI_YAW_1 = 232.327;    //  vehicle final transverse moi
const double S1_SPI = 279.2;             //  Specific impusle
const double S1_vmass0 = 300.0;        //  Vehicle init mass
const double S1_fmass0 = 240.0;        //  Vehicle init fuel mass
const double S1_RP = -16.84;             //  reference point

extern "C" int event_start()
{
    if (!IS_FLIGHT_EVENT_ARRIVED(FLIGHT_EVENT_CODE_LIFTOFF, dyn.egse_flight_event_handler_bitmap, dyn.flight_event_code_record))
        return 0;
    dyn.egse_flight_event_handler_bitmap &= ~(0x1U << FLIGHT_EVENT_CODE_LIFTOFF);
    PRINT_FLIGHT_EVENT_MESSAGE("EGSE", exec_get_sim_time(), "Recived flight_event_code", dyn.flight_event_code_record);
    dyn.V1.engine_ignition();
    dyn.V1.set_S1_TVC();
    return 0;
}

extern "C" void master_startup(Rocket_SimObject *dyn)
{
    dyn->egse_flight_event_handler_bitmap &= ~(0x1U << 0);
}

extern "C" int master_model_configuration(Rocket_SimObject *dyn)
{
    dyn->Sim.dynamics.set_DOF(6);
    dyn->Sim.dynamics.set_aero_flag(0);
    dyn->V1.set_liftoff(0);  // 1 only for test
}

extern "C" void master_init_time(Rocket_SimObject *dyn)
{
    /********************************Set simulation start time*****************************************************/
    uint32_t Year = 2017;
    uint32_t DOY = 81;
    uint32_t Hour = 2;
    uint32_t Min = 0;
    uint32_t Sec = 0;
    dyn->Sim.time->load_start_time(Year, DOY, Hour, Min, Sec);
}

extern "C" void master_init_environment(Rocket_SimObject *dyn)
{
    /***************************************environment*************************************************************/
    dyn->Sim.env.atmosphere_use_public();
    dyn->Sim.env.set_no_wind();
    dyn->Sim.env.set_no_wind_turbulunce();
}

extern "C" void master_init_slv(Rocket_SimObject *dyn)
{
    /****************************************SLV************************************************************************/
    double lonx = LONX;  //  Vehicle longitude - deg  module newton
    double latx = LATX;  //  Vehicle latitude  - deg  module newton
    double alt = ALT;    //  Vehicle altitude  - m  module newton
    dyn->V1.load_location(lonx, latx, alt);

    double phibdx = PHIBDX;  //  Rolling  angle of veh wrt geod coord - deg  module kinematics
    double thtbdx = THTBDX;  //  Pitching angle of veh wrt geod coord - deg  module kinematics
    double psibdx = PSIBDX;  //  Yawing   angle of veh wrt geod coord - deg  module kinematics
    dyn->V1.load_angle(psibdx, phibdx, thtbdx);

    double alpha0x = ALPHA0X;  // Initial angle-of-attack   - deg  module newton
    double beta0x = BETA0X;    // Initial sideslip angle    - deg  module newton
    double dvbe = DVBE;        // Vehicle geographic speed  - m/s  module newton
    dyn->V1.load_geodetic_velocity(alpha0x, beta0x, dvbe);
    dyn->V1.load_angular_velocity(0, 0, 0);
}

extern "C" void master_init_propulsion(Rocket_SimObject *dyn)
{
    /******************************propulsion & mass property***************************************************************************/
    dyn->V1.Allocate_stage(3);
    dyn->V1.set_stage_var(S1_SPI, S1_fmass0, S1_vmass0, 0.0, S1_FUEL_FLOW_RATE, S1_XCG_0, S1_XCG_1,
                          S1_MOI_ROLL_0, S1_MOI_ROLL_1, S1_MOI_PITCH_0, S1_MOI_PITCH_1, S1_MOI_YAW_0, S1_MOI_YAW_1,
                          0);
    dyn->V1.set_stage_var(S2_SPI, S2_fmass0, S2_vmass0, 0.0, S2_FUEL_FLOW_RATE, S2_XCG_0, S2_XCG_1,
                          S2_MOI_ROLL_0, S2_MOI_ROLL_1, S2_MOI_PITCH_0, S2_MOI_PITCH_1, S2_MOI_YAW_0, S2_MOI_YAW_1,
                          1);
    dyn->V1.set_stage_var(S3_SPI, S3_fmass0, S3_vmass0, 0.0, S3_FUEL_FLOW_RATE, S3_XCG_0, S3_XCG_1,
                          S3_MOI_ROLL_0, S3_MOI_ROLL_1, S3_MOI_PITCH_0, S3_MOI_PITCH_1, S3_MOI_YAW_0, S3_MOI_YAW_1,
                          2);
    // dyn->dynamics.set_reference_point(S1_RP);
    dyn->Sim.dynamics.set_reference_point_eq_xcg();
    dyn->V1.set_stage_1();
    dyn->V1.set_no_thrust();
}

extern "C" void master_init_sensors(Rocket_SimObject *dyn)
{
    /**************************************************Sensor****************************************************************/
    // Accelerometer
    double EMISA[3];   // gauss(0, 1.1e-4)
    double ESCALA[3];  // gauss(0, 2.e-5)
    double EBIASA[3];  // gauss(0, 1.e-6)

    // Create a Ideal Accelerometer
    dyn->Sim.accelerometer = new AccelerometerIdeal();

    // gyro
    double EMISG[3];   // gauss(0, 1.1e-4)
    double ESCALG[3];  // gauss(0, 2.e-5)
    double EBIASG[3];  // gauss(0, 1.e-6)

    // Create a Ideal Gyro
    dyn->Sim.gyro = new GyroIdeal();

    // dyn->sdt = new SDT_NONIDEAL();
    dyn->Sim.sdt = new SDT_ideal(CYCLE);
}

extern "C" void master_init_tvc(Rocket_SimObject *dyn)
{
    /****************************************************TVC*************************************************************************/
    dyn->V1.Allocate_ENG(1, dyn->V1.S1_Eng_list);
    dyn->V1.Allocate_ENG(1, dyn->V1.S2_Eng_list);
    dyn->V1.Allocate_ENG(1, dyn->V1.S3_Eng_list);

    // Allocate S1 Engine position
    dyn->V1.S1_Eng_list[0]->set_ENG_HINGE_POS(S1_RP, 0.0, 0.0);

    // Allocate S1 Engine gimbal direction
    dyn->V1.S1_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S2 Engine position
    dyn->V1.S2_Eng_list[0]->set_ENG_HINGE_POS(S2_RP, 0.0, 0.0);

    // Allocate S2 Engine gimbal direction
    dyn->V1.S2_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S3 Engine position
    dyn->V1.S3_Eng_list[0]->set_ENG_HINGE_POS(S3_RP, 0.0, 0.0);

    // Allocate S3 Engine gimbal direction
    dyn->V1.S3_Eng_list[0]->set_ENG_Dir(3);

    // Allocate S1 Actuator
    for (int i = 0; i < dyn->V1.S1_Eng_list.size(); i++)
        dyn->V1.S1_Eng_list[i]->Allocate_Actuator(2, NO_DYN);

    // Allocate S2 Actuator
    for (int i = 0; i < dyn->V1.S2_Eng_list.size(); i++)
        dyn->V1.S2_Eng_list[i]->Allocate_Actuator(2, NO_DYN);

    // Allocate S3 Actuator
    for (int i = 0; i < dyn->V1.S3_Eng_list.size(); i++)
        dyn->V1.S3_Eng_list[i]->Allocate_Actuator(2, NO_DYN);
}

extern "C" void flight_events_handler_configuration(Rocket_SimObject *dyn)
{
    /* events */
    jit_add_event("event_start", "LIFTOFF", 0.005);
    exec_set_terminate_time(185.001 + dyn->stand_still_time);
}
#endif  //  __DM_INIT_SEQUENCE_H__