#include "vehicle.hh"

LaunchVehicle::LaunchVehicle(double step_in)
{
    Aero = new Aerodynamics_var;
    Env = new EarthEnvironment_var;
    DM = new DM_var;
    Prop = new Prop_var;
    ACT = new ACT_var;
    Sensor = new Sensor_var;
    dt = step_in;
}

void LaunchVehicle::set_refa(double in) { Aero->refa = in; }
void LaunchVehicle::set_refd(double in) { Aero->refd = in; }
void LaunchVehicle::set_XCP(double in) { Aero->XCP(0) = in; }
void LaunchVehicle::set_reference_point(double rp) { DM->reference_point = rp; }
void LaunchVehicle::set_liftoff(int in) { DM->liftoff = in; }
void LaunchVehicle::load_angle(double yaw, double roll, double pitch)
{
    DM->psibdx = yaw;
    DM->phibdx = roll;
    DM->thtbdx = pitch;
}

void LaunchVehicle::Allocate_ENG(int NumEng, std::vector<ENG *> &Eng_list_In)
{
    ENG **ENG_list = new ENG *[NumEng];
    for (int i = 0; i < NumEng; i++) {
        ENG_list[i] = new ENG;
        Eng_list_In.push_back(ENG_list[i]);
    }
}

void LaunchVehicle::load_angular_velocity(double ppx_in, double qqx_in,
                                          double rrx_in)
{
    // body rate wrt Earth frame in body coordinates
    DM->WBEB = { ppx_in * RAD, qqx_in * RAD, rrx_in * RAD };
}

void LaunchVehicle::load_location(double lonx_in, double latx_in,
                                  double alt_in)
{
    DM->lonx = lonx_in;
    DM->latx = latx_in;
    DM->alt = alt_in;
}

void LaunchVehicle::load_geodetic_velocity(double alpha0x, double beta0x,
                                           double dvbe)
{
    DM->_dvbe = dvbe;
    DM->alphax = alpha0x;
    DM->betax = beta0x;
}
void LaunchVehicle::set_no_thrust() { Prop->thrust_state = NO_THRUST; }

void LaunchVehicle::Allocate_stage(unsigned int stage_num)
{
    struct STAGE_VAR **stage_var = new STAGE_VAR *[stage_num];
    unsigned int ii = 0;
    for (ii = 0; ii < stage_num; ii++) {
        stage_var[ii] = new STAGE_VAR;
        Stage_var_list.push_back(stage_var[ii]);
    }
}

void LaunchVehicle::set_stage_var(double isp, double fmass_init,
                                  double vmass_init, double aexit_in,
                                  double fuel_flow_rate_in, double xcg0,
                                  double xcg1, double moi_roll0,
                                  double moi_roll1, double moi_pitch0,
                                  double moi_pitch1, double moi_yaw0,
                                  double moi_yaw1, unsigned int num_stage)
{
    Stage_var_list[num_stage]->IBBB0.zeros();
    Stage_var_list[num_stage]->IBBB1.zeros();
    Stage_var_list[num_stage]->XCG_0.zeros();
    Stage_var_list[num_stage]->XCG_1.zeros();

    Stage_var_list[num_stage]->isp = isp;
    Stage_var_list[num_stage]->fmass0 = fmass_init;
    Stage_var_list[num_stage]->StageMass0 = vmass_init;
    Stage_var_list[num_stage]->fuel_flow_rate = fuel_flow_rate_in;
    Stage_var_list[num_stage]->IBBB0(0, 0) = moi_roll0;
    Stage_var_list[num_stage]->IBBB1(0, 0) = moi_roll1;
    Stage_var_list[num_stage]->IBBB0(1, 1) = moi_pitch0;
    Stage_var_list[num_stage]->IBBB1(1, 1) = moi_pitch1;
    Stage_var_list[num_stage]->IBBB0(2, 2) = moi_yaw0;
    Stage_var_list[num_stage]->IBBB1(2, 2) = moi_yaw1;
    Stage_var_list[num_stage]->XCG_0(0) = xcg0;
    Stage_var_list[num_stage]->XCG_1(0) = xcg1;
    Stage_var_list[num_stage]->fmasse = 0.0;
    Stage_var_list[num_stage]->fmassd = 0.0;
    Stage_var_list[num_stage]->aexit = aexit_in;
}

void LaunchVehicle::Allocate_RCS(int num,
                                 std::vector<RCS_Thruster *> &RT_list)
{
    RCS_Thruster **RT_LIST = new RCS_Thruster *[num];
    for (int i = 0; i < num; i++) {
        RT_LIST[i] = new RCS_Thruster;
        RT_list.push_back(RT_LIST[i]);
    }
}

void LaunchVehicle::engine_ignition() { Prop->thrust_state = INPUT_THRUST; }
void LaunchVehicle::set_stage_2() { Prop->stage = STAGE_2; }
void LaunchVehicle::set_stage_3() { Prop->stage = STAGE_3; }
void LaunchVehicle::set_faring_sep() { Prop->stage = FARING_SEP; }
void LaunchVehicle::set_faring_mass(double in) { Prop->faring_mass = in; }
void LaunchVehicle::set_stage_1() { Prop->stage = STAGE_1; }
void LaunchVehicle::set_payload_mass(double in) { Prop->payload_mass = in; }
void LaunchVehicle::set_mtvc(enum TVC_TYPE in) { ACT->mtvc = in; }
void LaunchVehicle::set_S1_TVC() { ACT->mtvc = S1_TVC; }
void LaunchVehicle::set_S2_TVC() { ACT->mtvc = S2_TVC; }
void LaunchVehicle::set_S3_TVC() { ACT->mtvc = S3_TVC; }