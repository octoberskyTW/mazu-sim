#include "integrate.hh"

#include "propulsion.hh"

Propulsion::Propulsion() {}

void Propulsion::init(LaunchVehicle *VehicleIn)
{
    VehicleIn->Prop->IBBB = calculate_IBBB(VehicleIn);
    VehicleIn->Prop->XCG = calculate_XCG(VehicleIn);
    VehicleIn->Prop->vmass = VehicleIn->Stage_var_list[STAGE_1]->StageMass0;

    // data_exchang->hset("vmass", VehicleIn->Prop->vmass);
    // data_exchang->hset("IBBB", VehicleIn->Prop->IBBB);
    // data_exchang->hset("XCG", VehicleIn->Prop->XCG);
    // data_exchang->hset("XCG_0", VehicleIn->Stage_var_list[STAGE_1]->XCG_0);
}

void Propulsion::algorithm(LaunchVehicle *VehicleIn)
{
    double psl(101300);  // chamber pressure - Pa
    Prop_var *Prop;
    Prop = VehicleIn->Prop;
    EarthEnvironment_var *E;
    E = VehicleIn->Env;

    double int_step = VehicleIn->dt;
    enum STAGE stage = Prop->stage;
    STAGE_VAR *Stage_var;
    Stage_var = VehicleIn->Stage_var_list[stage];

    double throttle_cmd = grab_throttle_cmd();
    double press = E->press;

    switch (Prop->thrust_state) {
    case NO_THRUST:
        Prop->thrust = 0;
        break;
    case INPUT_THRUST:
        Prop->thrust = throttle_cmd * 0.28235;
        Stage_var->fmasse = integrate(throttle_cmd, Stage_var->fmassd,
                                      Stage_var->fmasse, int_step);
        Stage_var->fmassd = throttle_cmd;
        Prop->mass_ratio = Stage_var->fmasse / Stage_var->fmass0;
        Prop->vmass = Stage_var->StageMass0 - Stage_var->fmasse;
        Prop->fmassr = Stage_var->fmass0 - Stage_var->fmasse;
        Prop->IBBB = calculate_IBBB(VehicleIn);
        Prop->XCG = calculate_XCG(VehicleIn);

        if (Prop->fmassr <= 0.0)
            Prop->thrust_state = NO_THRUST;
        break;
    }
}

void Propulsion::propagate_delta_v(LaunchVehicle *VehicleIn)
{
    enum STAGE stage = VehicleIn->Prop->stage;
    STAGE_VAR *Stage_var;
    Stage_var = VehicleIn->Stage_var_list[stage];
    VehicleIn->Prop->delta_v +=
        Stage_var->isp * AGRAV *
        (Stage_var->fuel_flow_rate / VehicleIn->Prop->vmass) * VehicleIn->dt;
}

double Propulsion::calculate_fmassr(LaunchVehicle *VehicleIn)
{
    enum STAGE stage = VehicleIn->Prop->stage;
    STAGE_VAR *Stage_var;
    Stage_var = VehicleIn->Stage_var_list[stage];
    return Stage_var->fmass0 - Stage_var->fmasse;
}

arma::vec3 Propulsion::calculate_XCG(LaunchVehicle *VehicleIn)
{
    enum STAGE stage = VehicleIn->Prop->stage;
    STAGE_VAR *Stage_var;
    Stage_var = VehicleIn->Stage_var_list[stage];
    return -(Stage_var->XCG_0 +
             (Stage_var->XCG_1 - Stage_var->XCG_0) * VehicleIn->Prop->mass_ratio);
}

arma::mat33 Propulsion::calculate_IBBB(LaunchVehicle *VehicleIn)
{
    enum STAGE stage = VehicleIn->Prop->stage;
    STAGE_VAR *Stage_var;
    Stage_var = VehicleIn->Stage_var_list[stage];
    return Stage_var->IBBB0 +
           (Stage_var->IBBB1 - Stage_var->IBBB0) * VehicleIn->Prop->mass_ratio;
}

void Propulsion::load_proptable(const char *filename)
{
    proptable = Datadeck(filename);
}
