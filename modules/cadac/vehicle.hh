#ifndef __VEHICLE_HH__
#define __VEHICLE_HH__
#include <armadillo>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
#include "aux.hh"
#include "component.hh"
#include "cadac_constants.hh"
#include "vehicle_var.hh"
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Simulation module abstraction class and vehicle class)
LIBRARY DEPENDENCY:
      ((../src/Vehicle.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
class Vehicle
{
protected:
    char Name[32];
    int module_num;

public:
    Vehicle(){};
    ~Vehicle(){};

    void set_name(char *in) { strcpy(Name, in); }
    char *get_name() { return Name; }
};

class LaunchVehicle : public Vehicle
{
public:
    LaunchVehicle(double step_in);
    ~LaunchVehicle(){};

    /* Set Aerodynamics variables */
    void set_refa(double in);
    void set_refd(double in);
    void set_XCP(double in);
    void set_reference_point(double rp);
    void set_liftoff(int in);
    void load_angle(double yaw, double roll, double pitch);
    void load_angular_velocity(double ppx_in, double qqx_in, double rrx_in);
    void load_location(double lonx_in, double latx_in, double alt_in);
    void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);

    /* Set stage variables */
    void Allocate_stage(unsigned int stage_num);
    void set_stage_var(double isp, double fmass_init, double vmass_init,
                       double aexit_in, double fuel_flow_rate_in, double xcg0,
                       double xcg1, double moi_roll0, double moi_roll1,
                       double moi_pitch0, double moi_pitch1, double moi_yaw0,
                       double moi_yaw1, unsigned int num_stage);
    void Allocate_RCS(int num, std::vector<RCS_Thruster *> &RT_list);
    void Allocate_ENG(int NumEng, std::vector<ENG *> &Eng_list_In);
    void set_payload_mass(double in);
    void set_faring_mass(double in);
    void set_stage_1();
    void set_stage_2();
    void set_stage_3();
    void set_faring_sep();
    void engine_ignition();
    void set_no_thrust();
    void set_mtvc(enum TVC_TYPE);
    void set_S1_TVC();
    void set_S2_TVC();
    void set_S3_TVC();

    Aerodynamics_var *Aero;
    EarthEnvironment_var *Env;
    DM_var *DM;
    Prop_var *Prop;
    ACT_var *ACT;
    Sensor_var *Sensor;

    std::vector<struct STAGE_VAR *> Stage_var_list;
    std::vector<struct ENG *> Eng_list;
    std::vector<RCS_Thruster *> Thruster_list;
    std::vector<ENG *> S1_Eng_list;
    std::vector<ENG *> S2_Eng_list;
    std::vector<ENG *> S3_Eng_list;

    double dt;
};

class FH_module
{
public:
    FH_module(){};
    ~FH_module(){};

    virtual void algorithm(LaunchVehicle *VehicleIn) = 0;
    virtual void init(LaunchVehicle *VehicleIn) = 0;

protected:
    // Data_exchang *data_exchang;
};

// class Vehicle_list {
//  private:
//   int howmany;
//   Vehicle **vehicle_ptr;

//  public:
//   Vehicle_list(){};
//   ~Vehicle_list(){};
//   void Add_vehicle(Vehicle *ptr);
// };
#endif  // __VEHICLE_HH__