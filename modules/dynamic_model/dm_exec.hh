#ifndef __EXEC_HH__
#define __EXEC_HH__
#include "accel/accelerometer_ideal.hh"
#include "aerodynamics.hh"
#include "earth_environment.hh"
#include "gyro/gyro_ideal.hh"
#include "propulsion.hh"
#include "rcs.hh"
#include "rocket_flight_dm.hh"
#include "sdt/sdt_ideal.hh"
#include "tvc.hh"
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/exec.cpp))
*******************************************************************************/

class Exec
{
public:
    Exec(){};
    ~Exec(){};

    Rocket_Flight_DM dynamics;
    EarthEnvironment env;
    AeroDynamics aerodynamics;
    Propulsion propulsion;
    Gyro *gyro;
    Accelerometer *accelerometer;
    SDT *sdt;
    TVC tvc;
    RCS rcs;
    double int_step;
    double fc_int_step;
    double stand_still_time;

    time_management *time = time_management::get_instance();

    void init(LaunchVehicle *Vehicle);
    void exec1(LaunchVehicle *Vehicle);
    void exec2(LaunchVehicle *Vehicle);
};

#endif
