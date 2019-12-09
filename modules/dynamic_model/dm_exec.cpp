#include "dm_exec.hh"

void Exec::init(LaunchVehicle *Vehicle)
{
    sdt->init(Vehicle);
    propulsion.init(Vehicle);
    env.init(Vehicle);
    dynamics.init(Vehicle);
    gyro->init(Vehicle);
    accelerometer->init(Vehicle);
    tvc.init(Vehicle);
    int_step = 0.005;
    fc_int_step = 0.005;
    stand_still_time = 0.0;
}

void Exec::exec1(LaunchVehicle *Vehicle)
{
    time->dm_time(int_step);
    env.algorithm(Vehicle);
    propulsion.algorithm(Vehicle);
    gyro->algorithm(Vehicle);
    accelerometer->algorithm(Vehicle);
    sdt->algorithm(Vehicle);
}

void Exec::exec2(LaunchVehicle *Vehicle)
{
    tvc.algorithm(Vehicle);
    dynamics.algorithm(Vehicle);
}