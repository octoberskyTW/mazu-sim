#include <armadillo>
#include <cstring>

#include "gyro_ideal.hh"

GyroIdeal::GyroIdeal() { snprintf(name, sizeof(name), "Ideal Gyro Sensor"); }

void GyroIdeal::init(LaunchVehicle *VehicleIn)
{
    VehicleIn->Sensor->WBICB = VehicleIn->DM->WBIB;
    VehicleIn->Sensor->EWBIB.zeros();
    return;
}

void GyroIdeal::algorithm(LaunchVehicle *VehicleIn)
{
    VehicleIn->Sensor->WBICB = VehicleIn->DM->WBIB;
    return;
}
