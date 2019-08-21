#include "accelerometer_ideal.hh"
AccelerometerIdeal::AccelerometerIdeal()
{
    snprintf(name, sizeof(name), "Ideal Accelerometer Sensor");
}

void AccelerometerIdeal::init(LaunchVehicle *VehicleIn)
{
    VehicleIn->Sensor->FSPCB = VehicleIn->DM->FSPB;
    VehicleIn->Sensor->EFSPB.zeros();
}

void AccelerometerIdeal::algorithm(LaunchVehicle *VehicleIn)
{
    VehicleIn->Sensor->FSPCB = VehicleIn->DM->FSPB;
}
