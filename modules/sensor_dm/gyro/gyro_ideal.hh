#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__

#include <armadillo>
#include <aux.hh>

#include "gyro.hh"

class GyroIdeal : public Gyro
{
    TRICK_INTERFACE(GyroIdeal);

public:
    GyroIdeal();

    virtual ~GyroIdeal() {}
    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);
};

#endif  // __GYRO_IDEAL__