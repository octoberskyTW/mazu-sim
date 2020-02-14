#ifndef __ACCELEROMETER_IDEAL_HH__
#define __ACCELEROMETER_IDEAL_HH__

#include <armadillo>
#include "aux.hh"

#include "accelerometer.hh"

class AccelerometerIdeal : public Accelerometer
{
    TRICK_INTERFACE(AccelerometerIdeal);

public:
    AccelerometerIdeal();
    virtual ~AccelerometerIdeal() {}
    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);
};

#endif  // __ACCELEROMETER_IDEAL_HH__
