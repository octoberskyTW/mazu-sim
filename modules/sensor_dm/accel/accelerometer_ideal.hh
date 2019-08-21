#ifndef __ACCELEROMETER_IDEAL_HH__
#define __ACCELEROMETER_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Accelerometer Implementation)
LIBRARY DEPENDENCY:
      ((../../src/accel/accelerometer_ideal.cpp))
*******************************************************************************/

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
