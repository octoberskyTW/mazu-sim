#ifndef __GYRO_IDEAL_HH__
#define __GYRO_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Ideal Gyro Implementation)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_ideal.cpp))
*******************************************************************************/

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