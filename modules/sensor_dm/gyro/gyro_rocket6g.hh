#ifndef __GYRO_ROCKET6G_HH__
#define __GYRO_ROCKET6G_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Non-Ideal Gyro Implementation from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/gyro/gyro_rocket6g.cpp))
*******************************************************************************/

#include <armadillo>
#include "aux.hh"
#include "gyro.hh"

class GyroRocket6G : public Gyro
{
    TRICK_INTERFACE(GyroRocket6G);

public:
    GyroRocket6G();

    virtual ~GyroRocket6G() {}
    virtual void init(LaunchVehicle *VehicleIn){};
    virtual void algorithm(LaunchVehicle *VehicleIn);

    std::default_random_engine generator;
};

#endif  // __GYRO_ROCKET6G__