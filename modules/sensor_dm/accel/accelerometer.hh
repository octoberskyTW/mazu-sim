#ifndef __ACCELEROMETER_HH__
#define __ACCELEROMETER_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Accelerometer model interface definition)
*******************************************************************************/
#include <armadillo>
#include <aux.hh>
#include "cadac_constants.hh"
#include "module.hh"
#include "numerical_constants.hh"
#include "stochastic.hh"
class Accelerometer : public Sensor
{
    TRICK_INTERFACE(Accelerometer);

public:
    char name[256];

    Accelerometer(){};

    virtual ~Accelerometer() {}
    virtual void init(LaunchVehicle *VehicleIn) = 0;
    virtual void algorithm(LaunchVehicle *VehicleIn) = 0;

    // virtual arma::vec3 get_computed_FSPB() { return FSPCB; }
    // virtual arma::vec3 get_error_of_computed_FSPB() { return EFSPB; }
};

#endif  // __ACCELEROMETER_HH__
