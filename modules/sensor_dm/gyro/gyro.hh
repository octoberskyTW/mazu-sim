#ifndef __GYRO_HH__
#define __GYRO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Gyro model interface definition)
*******************************************************************************/

#include <armadillo>
#include <aux.hh>
#include "cadac_constants.hh"
#include "module.hh"
#include "numerical_constants.hh"
#include "stochastic.hh"

class Gyro : public Sensor
{
    TRICK_INTERFACE(Gyro);

public:
    char name[256];

    Gyro(){};

    virtual ~Gyro() {}
    virtual void init(LaunchVehicle *VehicleIn) = 0;
    virtual void algorithm(LaunchVehicle *VehicleIn) = 0;

    // virtual arma::vec3 get_computed_WBIB() { return WBICB; }
    // virtual arma::vec3 get_error_of_computed_WBIB() { return EWBIB; }
};

#endif  // __GYRO_HH__
