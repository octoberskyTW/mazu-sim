#ifndef __GYRO_HH__
#define __GYRO_HH__

#include <armadillo>
#include <aux.hh>
#include "cadac_constants.hh"
#include "numerical_constants.hh"
#include "stochastic.hh"
#include "vehicle.hh"

class Gyro : public FH_module
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
