#ifndef __SDT_HH__
#define __SDT_HH__
#include <armadillo>
#include <functional>
#include "vehicle.hh"

class SDT : public FH_module
{
    TRICK_INTERFACE(SDT);

public:
    SDT(){};
    virtual ~SDT(){};

    virtual void init(LaunchVehicle *VehicleIn) = 0;
    virtual void algorithm(LaunchVehicle *VehicleIn) = 0;

    unsigned int k_limit; /* *o (--) sdt cycle limit */
};

#endif __SDT_HH__
