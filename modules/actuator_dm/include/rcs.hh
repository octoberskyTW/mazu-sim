#ifndef __RCS_HH__
#define __RCS_HH__

#include <armadillo>
#include <functional>
#include <vector>
#include "component.hh"
#include "vehicle.hh"

class RCS : public FH_module
{
    TRICK_INTERFACE(RCS);

public:
    RCS();

    virtual void algorithm(LaunchVehicle *VehicleIn);
    virtual void init(LaunchVehicle *VehicleIn);

    std::function<double()> grab_e_roll;
    std::function<double()> grab_e_pitch;
    std::function<double()> grab_e_yaw;
};

#endif  //__RCS_HH__