#ifndef __RCS_HH__
#define __RCS_HH__

/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the RCS thruster Module)
LIBRARY DEPENDENCY:
      ((../src/RCS.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
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