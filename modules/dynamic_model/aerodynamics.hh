#ifndef __AERODYNAMICS_HH__
#define __AERODYNAMICS_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the AERODYNAMICS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Aerodynamics.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "aux.hh"
#include "cadac_constants.hh"
#include "datadeck.hh"
#include "numerical_constants.hh"
#include "vehicle.hh"
class AeroDynamics : public FH_module
{
    TRICK_INTERFACE(AeroDynamics);

public:
    explicit AeroDynamics();

    void load_aerotable(const char *filename);

    virtual void init(LaunchVehicle *VehicleIn){};
    virtual void algorithm(LaunchVehicle *VehicleIn);

private:
    Datadeck aerotable; /* ** (--) Aero Deck */
};

#endif  // __AERODYNAMICS_HH__
