#ifndef __PROPULSION_HH__
#define __PROPULSION_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the propulsion Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Propulsion.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "aux.hh"
#include "datadeck.hh"
#include "cadac_constants.hh"
#include "module.hh"
#include "vehicle.hh"

class Propulsion : public Actuator
{
    TRICK_INTERFACE(Propulsion);

public:
    Propulsion();

    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);

    void load_proptable(const char *filename);

    // XXX: get_thrust_state
    std::function<int()> grab_beco_flag;

private:
    /* Rocket Engine state variables */

    Datadeck proptable;

    /* Internal Propagator / Calculators */
    void propagate_delta_v(LaunchVehicle *VehicleIn);

    /* Internal Calculators */
    // double calculate_thrust(double press);
    double calculate_fmassr(LaunchVehicle *VehicleIn);

    arma::vec3 calculate_XCG(LaunchVehicle *VehicleIn);
    arma::mat33 calculate_IBBB(LaunchVehicle *VehicleIn);
};

#endif  // __PROPULSION_HH__
