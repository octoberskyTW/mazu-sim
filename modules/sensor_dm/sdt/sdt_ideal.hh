#ifndef __SDT_IDEAL_HH__
#define __SDT_IDEAL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Sensor Data Transport module)
LIBRARY DEPENDENCY:
      ((../../src/SDT_ideal.cpp))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "sdt.hh"

class SDT_ideal : public SDT
{
    TRICK_INTERFACE(SDT_ideal);

public:
    SDT_ideal(unsigned int kl_in);

    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);

private:
    arma::mat33 build_321_rotation_matrix(arma::vec3 angle);
};

#endif  //  __SDT_IDEAL_HH__
