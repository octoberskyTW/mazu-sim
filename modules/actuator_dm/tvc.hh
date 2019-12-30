#ifndef __TVC_HH__
#define __TVC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Tvc.cpp))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include <tuple>
#include <vector>
#include "component.hh"
#include "vehicle.hh"
#include "aux.hh"
#include "cadac_constants.hh"
#include "signal_process.hh"

class TVC : public FH_module {
  TRICK_INTERFACE(TVC);

 public:
  TVC();

  virtual void init(LaunchVehicle *VehicleIn);

  virtual void algorithm(LaunchVehicle *VehicleIn);

  std::function<double()> grab_theta_a_cmd;
  std::function<double()> grab_theta_b_cmd;
  std::function<double()> grab_theta_c_cmd;
  std::function<double()> grab_theta_d_cmd;
};

#endif  // __TVC_HH__
