#ifndef __RCS_FC_HH__
#define __RCS_FC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the RCS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rcs_fc.cpp))
PROGRAMMERS:
      (( (Chun-Hsu Lai) ))
*******************************************************************************/
#include <armadillo>
#include "aux.hh"
#include "cadac_constants.hh"

class RCS_FC {
  TRICK_INTERFACE(RCS_FC);

 public:
  RCS_FC();
  RCS_FC(const RCS_FC& other);

  RCS_FC& operator=(const RCS_FC& other);

  void initialize();

  void actuate();

  enum RCS_TYPE { NO_RCS = 0, ON_OFF_RCS = 2 };

  enum RCS_MODE {
    NO_CONTROL = 0,
    ALL_GEODETIC_EULUR_ANGLE_CONTROL = 1,
    THRUST_VECTOR_DIRECTION_AND_ROLL_ANGLE_CONTROL = 2,
    INCIDENCE_AND_ROLL_ANGLE_CONTROL = 3,
    GEODETIC_YAW_ANGLE_CONTROL = 4
  };

  void enable_rcs();
  void disable_rcs();

  bool isEnabled();

  void set_mode(int in);
  enum RCS_MODE get_rcs_mode();

  std::function<arma::vec3()> grab_UTBC;
  std::function<arma::vec3()> grab_WBICB;
  std::function<double()> grab_alphacomx;
  std::function<double()> grab_betacomx;

  std::function<double()> grab_alphacx;
  std::function<double()> grab_betacx;

  std::function<double()> grab_thtbdcx;
  std::function<double()> grab_psibdcx;
  std::function<double()> grab_phibdcx;

  double get_e_roll();
  double get_e_pitch();
  double get_e_yaw();

  /* Input File */
  void set_rcs_tau(double in);
  void set_thtbdcomx(double in);
  void set_psibdcomx(double in);
  void set_phibdcomx(double in);

 private:
  /* Internal Getter */

  /* Internal Initializers */
  void default_data();

  /* Internal Propagator / Calculators */

  /* Internal Calculators */

  /* Routing references */

  /* Input */
  double thtbdcomx; /* *o  (d)    Pitch angle command */
  double psibdcomx; /* *o  (d)    Yaw angle command */
  double phibdcomx; /* *o  (d)    Roll angle command */

  /* Constants */
  enum RCS_TYPE rcs_type; /* *o  (--)   Attitude control, see RCS_TYPE */
  enum RCS_MODE rcs_mode; /* *o  (--)   Attitude control, see RCS_MODE */

  double rcs_tau; /* *o  (s)    Slope of the switching function */

  // double  acc_gain;        [> *io  (N*s2/m) Acceleration gain of side
  // thrusters <] double  side_force_max;  [> *io  (N)    Maximum side force of
  // thruster <]

  /* Propagative Stats */

  /* Generating Outputs */
  double e_roll;  /* *o  (--)   Roll error signal */
  double e_pitch; /* *o  (--)   Pitch error signal */
  double e_yaw;   /* *o  (--)   Yaw error signal */

  /* Non-propagating Diagnostic Variables */
  /* These can be deleted, but keep to remain trackable in trick simulator */
};

#endif  // __RCS_FC_HH__
