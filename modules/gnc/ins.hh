#ifndef __INS_HH__
#define __INS_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the INS Module On Board, Error equations based on Zipfel,
Figure 10.27, space stabilized INS with GPS updates) LIBRARY DEPENDENCY:
      ((../src/ins.cpp))
PROGRAMMERS:
      (( (Chun-Hsu Lai) (Chung-Fan Yang) ))
*******************************************************************************/
#include <armadillo>
#include <cassert>
#include <fstream>
#include <functional>
#include <tuple>
#include "time_management.hh"
#include "aux.hh"
#include "cadac_util.hh"
#include "dm_delta_ut.hh"
#include "integrate.hh"
#include "matrix_tool.hh"
#include "cadac_constants.hh"
#include "numerical_constants.hh"

class time_management;

class INS {
  TRICK_INTERFACE(INS);

 public:
  INS();

  INS(const INS& other);

  INS& operator=(const INS& other);

  void initialize();

  /* Internal Getter */
  arma::mat build_WEII();

  void load_location(double lonx, double latx, double alt);
  void load_angle(double yaw, double roll, double pitch);
  void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);
  void calculate_INS_derived_TEI();

  void update(double int_step);

  void set_ideal();
  void set_non_ideal();
  void set_gps_correction(unsigned int index);
  void set_liftoff(unsigned int index);

  /* Input File */

  std::function<arma::vec3()> grab_computed_WBIB;
  std::function<arma::vec3()> grab_error_of_computed_WBIB;

  std::function<arma::vec3()> grab_computed_FSPB;
  std::function<arma::vec3()> grab_error_of_computed_FSPB;

  // std::function<arma::vec3()>   grab_SBII;
  // std::function<arma::vec3()>   grab_VBII;
  // std::function<double()>       grab_dbi;
  // std::function<arma::mat33()>  grab_TBI;
  std::function<int()> grab_gps_update;
  std::function<arma::vec3()> grab_SXH;
  std::function<arma::vec3()> grab_VXH;
  std::function<void()> clear_gps_flag;
  // std::function<arma::mat33()>  grab_TEI;  // Only for testing
  // std::function<arma::vec3()>   grab_SBEE;
  // std::function<arma::vec3()>   grab_VBEE;
  // std::function<double()>   grab_phibdx;
  // std::function<double()>   grab_thtbdx;
  // std::function<double()>   grab_psibdx;
  std::function<arma::vec3()> grab_PHI;
  std::function<arma::vec3()> grab_DELTA_VEL;
  std::function<arma::vec3()> grab_PHI_HIGH;
  std::function<arma::vec3()> grab_PHI_LOW;

  double get_loncx();
  double get_latcx();
  double get_altc();
  double get_dvbec();
  double get_alphacx();
  double get_betacx();
  double get_phibdcx();
  double get_thtbdcx();
  double get_psibdcx();
  double get_thtvdcx();
  double get_phipcx();
  double get_alppcx();

  arma::vec3 get_SBIIC();
  arma::vec3 get_VBIIC();
  arma::vec3 get_WBICI();
  arma::vec3 get_EGRAVI();
  arma::mat33 get_TBIC();
  arma::vec3 get_SBEEC();
  arma::vec3 get_VBEEC();
  arma::mat33 get_TEIC();
  arma::vec4 get_TBDQ();
  arma::mat33 get_TBD();
  arma::mat33 get_TBICI();
  arma::mat33 get_TDCI();
  arma::vec3 get_WBECB();
  arma::vec3 get_ABICB();

  /* Internal Initializers */
  void default_data();

 private:
  arma::vec build_VBEB(double _alpha0x, double _beta0x, double _dvbe);

  time_management* time;

  /* Internal Propagator / Calculators */
  bool GPS_update();

  arma::vec3 calculate_INS_derived_postion(arma::vec3 SBII);
  arma::vec3 calculate_INS_derived_velocity(arma::vec3 VBII);
  arma::vec3 calculate_INS_derived_bodyrate(arma::mat33 TBIC_in, arma::vec3 WBICB);

  arma::mat33 calculate_INS_derived_TBI(arma::mat33 TBI);

  arma::vec3 calculate_gravity_error(double dbi);
  double calculate_INS_derived_dvbe();

  double calculate_INS_derived_alpha(arma::vec3 VBECB);
  double calculate_INS_derived_beta(arma::vec3 VBECB);

  double calculate_INS_derived_alpp(arma::vec3 VBECB);
  double calculate_INS_derived_phip(arma::vec3 VBECB);

  double calculate_INS_derived_psivd(arma::vec3 VBECD_in);
  double calculate_INS_derived_thtvd(arma::vec3 VBECD_in);

  void calculate_INS_derived_euler_angles(arma::mat33 TBD);

  void load_angle();
  void propagate_TBI_Q(double int_step, arma::vec3 WBICB);

  // void error_diagnostics();
  // For test
  arma::vec3 euler_angle(arma::mat33 TBD_in);
  arma::mat33 build_321_rotation_matrix(arma::vec3 angle);

  arma::vec AccelHarmonic(arma::vec3 SBII, double CS[21][21], int n_max,
                          int m_max, arma::mat33 TEIC_in);

  /* Internal Calculators */

  /* Sensors */

  /* Constants */
  arma::mat WEII;     /* *o  (r/s)    Earth's angular velocity (skew-sym) */
  double _WEII[3][3]; /* *o  (r/s)    Earth's angular velocity (skew-sym) */

  /* Propagative Stats */
  arma::vec EVBI;  /* *o  (m/s)   INS vel error */
  double _EVBI[3]; /* *o  (m/s)   INS vel error */

  arma::vec EVBID;  /* *o  (m/s)   INS vel error derivative */
  double _EVBID[3]; /* *o  (m/s)   INS vel error derivative */

  arma::vec ESBI;  /* *o  (m)     INS pos error */
  double _ESBI[3]; /* *o  (m)     INS pos error */

  arma::vec ESBID;  /* *o  (m)     INS pos error derivative */
  double _ESBID[3]; /* *o  (m)     INS pos error derivative */

  arma::vec RICI;  /* *o  (r)     INS tilt error derivative */
  double _RICI[3]; /* *o  (r)     INS tilt error */

  arma::vec RICID;  /* *o  (r)     INS tilt error derivative */
  double _RICID[3]; /* *o  (r)     INS tilt error derivative */

  /* Generating Outputs */
  arma::mat TBIC; /* *o  (--)    Computed T.M. of body wrt inertia coordinate */
  double _TBIC[3][3]; /* *o  (--)    Computed T.M. of body wrt inertia
                         coordinate */

  arma::vec TBIC_Q;  /* *o  (--)    Computed T.M of body wrt inertia coordinate
                        quaterion */
  double _TBIC_Q[4]; /* *o  (--)    Computed T.M of body wrt inertia coordinate
                        quaterion */

  arma::vec TBIDC_Q;  /* *io (--)    Transformation Matrix of body coord wrt
                         inertia coord derivative (Quaternion) */
  double _TBIDC_Q[4]; /* *io (--)    Transformation Matrix of body coord wrt
                         inertia coord derivative (Quaternion) */

  arma::vec
      SBIIC; /* *o  (m)     Computed pos of body wrt inertia reference point*/
  double _SBIIC[3]; /* *o  (m)     Computed pos of body wrt inertia reference
                       point*/

  arma::vec VBIIC;  /* *o  (m/s)   Computed body vel in inertia coor */
  double _VBIIC[3]; /* *o  (m/s)   Computed body vel in inertia coor */

  arma::vec SBEEC;  /* *o   (m)    Computed body position in ECEF */
  double _SBEEC[3]; /* *o   (m)    Computed body position in ECEF */

  arma::vec VBEEC;  /* *o   (m)    Computed body velocity in ECEF */
  double _VBEEC[3]; /* *o   (m)    Computed body velocity in ECEF */

  arma::vec
      WBICI; /* *o  (r/s)   Computed inertial body rate in inert coordinate */
  double _WBICI[3]; /* *o  (r/s)   Computed inertial body rate in inert
                       coordinate */

  arma::vec EGRAVI;  /* *o  (--)    error by gravity */
  double _EGRAVI[3]; /* *o  (--)    error by gravity */

  arma::vec VBECD;  /* *o  (m/s)   Geodetic velocity */
  double _VBECD[3]; /* *o  (m/s)   Geodetic velocity */

  arma::mat TDCI;     /* *o  (--)    Comp T.M. of geodetic wrt inertial */
  double _TDCI[3][3]; /* *o  (--)    Comp T.M. of geodetic wrt inertial */

  arma::mat TEIC;     /* *o  (--)   T.M. of inertia to ECEF */
  double _TEIC[3][3]; /* *o  (--)   T.M. of inertia to ECEF */

  arma::vec INS_I_ATT_ERR;
  double _INS_I_ATT_ERR[3];

  arma::vec TESTV;
  double _TESTV[3];

  arma::vec TMP_old;
  double _TMP_old[3];

  arma::vec VBIIC_old;
  double _VBIIC_old[3];

  arma::vec POS_ERR;
  double _POS_ERR[3];

  arma::vec GRAVGI;  /* *o (m/s2)  Gravity acceleration wrt inertial frame */
  double _GRAVGI[3]; /* *o (m/s2)  Gravity acceleration wrt inertial frame */

  arma::vec TBDQ;
  double _TBDQ[4];

  arma::mat TBD;
  double _TBD[3][3];

  arma::mat TBICI;
  double _TBICI[3][3];

  arma::mat TLI;
  double _TLI[3][3];

  arma::vec VBIIC_old_old;
  double _VBIIC_old_old[3];

  VECTOR(WBECB, 3); /* ** (r/s)  Body angular rate in ECEF */

  VECTOR(ABICB, 3); /* **  (m/s2) Computed body net acceleration */

  double
      dbic; /* *io  (m)     INS computed vehicle distance from Earth center */
  double dvbec; /* *io  (m/s)   Computed body speed wrt earth */

  double alphacx; /* *io  (d)     INS computed angle of attack */
  double betacx;  /* *io  (d)     INS computed sideslip angle */

  double thtvdcx; /* *io  (d)     INS computed vertical flight path angle */
  double psivdcx; /* *io  (d)     INS computed heading angle */

  double alppcx; /* *io  (d)     INS computed total angle of attack */
  double phipcx; /* *io  (d)     INS computed aero roll angle */

  double loncx; /* *io  (d)     INS derived longitude */
  double latcx; /* *io  (d)     INS derived latitude */
  double altc;  /* *io  (m)     INS derived altitude */

  double phibdcx; /* *io  (d)     INS computed geodetic Euler roll angle */
  double thtbdcx; /* *io  (d)     INS computed geodetic Euler pitch angle */
  double psibdcx; /* *io  (d)     INS computed geodetic Euler yaw angle */

  /* Non-propagating Diagnostic Variables */
  /* These can be deleted, but keep to remain trackable in trick simulator */

  double ins_pos_err;     /* *io  (m)     INS absolute postion error */
  double ins_vel_err;     /* *io  (m/s)   INS absolute velocity error */
  double ins_tilt_err;    /* *o  (r)     INS absolute tilt error */
  double ins_pose_err;    /* *io  (m)     INS absolute postion error */
  double ins_vele_err;    /* *io  (m/s)   INS absolute velocity error */
  double ins_phi_err;     /* *o (d)      INS absolute phi angle error */
  double ins_tht_err;     /* *o (d)     INS absolute tht angle error */
  double ins_psi_err;     /* *o  (d)     INS absolute psi angle error */
  unsigned int gpsupdate; /* *o (--)   Set wether use gps correction or not */
  unsigned int liftoff;   /* *o (--)  set LV liftoff */
  unsigned int ideal; /* *o (--)  choose which extropolate algorithm to use */

  unsigned int testindex;
};

#endif  // __INS_HH__
