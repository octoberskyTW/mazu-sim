#ifndef __CONTROL_HH__
#define __CONTROL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the CONTROL Module On Board)
LIBRARY DEPENDENCY:
      ((../src/control.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) (Chun-Hsu Lai) () () ))
*******************************************************************************/
#include <armadillo>
#include "aux.hh"
#include "datadeck.hh"
#include "env/atmosphere76.hh"
#include "env/atmosphere_nasa2002.hh"
#include "matrix_tool.hh"
#include "cadac_constants.hh"
#include "integrate.hh"

class Control {
  TRICK_INTERFACE(Control);

 public:
  Control();

  void initialize();

  void control(double int_step);
  void set_close_loop_pole(double in1, double in2);
  void set_factor(double in1, double in2);
  void set_feedforward_gain(double in1);

  void set_aero_coffe(double in1, double in2, double in3);
  void set_IBBB0(double in1, double in2, double in3);
  void set_IBBB1(double in1, double in2, double in3);
  void set_controller_var(double in1, double in2, double in3, double in4,
                          double in5, double in6, double in7);
  void set_NO_CONTROL();
  void set_acc_control();
  void set_engnum(double in);

  double get_theta_a_cmd();
  double get_theta_b_cmd();
  double get_theta_c_cmd();
  double get_theta_d_cmd();
  void load_aerotable(const char* filename);
  void atmosphere_use_nasa();
  void atmosphere_use_public();

  void set_ancomx(double in);
  void set_alcomx(double in);

  enum CONTROL_TYPE {
    NO_CONTROL = 0,
    S2_PITCH_DOWN_I,
    S2_PITCH_DOWN_II,
    S2_ROLL_CONTROL,
    S3_PITCH_DOWN,
    S2_AOA,
    S3_AOA,
    ACC_CONTROL_ON
  };

  std::function<int()> grab_thrust_state;
  std::function<double()> grab_dvbec;
  std::function<double()> grab_thtvdcx;
  std::function<double()> grab_thtbdcx;
  std::function<double()> grab_phibdcx;
  std::function<double()> grab_psibdcx;
  std::function<double()> grab_alphacx;
  std::function<double()> grab_altc;

  std::function<double()> grab_qqcx;
  std::function<double()> grab_rrcx;
  std::function<double()> grab_phipcx;
  std::function<double()> grab_alppcx;

  std::function<arma::vec3()> grab_FSPCB;

  std::function<arma::vec3()> grab_computed_WBIB;
  std::function<arma::vec4()> grab_TBDQ;
  std::function<arma::mat33()> grab_TBD;
  std::function<arma::mat33()> grab_TBICI;
  std::function<arma::mat33()> grab_TBIC;
  std::function<arma::vec3()> grab_WBECB;
  std::function<arma::vec3()> grab_ABICB;

  double get_delecx();
  double get_delrcx();

  void calculate_xcg_thrust(double int_step);

 private:
  cad::Atmosphere* atmosphere;

  double control_normal_accel(double ancomx_in, double int_step);
  double control_yaw_accel(double alcomx_in, double int_step);
  void aerodynamics_der();

  double delecx; /* *io (d)      Pitch command deflection */  // n
  double delrcx; /* *io (d)      Yaw command deflection */    // n

  enum CONTROL_TYPE maut; /* *io (--)     maut=|mauty|mautp| see table */
  int mfreeze;   /* *io (--)     =0:Unfreeze; =1:Freeze; increment for more */
  double waclp;  /* *io (r/s)    Nat freq of accel close loop complex pole */
  double zaclp;  /* *io (--)     Damping of accel close loop complex pole */
  double paclp;  /* *io (--)     Close loop real pole */
  double yyd;    /* *io (m/s2)   Yaw feed-forward derivative variable */
  double yy;     /* *io (m/s)    Yaw feed-forward integration variable */
  double zzd;    /* *io (m/s2)   Pitch feed-forward derivative variable */
  double zz;     /* *io (m/s)    Pitch feed-forward integration variable */
  double alcomx_actual; /* *io (--)     Later accel com limited by 'betalimx' */
  double ancomx_actual; /* *io (--)     Normal accel com limited by 'alplimx' */
  arma::vec GAINFP;  /* *io (--)     Feedback gains of pitch accel controller */
  double _GAINFP[3]; /* *io (--)     Feedback gains of pitch accel controller */
  double gainp;  /* *io (s2/m)   Proportional gain in pitch acceleration loop */
  double gainl;  /* *io (--)     Gain in lateral acceleration loop */
  double gkp;    /* *io (s)      Gain of roll rate feedback */
  double gkphi;  /* *io (--)     Gain of roll angle feedback */
  double isetc2; /* *io (--)     Flag to print freeze variables */
  double wacly;  /* *io (r/s)    Nat freq of accel close loop pole, yaw */
  double zacly;  /* *io (--)     Damping of accel close loop pole, yaw */
  double pacly;  /* *io (--)     Close loop real pole, yaw */
  double gainy;  /* *io (--)     Gain in lateral acceleration loop */
  arma::vec GAINFY;  /* *io (--)     Feedback gains of yaw accel controller */
  double _GAINFY[3]; /* *io (--)     Feedback gains of yaw accel controller */
  double factwaclp; /* *io (--)     Factor to mod 'waclp': waclp*(1+factwacl) */
  double factwacly; /* *io (--)     Factor to mod 'wacly': wacly*(1+factwacl) */
  double alcomx;    /* *io (--)     Lateral (horizontal) acceleration command */
  double ancomx;    /* *io (--)     Pitch (normal) acceleration command */

  double fmasse;
  double mdot;
  double fmass0;
  double xcg_0;
  double xcg_1;
  double isp;
  double vmass;
  double vmass0;

  arma::vec IBBB0;
  double _IBBB0[3];

  arma::vec IBBB1;
  double _IBBB1[3];

  arma::vec IBBB2;
  double _IBBB2[3];

  double theta_a_cmd;
  double theta_b_cmd;
  double theta_c_cmd;
  double theta_d_cmd;
  double lx;

  double xcg;
  double thrust;
  double mass_ratio;

  double reference_point;
  double d;

  // Aerodynamics def()
  Datadeck aerotable;

  double dla;
  double dlde;
  double dma;
  double dmq;
  double dmde;
  double dyb;
  double dydr;
  double dnb;
  double dnr;
  double dndr;
  double dllp;
  double dllda;
  double dnd;
  double cla;
  double clde;
  double cyb;
  double cydr;
  double cllda;
  double cllp;
  double cma;
  double cmde;
  double cmq;
  double cnb;
  double cndr;
  double cnr;
  double cn0;
  // diagnostics
  double stmarg_yaw;
  double stmarg_pitch;
  double realp1;
  double realp2;
  double wnp;
  double zetp;
  double rpreal;
  double realy1;
  double realy2;
  double wny;
  double zety;
  double ryreal;
  double refa;
  double refd;
  double xcp;
  double pdynmc;
  double vmach;

  double eng_num;
};

#endif  // __CONTROL_HH__
