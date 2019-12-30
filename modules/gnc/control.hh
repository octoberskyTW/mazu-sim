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
#include "dataflow_binding_sheipa.hh"

class PID_ctrl {
      TRICK_INTERFACE(PID_ctrl);
public:
      PID_ctrl(const double dt_in, const double kp_in, const double ki_in);
      PID_ctrl(const double dt_in, const double kp_in, const double ki_in, const double kd_in);
      void calculate(const double error, double *output);

private:
      enum PID_TYPE {
            PI = 0,
            PID
      };
      double pre_error;  // PID controller input error value
      double integral;
      double derivative;
      double dt;
      double kp;
      double ki;
      double kd;
      enum PID_TYPE ctrl_type;
};

class Control {
  TRICK_INTERFACE(Control);

 public:
  Control();

  void initialize();

  void control(double int_step, navi_egress_packet_t *navi_packet);

  void set_IBBB0(double in1, double in2, double in3);
  void set_IBBB1(double in1, double in2, double in3);
  void set_controller_var(double in1, double in2, double in3, double in4,
                          double in5, double in6, double in7);
  void set_NO_CONTROL();
  void set_engnum(double in);
  void set_reference_point(double in);

  double get_theta_a_cmd();
  double get_theta_b_cmd();
  double get_theta_c_cmd();
  double get_theta_d_cmd();
  double get_throttle_cmd();

  enum CONTROL_TYPE {
    NO_CONTROL = 0,
    CONTROL_ON
  };


  void calculate_xcg_thrust(double int_step);

 private:

  void Euler_Angle_Control(const double roll_cmd, const double pitch_cmd, const double yaw_cmd);
  void Velocity_Control(const double Vx_cmd, const double Vy_cmd, const double Vz_cmd);
  void update_commmand();

  enum CONTROL_TYPE maut; /* *io (--)     maut=|mauty|mautp| see table */

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
  double eng_num;

  PID_ctrl *Vx_PID;
  PID_ctrl *Acclx_PID;
  PID_ctrl *Roll_rate_PID;
  PID_ctrl *Pitch_rate_PID;
  PID_ctrl *Yaw_rate_PID;

  double roll_kp;
  double pitch_kp;
  double yaw_kp;
  double vz_kp;
  double vy_kp;
  arma::vec ATT_CMD;
  double _ATT_CMD[3];
  double throttle_cmd;
  arma::mat DecoupleMat;
  double _DecoupleMat[3][4];
  arma::vec ACT_CMD;
  double _ACT_CMD[4];

  navi_egress_packet_t *navi_data;
};

#endif  // __CONTROL_HH__
