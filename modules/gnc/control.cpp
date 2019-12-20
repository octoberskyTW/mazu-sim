#include "control.hh"

Control::Control()
    : VECTOR_INIT(IBBB0, 3),
      VECTOR_INIT(IBBB1, 3),
      VECTOR_INIT(IBBB2, 3),
      VECTOR_INIT(ATT_CMD, 3)
{
}

void Control::initialize()
{
    fmasse = -mdot * 0.05;
    theta_a_cmd = 0.0;
    theta_b_cmd = 0.0;
    theta_c_cmd = 0.0;
    theta_d_cmd = 0.0;
}

void Control::control(double int_step)
{
    calculate_xcg_thrust(int_step);

    switch (maut) {
    case NO_CONTROL:
        return;
        break;
    
    case CONTROL_ON:
        Velocity_Control(0.0, 0.0, 0.0);
        break;
    default:
        break;
    }
}

void Control::calculate_xcg_thrust(double int_step)
{
    fmasse += mdot * int_step;
    mass_ratio = fmasse / fmass0;
    xcg = xcg_0 + (xcg_1 - xcg_0) * mass_ratio;
    thrust = isp * mdot * AGRAV / eng_num;
    IBBB2 = IBBB0 + (IBBB1 - IBBB0) * mass_ratio;
    vmass = vmass0 - fmasse;
}

double Control::get_theta_a_cmd() { return theta_a_cmd; }
double Control::get_theta_b_cmd() { return theta_b_cmd; }
double Control::get_theta_c_cmd() { return theta_c_cmd; }
double Control::get_theta_d_cmd() { return theta_d_cmd; }

void Control::set_controller_var(double in1, double in2, double in3, double in4,
                                 double in5, double in6, double in7)
{
    vmass0 = in1;
    mdot = in2;
    fmass0 = in3;
    xcg_1 = in4;
    xcg_0 = in5;
    isp = in6;
    fmasse = in7;
}
void Control::set_IBBB0(double in1, double in2, double in3)
{
    IBBB0(0) = in1;
    IBBB0(1) = in2;
    IBBB0(2) = in3;
}
void Control::set_IBBB1(double in1, double in2, double in3)
{
    IBBB1(0) = in1;
    IBBB1(1) = in2;
    IBBB1(2) = in3;
}
void Control::set_NO_CONTROL() { maut = NO_CONTROL; }
void Control::set_engnum(double in) { eng_num = in; }
void Control::set_reference_point(double in) { reference_point = in; }


void Control::Euler_Angle_Control(const double roll_cmd, const double pitch_cmd, const double yaw_cmd) {
    arma::vec3 WBICB = grab_computed_WBIB();
    double thtbdcx = grab_thtbdcx();
    double phibdcx = grab_phibdcx();
    double psibdcx = grab_psibdcx();

    // Roll ctrl
    double error = (roll_cmd - phibdcx * RAD) * roll_kp - WBICB(0);
    Roll_rate_PID->calculate(error, _ATT_CMD);

    // Pitch ctrl
    error = (pitch_cmd - thtbdcx * RAD) * pitch_kp - WBICB(1);
    Pitch_rate_PID->calculate(error, (_ATT_CMD + 1));

    // Yaw ctrl
    error = (yaw_cmd - psibdcx * RAD) * yaw_kp - WBICB(2);
    Yaw_rate_PID->calculate(error, (_ATT_CMD + 2));
    return;
}

void Control::Velocity_Control(const double Vx_cmd, const double Vy_cmd, const double Vz_cmd) {
    arma::vec3 VBECD = grab_VBECD();
    arma::vec3 FSPCB = grab_FSPCB();

    double vy_ctrl_output = (Vy_cmd - VBECD(1)) * vy_kp;
    double vz_ctrl_output = (Vz_cmd - VBECD(2)) * vz_kp;
    Euler_Angle_Control(0.0, vy_ctrl_output, vz_ctrl_output);

    double vx_out(0.);
    double accx_out(0.);

    Vx_PID->calculate(Vx_cmd - VBECD(0), &vx_out);
    Acclx_PID->calculate(vx_out - FSPCB(0), &accx_out);

    throttle_cmd = accx_out + 0.6;
    return;
}

PID_ctrl::PID_ctrl(const double dt_in, const double kp_in, const double ki_in) {
    dt = dt_in;
    kp = kp_in;
    ki = ki_in;
    ctrl_type = PI;
}

PID_ctrl::PID_ctrl(const double dt_in, const double kp_in, const double ki_in, const double kd_in) {
    dt = dt_in;
    kp = kp_in;
    ki = ki_in;
    kd = kd_in;
    ctrl_type = PID;
}

void PID_ctrl::calculate(const double error, double *output) {
    switch (ctrl_type) {
        case PI:
            integral += error * dt;
            *output = kp * error + ki * integral;
            break;
        
        case PID:
            integral += error * dt;
            derivative = (error - pre_error) / dt;
            *output = kp * error + kd * derivative + ki * integral;
            pre_error = error;
            break;
        default:
            return;
            break;
    }
}