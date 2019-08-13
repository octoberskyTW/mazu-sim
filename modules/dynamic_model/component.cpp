#include "component.hh"

void ACT_1st::Actuate(double input_command, double int_step)
{
    ActRateOutput = (Tau * input_command - Tau * Act_prior_state);
    ActAccOutput = (ActRateOutput - Act_prior_rate) / int_step;

    if (fabs(ActAccOutput) > Ang_acc_limit) {
        if (ActAccOutput > 0) {
            ActAccOutput = Ang_acc_limit;
        } else {
            ActAccOutput = -Ang_acc_limit;
        }
        ActRateOutput = Act_prior_rate + ActAccOutput * int_step;
    }

    if (fabs(ActRateOutput) > Ang_rate_limit) {
        if (ActRateOutput > 0) {
            ActRateOutput = Ang_rate_limit;
        } else {
            ActRateOutput = -Ang_rate_limit;
        }
    }

    ActOuptut = Act_prior_state + ActRateOutput * int_step;
    Act_prior_state = ActOuptut;
    Act_prior_rate = ActRateOutput;

    if (fabs(ActOuptut) > Ang_limit) {
        if (ActOuptut > 0) {
            ActOuptut = Ang_limit;
        } else {
            ActOuptut = -Ang_limit;
        }
    }
}

void ACT_1st::set_1st_act_var(double Ang_limit_in, double Ang_rate_limit_in,
                              double Ang_acc_limit_in, double Tau_in)
{
    Ang_limit = Ang_limit_in;
    Ang_rate_limit = Ang_rate_limit_in;
    Ang_acc_limit = Ang_acc_limit_in;
    Tau = Tau_in;
}

void ACT_NO_DYN::Actuate(double input_command, double int_step)
{
    ActOuptut = input_command;
    double unused_var __attribute__((unused)) = int_step;
}

ENG::ENG()
    : VECTOR_INIT(Q, 6),
      MATRIX_INIT(T_N_B, 3, 3),
      VECTOR_INIT(THRUST, 3),
      VECTOR_INIT(ENG_HINGE_POS, 3),
      VECTOR_INIT(ENG_GAMMA_1, 3),
      VECTOR_INIT(ENG_GAMMA_2, 3),
      VECTOR_INIT(ENG_GAMMA_3, 3),
      VECTOR_INIT(ENG_BETA_4, 3),
      VECTOR_INIT(ENG_BETA_5, 3),
      VECTOR_INIT(ENG_BETA_6, 3),
      VECTOR_INIT(FTVC, 3),
      VECTOR_INIT(MTVC, 3)
{
    ENG_GAMMA_1(0) = ENG_BETA_4(0) = 1.0;
    ENG_GAMMA_1(1) = ENG_BETA_4(1) = 0.0;
    ENG_GAMMA_1(2) = ENG_BETA_4(2) = 0.0;

    ENG_GAMMA_2(0) = ENG_BETA_5(0) = 0.0;
    ENG_GAMMA_2(1) = ENG_BETA_5(1) = 1.0;
    ENG_GAMMA_2(2) = ENG_BETA_5(2) = 0.0;

    ENG_GAMMA_3(0) = ENG_BETA_6(0) = 0.0;
    ENG_GAMMA_3(1) = ENG_BETA_6(1) = 0.0;
    ENG_GAMMA_3(2) = ENG_BETA_6(2) = 1.0;
}

void ENG::set_ENG_HINGE_POS(double x, double y, double z)
{
    ENG_HINGE_POS(0) = x;
    ENG_HINGE_POS(1) = y;
    ENG_HINGE_POS(2) = z;
}

void ENG::set_ENG_Dir(int type_in) { type = static_cast<EngType>(type_in); }

void ENG::Allocate_Actuator(int NumAct, enum ActDynType TYPE)
{
    ACT **act_list = new ACT *[NumAct];
    switch (TYPE) {
    case FIRST:
        for (int i = 0; i < NumAct; i++) {
            ACT_1st *ACT_1st_ptr = new ACT_1st;
            act_list[i] = dynamic_cast<ACT *>(ACT_1st_ptr);
            Act_list.push_back(act_list[i]);
        }
        break;

    case NO_DYN:
        for (int i = 0; i < NumAct; i++) {
            ACT_NO_DYN *ACT_NO_DYN_ptr = new ACT_NO_DYN;
            act_list[i] = dynamic_cast<ACT *>(ACT_NO_DYN_ptr);
            Act_list.push_back(act_list[i]);
        }
        break;

    default:
        break;
    }
}

void ENG::calculate_Q(double input_ang, double thrust_in, arma::mat33 TBI,
                      double rp_in, enum EngType TYPE)
{
    THRUST(0) = thrust_in;
    THRUST(1) = 0.0;
    THRUST(2) = 0.0;

    arma::vec3 rho_en_1;

    rho_en_1(0) = rp_in;
    rho_en_1(1) = 0.0;
    rho_en_1(2) = 0.0;

    rho_en_1 = ENG_HINGE_POS -
               rho_en_1;  // hint: Reference ponit to Engine hinge position

    switch (TYPE) {
    case X:
        return;
        break;
    case Y:
        T_N_B(0, 0) = cos(input_ang);
        T_N_B(0, 1) = 0.0;
        T_N_B(0, 2) = -sin(input_ang);
        T_N_B(1, 0) = 0.0;
        T_N_B(1, 1) = 1.0;
        T_N_B(1, 2) = 0.0;
        T_N_B(2, 0) = sin(input_ang);
        T_N_B(2, 1) = 0.0;
        T_N_B(2, 2) = cos(input_ang);
        break;
    case Z:
        T_N_B(0, 0) = cos(input_ang);
        T_N_B(0, 1) = sin(input_ang);
        T_N_B(0, 2) = 0.0;
        T_N_B(1, 0) = -sin(input_ang);
        T_N_B(1, 1) = cos(input_ang);
        T_N_B(1, 2) = 0.0;
        T_N_B(2, 0) = 0.0;
        T_N_B(2, 1) = 0.0;
        T_N_B(2, 2) = 1.0;
        break;
    default:
        break;
    }
    Q(0) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_1);
    Q(1) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_2);
    Q(2) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_3);
    Q(3) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_4);
    Q(4) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_5);
    Q(5) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_6);
}

void ENG::calculate_Q(double input_ang_1, double input_ang_2, double thrust_in,
                      arma::mat33 TBI, double rp_in, enum EngType TYPE)
{
    THRUST(0) = thrust_in;
    THRUST(1) = 0.0;
    THRUST(2) = 0.0;

    arma::vec3 rho_en_1;

    rho_en_1(0) = rp_in;
    rho_en_1(1) = 0.0;
    rho_en_1(2) = 0.0;

    rho_en_1 = ENG_HINGE_POS -
               rho_en_1;  // hint: Reference ponit to Engine hinge position

    switch (TYPE) {
    case YZ:
        T_N_B(0, 0) = cos(input_ang_2) * cos(input_ang_1);
        T_N_B(0, 1) = cos(input_ang_2) * sin(input_ang_1);
        T_N_B(0, 2) = -sin(input_ang_2);
        T_N_B(1, 0) = -sin(input_ang_1);
        T_N_B(1, 1) = cos(input_ang_1);
        T_N_B(1, 2) = 0.0;
        T_N_B(2, 0) = sin(input_ang_2) * cos(input_ang_1);
        T_N_B(2, 1) = sin(input_ang_2) * sin(input_ang_1);
        T_N_B(2, 2) = cos(input_ang_2);
        break;
    default:
        break;
    }
    Q(0) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_1);
    Q(1) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_2);
    Q(2) = dot(trans(TBI) * trans(T_N_B) * THRUST, ENG_GAMMA_3);
    Q(3) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_4);
    Q(4) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_5);
    Q(5) = dot(trans(TBI) * trans(T_N_B) * THRUST,
               -trans(TBI) * cross_matrix(rho_en_1) * ENG_BETA_6);
}

RCS_Thruster::RCS_Thruster()
    : VECTOR_INIT(Q, 6),
      VECTOR_INIT(RHO, 3),
      VECTOR_INIT(RCS_GAMMA_1, 3),
      VECTOR_INIT(RCS_GAMMA_2, 3),
      VECTOR_INIT(RCS_GAMMA_3, 3),
      VECTOR_INIT(RCS_BETA_4, 3),
      VECTOR_INIT(RCS_BETA_5, 3),
      VECTOR_INIT(RCS_BETA_6, 3),
      VECTOR_INIT(Thrust_torque, 3)
{
    this->saved_value = 0;
    RCS_GAMMA_1(0) = RCS_BETA_4(0) = 1.0;
    RCS_GAMMA_1(1) = RCS_BETA_4(1) = 0.0;
    RCS_GAMMA_1(2) = RCS_BETA_4(2) = 0.0;

    RCS_GAMMA_2(0) = RCS_BETA_5(0) = 0.0;
    RCS_GAMMA_2(1) = RCS_BETA_5(1) = 1.0;
    RCS_GAMMA_2(2) = RCS_BETA_5(2) = 0.0;

    RCS_GAMMA_3(0) = RCS_BETA_6(0) = 0.0;
    RCS_GAMMA_3(1) = RCS_BETA_6(1) = 0.0;
    RCS_GAMMA_3(2) = RCS_BETA_6(2) = 1.0;
}

RCS_Thruster::RCS_Thruster(const RCS_Thruster &other)
    : VECTOR_INIT(Q, 6),
      VECTOR_INIT(RHO, 3),
      VECTOR_INIT(RCS_GAMMA_1, 3),
      VECTOR_INIT(RCS_GAMMA_2, 3),
      VECTOR_INIT(RCS_GAMMA_3, 3),
      VECTOR_INIT(RCS_BETA_4, 3),
      VECTOR_INIT(RCS_BETA_5, 3),
      VECTOR_INIT(RCS_BETA_6, 3),
      VECTOR_INIT(Thrust_torque, 3)
{
    this->saved_value = other.saved_value;
    this->dead_zone = other.dead_zone;
    this->hysteresis = other.hysteresis;
    this->thrust = other.thrust;
    this->Q = other.Q;
    this->RHO = other.RHO;
    this->RCS_GAMMA_1 = other.RCS_GAMMA_1;
    this->RCS_GAMMA_2 = other.RCS_GAMMA_2;
    this->RCS_GAMMA_3 = other.RCS_GAMMA_3;
    this->RCS_BETA_4 = other.RCS_BETA_4;
    this->RCS_BETA_5 = other.RCS_BETA_5;
    this->RCS_BETA_6 = other.RCS_BETA_6;
    this->mode = other.mode;
}

RCS_Thruster &RCS_Thruster::operator=(const RCS_Thruster &other)
{
    if (&other == this)
        return *this;

    this->saved_value = other.saved_value;
    this->dead_zone = other.dead_zone;
    this->hysteresis = other.hysteresis;
    this->thrust = other.thrust;
    this->Q = other.Q;
    this->RHO = other.RHO;
    this->RCS_GAMMA_1 = other.RCS_GAMMA_1;
    this->RCS_GAMMA_2 = other.RCS_GAMMA_2;
    this->RCS_GAMMA_3 = other.RCS_GAMMA_3;
    this->RCS_BETA_4 = other.RCS_BETA_4;
    this->RCS_BETA_5 = other.RCS_BETA_5;
    this->RCS_BETA_6 = other.RCS_BETA_6;
    this->mode = other.mode;
    return *this;
}

void RCS_Thruster::calculate_Q(double input, arma::mat33 TBI,
                               enum EngType type_in)
{
    arma::vec3 Thruster_T;
    switch (type_in) {
    case X:
        Thruster_T(0) = thrust;
        Thruster_T(1) = 0.0;
        Thruster_T(2) = 0.0;
        break;

    case Y:
        Thruster_T(0) = 0.0;
        Thruster_T(1) = thrust;
        Thruster_T(2) = 0.0;
        break;

    case Z:
    default:
        Thruster_T(0) = 0.0;
        Thruster_T(1) = 0.0;
        Thruster_T(2) = thrust;
        break;
    }

    Thruster_T = Schmitt_trigger(input) * Thruster_T;

    Q(0) = dot(trans(TBI) * Thruster_T, RCS_GAMMA_1);
    Q(1) = dot(trans(TBI) * Thruster_T, RCS_GAMMA_2);
    Q(2) = dot(trans(TBI) * Thruster_T, RCS_GAMMA_3);
    Q(3) = dot(trans(TBI) * Thruster_T,
               -trans(TBI) * cross_matrix(RHO) * RCS_BETA_4);
    Q(4) = dot(trans(TBI) * Thruster_T,
               -trans(TBI) * cross_matrix(RHO) * RCS_BETA_5);
    Q(5) = dot(trans(TBI) * Thruster_T,
               -trans(TBI) * cross_matrix(RHO) * RCS_BETA_6);
}
/* TBI  =  Transform matrix from Inertia to Body coordinate */
void RCS_Thruster::calculate_Torque_Q(double input, arma::mat33 TBI,
                                      enum EngType type_in)
{
    arma::mat33 unused __attribute__((unused)) = TBI;
    switch (type_in) {
    case X:
        Thrust_torque(0) = Roll_mom_max;
        Thrust_torque(1) = 0.0;
        Thrust_torque(2) = 0.0;
        break;

    case Y:
        Thrust_torque(0) = 0.0;
        Thrust_torque(1) = Pitch_mom_max;
        Thrust_torque(2) = 0.0;
        break;

    case Z:
    default:
        Thrust_torque(0) = 0.0;
        Thrust_torque(1) = 0.0;
        Thrust_torque(2) = Yaw_mom_max;
        break;
    }
    Thrust_torque = Schmitt_trigger(input) * Thrust_torque;

    Q(0) = 0.0;
    Q(1) = 0.0;
    Q(2) = 0.0;
    Q(3) = dot(Thrust_torque, RCS_BETA_4);
    Q(4) = dot(Thrust_torque, RCS_BETA_5);
    Q(5) = dot(Thrust_torque, RCS_BETA_6);
}

int RCS_Thruster::Schmitt_trigger(double in)
{
    // local variable
    int output(0);
    if (in == saved_value)
        return 0;
    // saved_value signal 'trend' (=1 increasing, =-1 decreasing)
    // saved_value signal 'side' (=-1 left, =1 right)
    int trend = sign(in - saved_value);
    int side = sign(saved_value);
    double trigger = (dead_zone * side + hysteresis * trend) / 2.0;

    if (saved_value >= trigger && side == 1) {
        output = 1;
    } else if (saved_value <= trigger && side == -1) {
        output = -1;
    } else {
        output = 0;
    }

    saved_value = in;

    return output;
}

void RCS_Thruster::clear()
{
    this->saved_value = 0;

    return;
}

void RCS_Thruster::set_thruster_var(double in1, double in2, double in3,
                                    int in4)
{
    dead_zone = in1;
    hysteresis = in2;
    thrust = in3;
    mode = static_cast<EngType>(in4);
}

void RCS_Thruster::set_RHO(double in1, double in2, double in3)
{
    RHO(0) = in1;
    RHO(1) = in2;
    RHO(2) = in3;
}

void RCS_Thruster::set_mom_max(double in1, double in2, double in3)
{
    Roll_mom_max = in1;
    Pitch_mom_max = in2;
    Yaw_mom_max = in3;
}
