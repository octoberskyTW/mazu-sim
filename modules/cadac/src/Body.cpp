#include "Body.hh"

Body::Body() :
    VECTOR_INIT(POSITION, 3),
    VECTOR_INIT(VELOCITY, 3),
    VECTOR_INIT(ACCELERATION, 3),
    VECTOR_INIT(ANGLE, 3),
    VECTOR_INIT(ANGLE_VEL, 3),
    VECTOR_INIT(ANGLE_ACC, 3),
    MATRIX_INIT(M, 6, 6),
    VECTOR_INIT(FORCE, 3),
    VECTOR_INIT(TORQUE, 3),
    VECTOR_INIT(APPILED_TORQUE, 3),
    MATRIX_INIT(TBI, 3, 3),
    VECTOR_INIT(TBI_Q, 4),
    VECTOR_INIT(TBID_Q, 4) {
    M.eye();
}

arma::vec Body::get_POSITION() { return POSITION; }
arma::mat Body::get_TBI() { return TBI; }
arma::vec Body::get_ANGLE_VEL() { return ANGLE_VEL; }
arma::vec Body::get_ANGLE() { return ANGLE; }
arma::vec Body::get_VELOCITY() { return VELOCITY; }
arma::vec Body::get_ACCELERATION() { return ACCELERATION; }
arma::vec Body::get_ANGLE_ACC() { return ANGLE_ACC; }
arma::vec Body::get_FORCE() { return FORCE; }
arma::vec Body::get_TORQUE() { return TORQUE; }
arma::mat Body::get_M() { return M; }
arma::vec Body::get_TBI_Q() { return TBI_Q;}
arma::vec Body::get_TBID_Q() { return TBID_Q;}
unsigned int Body::get_num() { return num; }

void Body::set_POSITION(const arma::vec &PosIn) { POSITION = PosIn; }
void Body::set_VELOCITY(const arma::vec &VelIn) { VELOCITY = VelIn; }
void Body::set_ACCELERATION(const arma::vec &AccIn) { ACCELERATION = AccIn; }
void Body::set_ANGLE(const arma::vec &AngIn) { ANGLE = AngIn; }
void Body::set_ANGLE_VEL(const arma::vec &AngvelIn) { ANGLE_VEL = AngvelIn; }
void Body::set_ANGLE_ACC(const arma::vec &AngaccIn) { ANGLE_ACC = AngaccIn; }
void Body::set_TBI(const arma::mat &TBIIn) { 
    TBI = TBIIn; 
    TBI_Q = Matrix2Quaternion(TBI);
    }
void Body::set_FORCE(const arma::vec &ForceIn) { FORCE = ForceIn; }

Ground::Ground(unsigned int NumIn) {
    for (unsigned int i = 0; i < 3; i++) {
        M(i, i) = 1.0;
        M(i + 3, i + 3) = 1.0;
    }

    type = 0;
    num = NumIn;
    TBI.eye();
    TBI_Q = Matrix2Quaternion(TBI);
    POSITION.zeros();
    VELOCITY.zeros();
    ACCELERATION.zeros();
    ANGLE.zeros();
    ANGLE_VEL.zeros();
    ANGLE_ACC.zeros();
    FORCE.zeros();
    TORQUE.zeros();
}

Mobilized_body::Mobilized_body(unsigned int NumIn, const arma::vec PosIn, const arma::vec VelIn, const arma::vec AccIn, const arma::mat TBIIn
        , const arma::vec ANG_VEL_In, const arma::vec ANG_ACC_In, double MIn, const arma::mat IIn
        , const arma::vec F_In, const arma::vec T_In) {
    
    type = 1;
    num = NumIn;

    POSITION = PosIn;
    VELOCITY = VelIn;
    ACCELERATION = AccIn;
    ANGLE_VEL = ANG_VEL_In;
    ANGLE_ACC = ANG_ACC_In;
    FORCE = F_In;
    APPILED_TORQUE = T_In;
    for (auto i = 0; i < 3; i++) {
        M(i, i) = MIn;
    }

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            M(3 + i, 3 + j) = IIn(i, j);
        }
    }

    TBI = TBIIn;
    TBI_Q = Matrix2Quaternion(TBI);
    ANGLE = euler_angle(TBI);
    POSITION = trans(TBI) * POSITION;
    VELOCITY = trans(TBI) * VELOCITY;
    ACCELERATION = trans(TBI) * ACCELERATION;
    ANGLE_VEL = trans(TBI) * ANGLE_VEL;
    ANGLE_ACC = trans(TBI) * ANGLE_ACC;
    FORCE.zeros();
    TORQUE.zeros();
}

Mobilized_body::Mobilized_body(unsigned int NumIn, const arma::vec PosIn, const arma::vec VelIn, const arma::vec AccIn, const arma::mat TBIIn
        , const arma::vec ANG_VEL_In, const arma::vec ANG_ACC_In, double MIn, const arma::mat IIn
        , const arma::vec F_In) {
    
    type = 1;
    num = NumIn;

    POSITION = PosIn;
    VELOCITY = VelIn;
    ACCELERATION = AccIn;
    ANGLE_VEL = ANG_VEL_In;
    ANGLE_ACC = ANG_ACC_In;
    FORCE = F_In;
    for (auto i = 0; i < 3; i++) {
        M(i, i) = MIn;
    }

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            M(3 + i, 3 + j) = IIn(i, j);
        }
    }

    TBI = TBIIn;
    TBI_Q = Matrix2Quaternion(TBI);
    ANGLE = euler_angle(TBI);
    POSITION = trans(TBI) * POSITION;
    VELOCITY = trans(TBI) * VELOCITY;
    ACCELERATION = trans(TBI) * ACCELERATION;
    ANGLE_VEL = trans(TBI) * ANGLE_VEL;
    ANGLE_ACC = trans(TBI) * ANGLE_ACC;
    FORCE.zeros();
    TORQUE.zeros();
    APPILED_TORQUE.zeros();
}

void Mobilized_body::update(arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn
        , arma::vec ANG_VEL_In) {

    TBI = Quaternion2Matrix(TBI_QIn);

    POSITION = PosIn;
    VELOCITY = VelIn;
    ANGLE = euler_angle(TBI);
    ANGLE_VEL = ANG_VEL_In;

    /* Prepare for orthonormalization */
    double quat_metric = TBI_QIn(0) * TBI_QIn(0) + TBI_QIn(1) * TBI_QIn(1) +
                         TBI_QIn(2) * TBI_QIn(2) + TBI_QIn(3) * TBI_QIn(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */  //  Zipfel p.141
    TBID_Q(0) = 0.5 * (-ANG_VEL_In(0) * TBI_QIn(1) - ANG_VEL_In(1) * TBI_QIn(2) -
                           ANG_VEL_In(2) * TBI_QIn(3)) +
                    50. * erq * TBI_QIn(0);
    TBID_Q(1) = 0.5 * (ANG_VEL_In(0) * TBI_QIn(0) + ANG_VEL_In(2) * TBI_QIn(2) -
                           ANG_VEL_In(1) * TBI_QIn(3)) +
                    50. * erq * TBI_QIn(1);
    TBID_Q(2) = 0.5 * (ANG_VEL_In(1) * TBI_QIn(0) - ANG_VEL_In(2) * TBI_QIn(1) +
                           ANG_VEL_In(0) * TBI_QIn(3)) +
                    50. * erq * TBI_QIn(2);
    TBID_Q(3) = 0.5 * (ANG_VEL_In(2) * TBI_QIn(0) + ANG_VEL_In(1) * TBI_QIn(1) -
                           ANG_VEL_In(0) * TBI_QIn(2)) +
                    50. * erq * TBI_QIn(3);

    TORQUE = APPILED_TORQUE - skew_sym(ANGLE_VEL) * M.submat(3, 3, 5, 5) * ANGLE_VEL;
}
