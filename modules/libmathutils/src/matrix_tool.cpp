#include <armadillo>
#include <cmath>

#include <armadillo>
#include "mathutils/matrix_tool.hh"
#include "mathutils/numerical_constants.hh"

arma::vec3 pol_from_cart(arma::vec3 in)
{
    double d = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double denom;
    arma::vec3 POLAR;

    double v1 = in(0);
    double v2 = in(1);
    double v3 = in(2);

    d = norm(in);
    azimuth = atan2(v2, v1);

    denom = sqrt(v1 * v1 + v2 * v2);
    if (denom > 0.) {
        elevation = atan2(-v3, denom);
    } else {
        if (v3 > 0)
            elevation = -PI / 2.;
        if (v3 < 0)
            elevation = PI / 2.;
        if (v3 == 0)
            elevation = 0.;
    }

    POLAR(0) = d;
    POLAR(1) = azimuth;
    POLAR(2) = elevation;

    return POLAR;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the angle between two 3x1 vectors
// Example: theta=angle(VEC1,VEC2);
// 010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double angle(arma::vec3 VEC1, arma::vec3 VEC2)
{
    double argument;
    double scalar = dot(VEC1, VEC2);
    double abs1 = norm(VEC1);
    double abs2 = norm(VEC2);

    double dum = abs1 * abs2;
    if (abs1 * abs2 > arma::datum::eps)
        argument = scalar / dum;
    else
        argument = 1.;
    if (argument > 1.)
        argument = 1.;
    if (argument < -1.)
        argument = -1.;

    return acos(argument);
}

///////////////////////////////////////////////////////////////////////////////
// Return skew symmetric matrix of a Vector3
//            | 0 -c  b|        |a|
//            | c  0 -a| <--    |b|
//            |-b  a  0|        |c|
//
// Example: MAT = VEC.skew_sym();
///////////////////////////////////////////////////////////////////////////////
arma::mat33 skew_sym(arma::vec3 vec)
{
    arma::mat33 RESULT;

    RESULT(0, 0) = 0;
    RESULT(1, 0) = vec(2);
    RESULT(2, 0) = -vec(1);
    RESULT(0, 1) = -vec(2);
    RESULT(1, 1) = 0;
    RESULT(2, 1) = vec(0);
    RESULT(0, 2) = vec(1);
    RESULT(1, 2) = -vec(0);
    RESULT(2, 2) = 0;

    return RESULT;
}

////////////////////////////////////////////////////////////////////////////////
// Returns the T.M. of the psivg -> thtvg sequence
//
// 010628 Created by Peter H Zipfel
// 170121 Create Armadillo Version by soncyang
////////////////////////////////////////////////////////////////////////////////
arma::mat33 build_psivg_thtvg_TM(const double &psivg, const double &thtvg)
{
    arma::mat33 AMAT;

    AMAT(0, 2) = -sin(thtvg);
    AMAT(1, 0) = -sin(psivg);
    AMAT(1, 1) = cos(psivg);
    AMAT(2, 2) = cos(thtvg);
    AMAT(0, 0) = (AMAT(2, 2) * AMAT(1, 1));
    AMAT(0, 1) = (-AMAT(2, 2) * AMAT(1, 0));
    AMAT(2, 0) = (-AMAT(0, 2) * AMAT(1, 1));
    AMAT(2, 1) = (AMAT(0, 2) * AMAT(1, 0));
    AMAT(1, 2) = 0.0;

    return AMAT;
}

////////////////////////////////////////////////////////////////////////////////
// Returns the T.M. of the psi->tht->phi sequence
// Euler angle transformation matrix of flight mechanics
//
// 011126 Created by Peter H Zipfel
// 170121 Add Armadillo version
////////////////////////////////////////////////////////////////////////////////
arma::mat33 build_psi_tht_phi_TM(const double &psi, const double &tht, const double &phi)
{
    double spsi = sin(psi);
    double cpsi = cos(psi);
    double stht = sin(tht);
    double ctht = cos(tht);
    double sphi = sin(phi);
    double cphi = cos(phi);

    arma::mat33 AMAT;
    AMAT(0, 0) = cpsi * ctht;
    AMAT(1, 0) = cpsi * stht * sphi - spsi * cphi;
    AMAT(2, 0) = cpsi * stht * cphi + spsi * sphi;
    AMAT(0, 1) = spsi * ctht;
    AMAT(1, 1) = spsi * stht * sphi + cpsi * cphi;
    AMAT(2, 1) = spsi * stht * cphi - cpsi * sphi;
    AMAT(0, 2) = -stht;
    AMAT(1, 2) = ctht * sphi;
    AMAT(2, 2) = ctht * cphi;

    return AMAT;
}

arma::vec4 Matrix2Quaternion(arma::mat33 Matrix_in)
{
    arma::vec4 q_square;
    double q_square_max;
    int j;
    arma::vec4 Quaternion;
    Matrix_in = trans(Matrix_in);
    q_square[0] = fabs(1.0 + Matrix_in(0, 0) + Matrix_in(1, 1) + Matrix_in(2, 2));
    q_square[1] = fabs(1.0 + Matrix_in(0, 0) - Matrix_in(1, 1) - Matrix_in(2, 2));
    q_square[2] = fabs(1.0 - Matrix_in(0, 0) + Matrix_in(1, 1) - Matrix_in(2, 2));
    q_square[3] = fabs(1.0 - Matrix_in(0, 0) - Matrix_in(1, 1) + Matrix_in(2, 2));

    q_square_max = q_square.max();
    j = q_square.index_max();

    switch (j) {
    case 0:
        Quaternion(0) = 0.5 * sqrt(q_square_max);
        Quaternion(1) = 0.25 * (Matrix_in(2, 1) - Matrix_in(1, 2)) / Quaternion[0];
        Quaternion(2) = 0.25 * (Matrix_in(0, 2) - Matrix_in(2, 0)) / Quaternion[0];
        Quaternion(3) = 0.25 * (Matrix_in(1, 0) - Matrix_in(0, 1)) / Quaternion[0];
        break;
    case 1:
        Quaternion(1) = 0.5 * sqrt(q_square_max);
        Quaternion(0) = 0.25 * (Matrix_in(2, 1) - Matrix_in(1, 2)) / Quaternion[1];
        Quaternion(2) = 0.25 * (Matrix_in(1, 0) + Matrix_in(0, 1)) / Quaternion[1];
        Quaternion(3) = 0.25 * (Matrix_in(0, 2) + Matrix_in(2, 0)) / Quaternion[1];
        break;
    case 2:
        Quaternion(2) = 0.5 * sqrt(q_square_max);
        Quaternion(0) = 0.25 * (Matrix_in(0, 2) - Matrix_in(2, 0)) / Quaternion[2];
        Quaternion(1) = 0.25 * (Matrix_in(1, 0) + Matrix_in(0, 1)) / Quaternion[2];
        Quaternion(3) = 0.25 * (Matrix_in(2, 1) + Matrix_in(1, 2)) / Quaternion[2];
        break;
    case 3:
        Quaternion(3) = 0.5 * sqrt(q_square_max);
        Quaternion(0) = 0.25 * (Matrix_in(1, 0) - Matrix_in(0, 1)) / Quaternion[3];
        Quaternion(1) = 0.25 * (Matrix_in(2, 0) + Matrix_in(0, 2)) / Quaternion[3];
        Quaternion(2) = 0.25 * (Matrix_in(2, 1) + Matrix_in(1, 2)) / Quaternion[3];
        break;
    }

    return Quaternion;
}

arma::mat33 Quaternion2Matrix(arma::vec4 Quaternion_in)
{
    arma::mat33 Matrix_out;

    Matrix_out(0, 0) = 2. * (Quaternion_in(0) * Quaternion_in(0) + Quaternion_in(1) * Quaternion_in(1)) - 1.;
    Matrix_out(0, 1) = 2. * (Quaternion_in(1) * Quaternion_in(2) + Quaternion_in(0) * Quaternion_in(3));
    Matrix_out(0, 2) = 2. * (Quaternion_in(1) * Quaternion_in(3) - Quaternion_in(0) * Quaternion_in(2));
    Matrix_out(1, 0) = 2. * (Quaternion_in(1) * Quaternion_in(2) - Quaternion_in(0) * Quaternion_in(3));
    Matrix_out(1, 1) = 2. * (Quaternion_in(0) * Quaternion_in(0) + Quaternion_in(2) * Quaternion_in(2)) - 1.;
    Matrix_out(1, 2) = 2. * (Quaternion_in(2) * Quaternion_in(3) + Quaternion_in(0) * Quaternion_in(1));
    Matrix_out(2, 0) = 2. * (Quaternion_in(1) * Quaternion_in(3) + Quaternion_in(0) * Quaternion_in(2));
    Matrix_out(2, 1) = 2. * (Quaternion_in(2) * Quaternion_in(3) - Quaternion_in(0) * Quaternion_in(1));
    Matrix_out(2, 2) = 2. * (Quaternion_in(0) * Quaternion_in(0) + Quaternion_in(3) * Quaternion_in(3)) - 1.;

    return Matrix_out;
}

arma::vec4 Quaternion_conjugate(arma::vec4 Quaternion_in)
{
    arma::vec4 Quaternion_out;

    Quaternion_out(0) = Quaternion_in(0);
    Quaternion_out(1) = -Quaternion_in(1);
    Quaternion_out(2) = -Quaternion_in(2);
    Quaternion_out(3) = -Quaternion_in(3);

    return Quaternion_out;
}

arma::vec4 Quaternion_cross(arma::vec4 Quaternion_in1, arma::vec4 Quaternion_in2)
{
    arma::vec4 Quaternion_out;

    Quaternion_out(0) = Quaternion_in1(0) * Quaternion_in2(0) - Quaternion_in1(1) * Quaternion_in2(1) - Quaternion_in1(2) * Quaternion_in2(2) - Quaternion_in1(3) * Quaternion_in2(3);
    Quaternion_out(1) = Quaternion_in1(1) * Quaternion_in2(0) + Quaternion_in1(0) * Quaternion_in2(1) - Quaternion_in1(3) * Quaternion_in2(2) + Quaternion_in1(2) * Quaternion_in2(3);
    Quaternion_out(2) = Quaternion_in1(2) * Quaternion_in2(0) + Quaternion_in1(3) * Quaternion_in2(1) + Quaternion_in1(0) * Quaternion_in2(2) - Quaternion_in1(1) * Quaternion_in2(3);
    Quaternion_out(3) = Quaternion_in1(3) * Quaternion_in2(0) - Quaternion_in1(2) * Quaternion_in2(1) + Quaternion_in1(1) * Quaternion_in2(2) + Quaternion_in1(0) * Quaternion_in2(3);

    return Quaternion_out;
}

void Quaternion2Euler(arma::vec4 Quaternion_in, double &Roll, double &Pitch, double &Yaw)
{
    Roll = atan2(2.0 * (Quaternion_in(0) * Quaternion_in(1) + Quaternion_in(2) * Quaternion_in(3)), 1.0 - 2.0 * (Quaternion_in(1) * Quaternion_in(1) + Quaternion_in(2) * Quaternion_in(2)));
    Pitch = asin(2.0 * (Quaternion_in(0) * Quaternion_in(2) - Quaternion_in(3) * Quaternion_in(1)));
    Yaw = atan2(2.0 * (Quaternion_in(0) * Quaternion_in(3) + Quaternion_in(1) * Quaternion_in(2)), 1.0 - 2.0 * (Quaternion_in(2) * Quaternion_in(2) + Quaternion_in(3) * Quaternion_in(3)));
}

arma::vec4 Euler2Quaternion(double Roll, double Pitch, double Yaw)
{
    arma::vec4 Quaternion_out;
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);

    Quaternion_out(0) = cy * cr * cp + sy * sr * sp;
    Quaternion_out(1) = cy * sr * cp - sy * cr * sp;
    Quaternion_out(2) = cy * cr * sp + sy * sr * cp;
    Quaternion_out(3) = sy * cr * cp - cy * sr * sp;

    return Quaternion_out;
}

int QuaternionMultiply(arma::vec4 &Q_out, arma::vec4 const &Q_in1, arma::vec4 const &Q_in2)
{
    //Q_in1.print("Qin1:%5.5lf");
    //Q_in2.print("Qin2:%5.5lf");
    Q_out(0) = Q_in1(0) * Q_in2(0) - Q_in1(1) * Q_in2(1) - Q_in1(2) * Q_in2(2) - Q_in1(3) * Q_in2(3);
    Q_out(1) = Q_in1(0) * Q_in2(1) + Q_in1(1) * Q_in2(0) + Q_in1(2) * Q_in2(3) - Q_in1(3) * Q_in2(2);
    Q_out(2) = Q_in1(0) * Q_in2(2) - Q_in1(1) * Q_in2(3) + Q_in1(2) * Q_in2(0) + Q_in1(3) * Q_in2(1);
    Q_out(3) = Q_in1(0) * Q_in2(3) + Q_in1(1) * Q_in2(2) - Q_in1(2) * Q_in2(1) + Q_in1(3) * Q_in2(0);
    //Q_out.print("Qout2:%5.5lf");
    return 0;
}

arma::vec4 QuaternionInverse(arma::vec4 Q_in)
{
    return Quaternion_conjugate(Q_in) / sqrt(Q_in(0) * Q_in(0) + Q_in(1) * Q_in(1) + Q_in(2) * Q_in(2) + Q_in(3) * Q_in(3));
}

arma::vec4 QuaternionTranspose(arma::vec4 Q_in)
{
    arma::vec4 Q_out;

    Q_out(0) = -Q_in(0);
    Q_out(1) = Q_in(1);
    Q_out(2) = Q_in(2);
    Q_out(3) = Q_in(3);

    return Q_out;
}

arma::vec3 QuaternionRotation(arma::vec4 Q_in, arma::vec3 V_in)
{
    arma::vec4 Q_temp;
    arma::vec3 V_out;

    Q_temp(0) = 0.0;
    Q_temp(1) = V_in(0);
    Q_temp(2) = V_in(1);
    Q_temp(3) = V_in(2);

    Q_temp = Quaternion_cross(Quaternion_conjugate(Q_in), Quaternion_cross(Q_temp, Q_in));

    V_out(0) = Q_temp(1);
    V_out(1) = Q_temp(2);
    V_out(2) = Q_temp(3);

    return V_out;
}

arma::mat33 cross_matrix(arma::vec3 in)
{
    arma::mat33 c_matrix;

    c_matrix(0, 0) = 0.0;
    c_matrix(0, 1) = -in(2);
    c_matrix(0, 2) = in(1);
    c_matrix(1, 0) = in(2);
    c_matrix(1, 1) = 0.0;
    c_matrix(1, 2) = -in(0);
    c_matrix(2, 0) = -in(1);
    c_matrix(2, 1) = in(0);
    c_matrix(2, 2) = 0.0;

    return c_matrix;
}

arma::mat33 TMX(double ang)
{
    arma::mat33 TM;

    TM(0, 0) = 1.0;
    TM(0, 1) = 0.0;
    TM(0, 2) = 0.0;
    TM(1, 0) = 0.0;
    TM(1, 1) = cos(ang);
    TM(1, 2) = sin(ang);
    TM(2, 0) = 0.0;
    TM(2, 1) = -sin(ang);
    TM(2, 2) = cos(ang);

    return TM;
}

arma::mat33 TMY(double ang)
{
    arma::mat33 TM;

    TM(0, 0) = cos(ang);
    TM(0, 1) = 0.0;
    TM(0, 2) = -sin(ang);
    TM(1, 0) = 0.0;
    TM(1, 1) = 1.0;
    TM(1, 2) = 0.0;
    TM(2, 0) = sin(ang);
    TM(2, 1) = 0.0;
    TM(2, 2) = cos(ang);

    return TM;
}

arma::mat33 TMZ(double ang)
{
    arma::mat33 TM;

    TM(0, 0) = cos(ang);
    TM(0, 1) = sin(ang);
    TM(0, 2) = 0.0;
    TM(1, 0) = -sin(ang);
    TM(1, 1) = cos(ang);
    TM(1, 2) = 0.0;
    TM(2, 0) = 0.0;
    TM(2, 1) = 0.0;
    TM(2, 2) = 1.0;

    return TM;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the sign of the variable
// Example: value_signed=value*sign(variable)
// 010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int sign(const double &variable)
{
    int sign = 0;
    if (variable < 0.)
        sign = -1;
    if (variable >= 0.)
        sign = 1;

    return sign;
}

arma::vec3 euler_angle(arma::mat33 TBD_in)
{
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    double mroll = 0;

    double tbd13 = TBD_in(0, 2);
    double tbd11 = TBD_in(0, 0);
    double tbd33 = TBD_in(2, 2);
    double tbd12 = TBD_in(0, 1);
    double tbd23 = TBD_in(1, 2);

    arma::vec3 euler_ang;
    // *geodetic Euler angles
    // computed pitch angle: 'thtbdc'
    // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
    // forced to be a small positive number to prevent division by zero
    if (fabs(tbd13) < 1) {
        thtbdc = asin(-tbd13);
        cthtbd = cos(thtbdc);
    } else {
        thtbdc = PI / 2 * sign(-tbd13);
        cthtbd = EPS;
    }
    // computed yaw angle: 'psibdc'
    double cpsi = tbd11 / cthtbd;
    if (fabs(cpsi) > 1)
        cpsi = 1 * sign(cpsi);
    psibdc = acos(cpsi) * sign(tbd12);

    // computed roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if (fabs(cphi) > 1)
        cphi = 1 * sign(cphi);

    // selecting the Euler roll angle of flight mechanics (not for thtbdc=90 or
    // =-90deg)
    if (mroll == 0 || mroll == 1)
        // roll feedback for right side up
        phibdc = acos(cphi) * sign(tbd23);
    else if (mroll == 2)
        // roll feedback for inverted flight
        phibdc = acos(-cphi) * sign(-tbd23);

    euler_ang(0) = phibdc;
    euler_ang(1) = thtbdc;
    euler_ang(2) = psibdc;

    return euler_ang;
}