#ifndef __MATRIX_TOOL_HH__
#define __MATRIX_TOOL_HH__

#include <armadillo>

/** Returns polar from cartesian coordinates.
 * magnitude = POLAR(0,0) = |V|
 * azimuth   = POLAR(1,0) = atan2(V2,V1)
 * elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
 * Example: POLAR = VEC.pol_from_cart();
 */
arma::vec3 pol_from_cart(const arma::vec3 &in);

/// @return the angle between two 3x1 vectors
double angle(const arma::vec3 &VEC1, const arma::vec3 &VEC2);

/// @return skew symmetric matrix of a Vector3
arma::mat33 skew_sym(const arma::vec3 &vec);

/// @return the T.M. of the psivg -> thtvg sequence
arma::mat33 build_psivg_thtvg_TM(const double &psivg, const double &thtvg);

/// @return the Euler T.M. of the psi->tht->phi sequence
arma::mat33 build_psi_tht_phi_TM(const double &psi, const double &tht, const double &phi);

arma::vec4 Matrix2Quaternion(arma::mat33 Matrix_in);
arma::mat33 Quaternion2Matrix(const arma::vec4 &Quaternion_in);
arma::vec4 Quaternion_conjugate(const arma::vec4 &Quaternion_in);
arma::vec4 Quaternion_cross(const arma::vec4 &Quaternion_in1, const arma::vec4 &Quaternion_in2);
void Quaternion2Euler(const arma::vec4 &Quaternion_in, double &Roll, double &Pitch, double &Yaw);
arma::vec4 Euler2Quaternion(const double &Roll, const double &Pitch, const double &Yaw);
int QuaternionMultiply(arma::vec4 &Q_out, arma::vec4 const &Q_in1, arma::vec4 const &Q_in2);
arma::vec4 QuaternionInverse(const arma::vec4 &Q_in);
arma::vec4 QuaternionTranspose(const arma::vec4 &Q_in);
arma::vec3 QuaternionRotation(const arma::vec4 &Q_in, const arma::vec3 &V_in);
arma::mat33 cross_matrix(const arma::vec3 &in);
arma::mat33 TMX(const double &ang);
arma::mat33 TMY(const double &ang);
arma::mat33 TMZ(const double &ang);
arma::vec3 euler_angle(const arma::mat33 &TBD_in);

#define STORE_MAT33(dest, src)          \
    do {                                \
        auto cpy = src;                 \
        trans(cpy);                     \
        double *in = cpy.memptr();      \
        memcpy(dest, in, sizeof(dest)); \
    } while (0);


#define STORE_VEC(dest, src)            \
    do {                                \
        double *in = src.memptr();      \
        memcpy(dest, in, sizeof(dest)); \
    } while (0);

#define GRAB_VAR(x) [&]() { return x; }
#define GRAB_VEC3(x) [&]() { return arma::vec3(x); }
#define GRAB_MAT33(x) [&]() { return arma::mat33((const double *) (&x)); }

int sign(const double &variable);

#endif  // __MATRIX_TOOL_HH__
