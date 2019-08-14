#ifndef __MATRIX_TOOL_HH__
#define __MATRIX_TOOL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Matrix utility functions)
LIBRARY DEPENDENCY:
      ((../../src/matrix/utility.cpp))
*******************************************************************************/
#include <armadillo>

/** Returns polar from cartesian coordinates.
 * magnitude = POLAR(0,0) = |V|
 * azimuth   = POLAR(1,0) = atan2(V2,V1)
 * elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
 * Example: POLAR = VEC.pol_from_cart();
 */
arma::vec3 pol_from_cart(arma::vec3 in);

/// @return the angle between two 3x1 vectors
double angle(arma::vec3 VEC1, arma::vec3 VEC2);

/// @return skew symmetric matrix of a Vector3
arma::mat33 skew_sym(arma::vec3 vec);

/// @return the T.M. of the psivg -> thtvg sequence
arma::mat33 build_psivg_thtvg_TM(const double &psivg, const double &thtvg);

/// @return the Euler T.M. of the psi->tht->phi sequence
arma::mat33 build_psi_tht_phi_TM(const double &psi, const double &tht, const double &phi);

arma::vec4 Matrix2Quaternion(arma::mat33 Matrix_in);
arma::mat33 Quaternion2Matrix(arma::vec4 Quaternion_in);
arma::vec4 Quaternion_conjugate(arma::vec4 Quaternion_in);
arma::vec4 Quaternion_cross(arma::vec4 Quaternion_in1, arma::vec4 Quaternion_in2);
void Quaternion2Euler(arma::vec4 Quaternion_in, double &Roll, double &Pitch, double &Yaw);
arma::vec4 Euler2Quaternion(double Roll, double Pitch, double Yaw);
arma::vec4 QuaternionMultiply(arma::vec4 Q_in1, arma::vec4 Q_in2);
arma::vec4 QuaternionInverse(arma::vec4 Q_in);
arma::vec4 QuaternionTranspose(arma::vec4 Q_in);
arma::vec3 QuaternionRotation(arma::vec4 Q_in, arma::vec3 V_in);
arma::mat33 cross_matrix(arma::vec3 in);
arma::mat33 TMX(double ang);
arma::mat33 TMY(double ang);
arma::mat33 TMZ(double ang);

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
