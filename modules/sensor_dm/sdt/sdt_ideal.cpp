#include <cassert>
#include <iostream>
#include "aux.hh"
#include "cadac_constants.hh"
#include "integrate.hh"
#include "matrix_tool.hh"
#include "numerical_constants.hh"
#include "sdt_ideal.hh"


SDT_ideal::SDT_ideal(unsigned int kl_in) { k_limit = kl_in; }

void SDT_ideal::init(LaunchVehicle *VehicleIn)
{
    Sensor_var *S;
    S = VehicleIn->Sensor;

    S->k = 0;
    S->PHI.zeros();
    S->WBISB_old.zeros();
    S->DELTA_ALPHA.zeros();
    S->DELTA_VEL.zeros();
}
void SDT_ideal::algorithm(LaunchVehicle *VehicleIn)
{
    Sensor_var *S;
    S = VehicleIn->Sensor;
    double int_step = VehicleIn->dt;
    S->WBISB = S->WBICB;
    S->FSPSB = S->FSPCB;

    if (S->k == k_limit || S->k == 1) {
        S->k = 1;
        S->PHI.zeros();
        S->ALPHA.zeros();
        S->DELTA_VEL.zeros();
    }

    // Coning compensation algorithm

    arma::vec3 zo(arma::fill::zeros);

    arma::vec3 tmp = S->WBISB;

    S->DELTA_ALPHA = S->WBISB * int_step;  // Strapdown Analytics 7.1.1.1.1-5

    S->ALPHA += S->DELTA_ALPHA;  // Strapdown Analytics 7.1.1.1.1-5

    S->DELTA_ALPHA_old = S->DELTA_ALPHA;

    S->WBISB_old = tmp;

    S->PHI += S->DELTA_ALPHA;  // + 0.5 * DELTA_BETA;

    S->FSPSB = build_321_rotation_matrix(S->DELTA_ALPHA) *
               S->FSPSB;  // Strapdown 7.2.2-3

    S->DELTA_VEL +=
        S->FSPSB * int_step;  // + integrate(cross2, cross2_old, zo, int_step);
    S->FSPSB_old = S->FSPSB;
    S->k++;
}

arma::mat33 SDT_ideal::build_321_rotation_matrix(arma::vec3 angle)
{
    arma::mat33 TM;
    TM(0, 0) = cos(angle(2)) * cos(angle(1));
    TM(0, 1) = sin(angle(2)) * cos(angle(1));
    TM(0, 2) = -sin(angle(1));
    TM(1, 0) = (cos(angle(2)) * sin(angle(1)) * sin(angle(0))) -
               (sin(angle(2)) * cos(angle(0)));
    TM(1, 1) = (sin(angle(2)) * sin(angle(1)) * sin(angle(0))) +
               (cos(angle(2)) * cos(angle(0)));
    TM(1, 2) = cos(angle(1)) * sin(angle(0));
    TM(2, 0) = (cos(angle(2)) * sin(angle(1)) * cos(angle(0))) +
               (sin(angle(2)) * sin(angle(0)));
    TM(2, 1) = (sin(angle(2)) * sin(angle(1)) * cos(angle(0))) -
               (cos(angle(2)) * sin(angle(0)));
    TM(2, 2) = cos(angle(1)) * cos(angle(0));

    return TM;
}
