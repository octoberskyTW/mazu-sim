
#include <cstdlib>
#include "cadac_constants.hh"
#include "integrate.hh"
#include "numerical_constants.hh"
#include "wind.hh"

cad::Wind::Wind(double twind_In, double vertical_wind)
    : has_turbulance(false),
      VECTOR_INIT(VAED, 3),
      VECTOR_INIT(VAEDS, 3),
      VECTOR_INIT(VAEDSD, 3)
{
    this->twind = twind_In;
    this->vertical_wind_speed = vertical_wind;

    VAED.zeros();
    VAEDS.zeros();
    VAEDSD.zeros();
}

void cad::Wind::propagate_VAED(double int_step)
{
    // wind components in geodetic coordinates
    arma::vec3 VAED_RAW;
    VAED_RAW(0) = -vwind * cos(psiwdx * RAD);
    VAED_RAW(1) = -vwind * sin(psiwdx * RAD);
    VAED_RAW(2) = this->vertical_wind_speed;

    // smoothing wind by filtering with time constant 'twind' sec
    arma::vec3 VAEDSD_NEW = (VAED_RAW - VAEDS) * (1 / twind);
    VAEDS = integrate(VAEDSD_NEW, VAEDSD, VAEDS, int_step);
    VAEDSD = VAEDSD_NEW;
    VAED = VAEDS;
}

void cad::Wind::apply_turbulance_if_have(double int_step, double dvba, arma::mat33 TBD, double alppx, double phipx)
{
    if (has_turbulance) {
        arma::vec3 VTAD;

        double value1;
        do {
            unsigned int seed1 = static_cast<unsigned int>(time(NULL));
            value1 = static_cast<double>(rand_r(&seed1)) / RAND_MAX;
        } while (value1 == 0);

        unsigned int seed2 = static_cast<unsigned int>(time(NULL));
        double value2 = static_cast<double>(rand_r(&seed2)) / RAND_MAX;
        gauss_value = (1 / sqrt(int_step)) * sqrt(2 * log(1 / value1)) * cos(2 * PI * value2);

        // filter, converting white gaussian noise into a time sequence of Dryden
        // turbulence velocity variable 'tau'  (One - dimensional cross - velocity Dryden spectrum)
        // integrating first state variable
        double taux1d_new = taux2;
        taux1 = integrate(taux1d_new, taux1d, taux1, int_step);
        taux1d = taux1d_new;

        // integrating second state variable
        double vl = dvba / turb_length;
        double taux2d_new = -vl * vl * taux1 - 2 * vl * taux2 + vl * vl * gauss_value;
        taux2 = integrate(taux2d_new, taux2d, taux2, int_step);
        taux2d = taux2d_new;

        // computing Dryden 'tau' from the two filter states ('2 * PI' changed to 'PI' according to Pritchard)
        tau = turb_sigma * sqrt(1 / (vl * PI)) * (taux1 + sqrt(3.) * taux2 / vl);

        // inserting the turbulence into the load factor plane (aeroballistic 1A - 3A plane)
        // and transforming into body coordinates VTAB = TBA * VTAA; VTAA = [0 0 tau]
        arma::vec3 VTAB;
        VTAB(0) = -tau * sin(alppx * RAD);
        VTAB(1) = tau * sin(phipx * RAD) * cos(alppx * RAD);
        VTAB(2) = tau * cos(phipx * RAD) * cos(alppx * RAD);

        // turbulence in geodetic coordinates
        VTAD = trans(TBD) * VTAB;

        VAED = VTAD + VAEDS;
    }
}

void cad::Wind::set_altitude(double altitude_in_meter) { altitude = altitude_in_meter; }
