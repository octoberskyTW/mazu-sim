#ifndef __WIND_HH__
#define __WIND_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (wind model interface definition)
LIBRARY DEPENDENCY:
      ((../../src/env/wind.cpp))
*******************************************************************************/
#include <armadillo>
#include <aux.hh>

namespace cad
{
class Wind
{
    TRICK_INTERFACE(cad__Wind);

public:
    char name[256];

    Wind(double twind_In, double vertical_wind);

    virtual ~Wind() {}

    virtual void set_altitude(double altitude_in_meter);
    virtual void propagate_VAED(double int_step);
    virtual void apply_turbulance_if_have(double int_step, double dvba, arma::mat33 TBD, double alppx, double phipx);

    virtual double get_speed_of_wind() { return vwind; }
    virtual double get_direction_of_wind() { return psiwdx; }

    virtual arma::vec3 get_VAED() { return VAED; }
    virtual arma::vec3 get_VAEDS() { return VAEDS; }
    virtual arma::vec3 get_VAEDSD() { return VAEDSD; }

    virtual void enable_turbulance(double turb_length_In, double turb_sigma_In,
                                   double taux1_In, double taux1d_In,
                                   double taux2_In, double taux2d_In,
                                   double tau_In, double gauss_value_In)
    {
        has_turbulance = true;

        this->turb_length = turb_length_In;
        this->turb_sigma = turb_sigma_In;
        this->taux1 = taux1_In;
        this->taux1d = taux1d_In;
        this->taux2 = taux2_In;
        this->taux2d = taux2d_In;
        this->tau = tau_In;
        this->gauss_value = gauss_value_In;
    }
    virtual void disable_turbulance() { has_turbulance = false; }

protected:
    bool has_turbulance;

    double twind;    /* *o (s)          Wind smoothing time constant - sec */
    double altitude; /* *o (m)          Current Altitude */

    double vertical_wind_speed; /* *o (m/s)        Vertical air speed (pos.down) - m/s */

    double vwind;  /* *o (m/s)        Wind Speed */
    double psiwdx; /* *o (d)          Wind direction from north */

private:
    arma::vec VAED;  /* *o (m/s)        Smoothed wind velocity in geodetic coord */
    double _VAED[3]; /* *o (m/s)        Smoothed wind velocity in geodetic coord */

    arma::vec VAEDS;  /* *o (m/s)        Smoothed wind velocity in geodetic coord - m/s */
    double _VAEDS[3]; /* *o (m/s)        Smoothed wind velocity in geodetic coord - m/s */

    arma::vec VAEDSD;  /* *o (m/s)        Smoothed wind velocity derivative - m/s */
    double _VAEDSD[3]; /* *o (m/s)        Smoothed wind velocity derivative - m/s */

    double turb_length; /* *o (m)          Turbulence correlation length - m*/
    double turb_sigma;  /* *o (m/s)        Turbulence magnitude (1sigma) - m/s*/
    double taux1;       /* *o (--)         First turbulence state variable - ND*/
    double taux1d;      /* *o (--)         First turbulence state variable - ND*/
    double taux2;       /* *o (1/s)        First turbulence state variable derivative - 1/s*/
    double taux2d;      /* *o (1/s)        First turbulence state variable derivative - 1/s*/
    double tau;         /* *o (m/s)        Turblence velocity component in load factor plane - m/s*/
    double gauss_value; /* *o (--)         White Gaussian noise - ND*/
};
}  // namespace cad

#endif  // __WIND_HH__
