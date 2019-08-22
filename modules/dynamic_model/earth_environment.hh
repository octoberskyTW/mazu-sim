#ifndef __EARTH_ENVIRONMENT_HH__
#define __EARTH_ENVIRONMENT_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the EarthEnvironment Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/EarthEnvironment.cpp))
PROGRAMMERS:
      (((Chun-Hsu Lai) () () () ))
*******************************************************************************/
#include <armadillo>
#include <functional>
#include "aux.hh"
#include "cadac_constants.hh"
#include "numerical_constants.hh"

#include "env/atmosphere.hh"
#include "env/atmosphere76.hh"
#include "env/atmosphere_nasa2002.hh"
#include "env/atmosphere_weatherdeck.hh"

#include "env/wind.hh"
#include "env/wind_constant.hh"
#include "env/wind_no.hh"
#include "env/wind_tabular.hh"

#include "dm_delta_ut.hh"
#include "time_management.hh"
#include "vehicle.hh"

class EarthEnvironment : public FH_module
{
    TRICK_INTERFACE(EarthEnvironment);

public:
    EarthEnvironment();

    ~EarthEnvironment();

    void atmosphere_use_public();
    void atmosphere_use_nasa();
    void atmosphere_use_weather_deck(char *filename);

    void set_no_wind();
    void set_constant_wind(double dvae, double dir, double twind,
                           double vertical_wind);
    void set_tabular_wind(char *filename, double twind, double vertical_wind);

    void set_no_wind_turbulunce();
    void set_wind_turbulunce(double turb_length, double turb_sigma, double taux1,
                             double taux1d, double taux2, double taux2d,
                             double tau, double gauss_value);

    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);

private:
    time_management *time;

    /* Constants */
    cad::Atmosphere *atmosphere;
    cad::Wind *wind;

    /* Function declaration */
    arma::mat RNP();
    arma::vec AccelHarmonic(arma::vec3 SBII, arma::mat33 TEI, arma::mat33 TGI,
                            int n_max, int m_max);
};

#endif  // __EARTH_ENVIRONMENT_HH__
