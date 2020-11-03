#include <algorithm>
#include "earth_environment.hh"
#include "cadac_util.hh"
#include "aux.hh"

#include "env/atmosphere.hh"
#include "env/atmosphere76.hh"
#include "env/atmosphere_nasa2002.hh"
#include "env/atmosphere_weatherdeck.hh"

#include "env/wind.hh"
#include "env/wind_constant.hh"
#include "env/wind_no.hh"
#include "env/wind_tabular.hh"
#include "earth_coeff.hh"

EarthEnvironment::EarthEnvironment() : time(time_management::Instance())
{
    atmosphere = NULL;
    wind = NULL;
}

EarthEnvironment::~EarthEnvironment()
{
    if (atmosphere)
        delete atmosphere;
    if (wind)
        delete wind;
}

void EarthEnvironment::init(LaunchVehicle *VehicleIn)
{
    VehicleIn->Env->dvba = VehicleIn->DM->_dvbe;
    VehicleIn->Env->TEI = RNP();
    VehicleIn->Env->WEIE_skew(0, 1) = -WEII3;
    VehicleIn->Env->WEIE_skew(1, 0) = WEII3;
    VehicleIn->Env->WEIE(2) = WEII3;
}

void EarthEnvironment::atmosphere_use_public()
{
    atmosphere = new cad::Atmosphere76();
}

void EarthEnvironment::atmosphere_use_nasa()
{
    atmosphere = new cad::Atmosphere_nasa2002();
}

void EarthEnvironment::atmosphere_use_weather_deck(char *filename)
{
    atmosphere = new cad::Atmosphere_weatherdeck(filename);
}

void EarthEnvironment::set_no_wind() { wind = new cad::Wind_No(); }

void EarthEnvironment::set_constant_wind(double dvae, double dir, double twind,
                                         double vertical_wind)
{
    wind = new cad::Wind_Constant(dvae, dir, twind, vertical_wind);
}

void EarthEnvironment::set_tabular_wind(char *filename, double twind,
                                        double vertical_wind)
{
    wind = new cad::Wind_Tabular(filename, twind, vertical_wind);
}

void EarthEnvironment::set_no_wind_turbulunce()
{
    if (wind)
        wind->disable_turbulance();
}

void EarthEnvironment::set_wind_turbulunce(double turb_length,
                                           double turb_sigma, double taux1,
                                           double taux1d, double taux2,
                                           double taux2d, double tau,
                                           double gauss_value)
{
    if (wind)
        wind->enable_turbulance(turb_length, turb_sigma, taux1, taux1d, taux2,
                                taux2d, tau, gauss_value);
}

void EarthEnvironment::algorithm(LaunchVehicle *VehicleIn)
{
    EarthEnvironment_var *E;
    DM_var *D;
    E = VehicleIn->Env;
    D = VehicleIn->DM;
    double int_step = VehicleIn->dt;

    /******************************/

    AccelHarmonic(E->GRAVG, D->SBEE, E->TEI, 20, 20);

    // flight conditionsnorm(VBAD)
    E->VBAB =
        D->TBI * trans(E->TEI) * (D->VBEE - trans(D->TDE) * wind->get_VAED());
    E->dvba = norm(E->VBAB);

    atmosphere->set_altitude(D->alt);
    wind->set_altitude(D->alt);
    wind->propagate_VAED(int_step);
    wind->apply_turbulance_if_have(int_step, E->dvba, D->TBD, D->alppx, D->phipx);

    // mach number
    E->vmach = fabs(E->dvba / atmosphere->get_speed_of_sound());

    // dynamic pressure
    E->pdynmc = 0.5 * atmosphere->get_density() * E->dvba * E->dvba;
    E->tempc = atmosphere->get_temperature_in_kelvin() + 273.16;
    E->gravg = norm(E->GRAVG);
    E->press = atmosphere->get_pressure();
    E->rho = atmosphere->get_density();
    E->tempk = atmosphere->get_temperature_in_kelvin();
    E->VAED = wind->get_VAED();
    /* Timing issue */
    E->TEI = RNP();  // Calculate Rotation-Nutation-Precession (ECI to ECEF) Matrix
}

/* Rotation-Nutation-Precession transfor Matrix (ECI to ECEF) */
arma::mat EarthEnvironment::RNP()
{
    /* double WEIE = 7.2921151467E-5; */
    time_util::UTC_TIME utc_caldate;
    time_util::GPS_TIME tmp_gps;
    unsigned char i;
    double UTC, UT1;
    arma::mat33 M_rotation;
    arma::mat33 M_nutation;
    arma::mat33 M_precession;
    // arma::mat33 M_nut_n_pre;
    double t, t2, t3, thetaA, zetaA, zA;
    double epsilonA, epsilonAP, F, D, omega;
    double sideral_time(0);
    double L, La, gamma, delta_psi, delta_epsilon;
    double dUT1;
    double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;
    double s_delta_psi, c_delta_psi, s_epsilonA, c_epsilonA, s_epsilonAP,
        c_epsilonAP;
    double s2_half_delta_psi, s_delta_epsilon, c_delta_epsilon;
    int index;

    /*------------------------------------------------------------------ */
    /* --------------- Interface to Global Variable ------------*/
    /*------------------------------------------------------------------ */

    /*------------------------------------------------------------- */
    /*--------------- Calculate the UTC time -------------*/
    /*------------------------------------------------------------- */
    /* GPS time converted from GPS format to YYYY/MM/DD/MM/SS */
    /* Correction for time difference btwn GPS & UTC is applied implicitly */
    /***to prevent change origin gpstime data****/
    tmp_gps = time->get_gpstime();
    /*********************************************/
    utc_caldate = time_util::UTC_TIME(tmp_gps); /* leap second is considered */

    UTC = utc_caldate.get_hour() * 3600.0 + utc_caldate.get_min() * 60.0 +
          utc_caldate.get_sec();

    index = static_cast<int32_t>(time->get_modified_julian_date().get_mjd() -
                                 DM_UT1_UT_BASE_MJD); /* get dUT1 = Ut1 - UT from table*/
    if ((index >= 0) &&
        (index < MAX_DM_UT1_UT_INDEX)) {
        dUT1 = DM_UT1_UT[index];
    } else {
        dUT1 = -0.008853655954360; /* mean value during 19760519~20120811, FSW: dUT1
                                  = 0.4; */
    }

    UT1 = UTC + dUT1;

    /*----------------------------------------------------------- */
    /*-------------- Precession Matrix  ------------------- */
    /*----------------------------------------------------------- */
    t = (time->get_modified_julian_date().get_jd() - 2451545.0) /
        36525.0; /* J2000.5 : Julian Day is 2451545, unit in day */

    t2 = t * t;
    t3 = t * t * t;

    thetaA = 2004.3109 * t - 0.42665 * t2 - 0.041833 * t3; /* unit : arcsec */
    zetaA = 2306.2181 * t + 0.30188 * t2 + 0.017998 * t3;  /* unit : arcsec */
    zA = 2306.2181 * t + 1.09468 * t2 + 0.018203 * t3;     /* unit : arcsec */

    s_thetaA = sin(thetaA * DM_arcsec2r);
    c_thetaA = cos(thetaA * DM_arcsec2r);
    s_zetaA = sin(zetaA * DM_arcsec2r);
    c_zetaA = cos(zetaA * DM_arcsec2r);
    s_zA = sin(zA * DM_arcsec2r);
    c_zA = cos(zA * DM_arcsec2r);

    M_precession(0, 0) = -s_zA * s_zetaA + c_zA * c_thetaA * c_zetaA;
    M_precession(0, 1) = -s_zA * c_zetaA - c_zA * c_thetaA * s_zetaA;
    M_precession(0, 2) = -c_zA * s_thetaA;
    M_precession(1, 0) = c_zA * s_zetaA + s_zA * c_thetaA * c_zetaA;
    M_precession(1, 1) = c_zA * c_zetaA - s_zA * c_thetaA * s_zetaA;
    M_precession(1, 2) = -s_zA * s_thetaA;
    M_precession(2, 0) = s_thetaA * c_zetaA;
    M_precession(2, 1) = -s_zetaA * s_thetaA;
    M_precession(2, 2) = c_thetaA;

    /*------------------------------------------------------ */
    /*------------- Nutation Matrix ---------------------*/
    /*------------------------------------------------------ */

    /*** Original Algorithm ***/
    /* IAU-80 nutation */
    epsilonA = (84381.448 - 46.815 * t - 0.00059 * t2 + 0.001813 * t3) *
               DM_arcsec2r; /* unit: radian */
    L = (485866.7330 + 1717915922.633 * t + 31.310 * t2 + 0.064 * t3) *
        DM_arcsec2r; /* unit: radian */
    La = (1287099.804 + 129596581.2240 * t - 0.5770 * t2 - 0.012 * t3) *
         DM_arcsec2r; /* unit: radian */
    F = (335778.8770 + 1739527263.137 * t - 13.257 * t2 + 0.011 * t3) *
        DM_arcsec2r; /* unit: radian */
    D = (1072261.307 + 1602961601.328 * t - 6.8910 * t2 + 0.019 * t3) *
        DM_arcsec2r; /* unit: radian */
    omega = (450160.2800 - 6962890.539 * t + 7.4550 * t2 + 0.008 * t3) *
            DM_arcsec2r; /* unit: radian */

    delta_psi = 0.0;
    delta_epsilon = 0.0;

    for (i = 0; i < 106; i++) {
        gamma = NUTATION_COEFF[i][0] * L + NUTATION_COEFF[i][1] * La +
                NUTATION_COEFF[i][2] * F + NUTATION_COEFF[i][3] * D +
                NUTATION_COEFF[i][4] * omega; /* unit: radian */
        delta_psi += (NUTATION_COEFF[i][5] + NUTATION_COEFF[i][6] * t) *
                                    sin(gamma); /* unit: arcsec */
        delta_epsilon += (NUTATION_COEFF[i][7] + NUTATION_COEFF[i][8] * t) *
                                cos(gamma); /* unit: arcsec */
    }

    epsilonAP = epsilonA + delta_epsilon * DM_arcsec2r; /* unit: radian */

    s_delta_psi = sin(delta_psi * DM_arcsec2r);
    c_delta_psi = cos(delta_psi * DM_arcsec2r);
    s2_half_delta_psi =
        sin(delta_psi / 2.0 * DM_arcsec2r) * sin(delta_psi / 2.0 * DM_arcsec2r);
    s_epsilonA = sin(epsilonA);
    c_epsilonA = cos(epsilonA);
    s_delta_epsilon = sin(delta_epsilon * DM_arcsec2r);
    c_delta_epsilon = cos(delta_epsilon * DM_arcsec2r);
    s_epsilonAP = sin(epsilonAP);
    c_epsilonAP = cos(epsilonAP);

    M_nutation(0, 0) = c_delta_psi;
    M_nutation(0, 1) = -s_delta_psi * c_epsilonA;
    M_nutation(0, 2) = -s_delta_psi * s_epsilonA;
    M_nutation(1, 0) = c_epsilonAP * s_delta_psi;
    M_nutation(1, 1) =
        c_delta_epsilon - 2. * s2_half_delta_psi * c_epsilonA * c_epsilonAP;
    M_nutation(1, 2) =
        -s_delta_epsilon - 2. * s2_half_delta_psi * s_epsilonA * c_epsilonAP;
    M_nutation(2, 0) = s_epsilonAP * s_delta_psi;
    M_nutation(2, 1) =
        s_delta_epsilon - 2. * s2_half_delta_psi * c_epsilonA * s_epsilonAP;
    M_nutation(2, 2) =
        c_delta_epsilon - 2. * s2_half_delta_psi * s_epsilonA * s_epsilonAP;

    /*----------------------------------------------------------- */
    /*-------- Matrice of Nutation * Precession --------*/
    /*----------------------------------------------------------- */
    // M_nut_n_pre = M_nutation * M_precession;

    /*----------------------------------------------------------- */
    /*------------------- Rotation Matrix --------------------*/
    /*----------------------------------------------------------- */
    sideral_time =
        UT1 + (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3);
    sideral_time = sideral_time * DM_sec2r +
                    delta_psi * cos(epsilonA) * DM_arcsec2r; /* unit: radian */

    M_rotation(0, 0) = cos(sideral_time);
    M_rotation(0, 1) = sin(sideral_time);
    M_rotation(0, 2) = 0.0;
    M_rotation(1, 0) = -sin(sideral_time);
    M_rotation(1, 1) = cos(sideral_time);
    M_rotation(1, 2) = 0.0;
    M_rotation(2, 0) = 0.0;
    M_rotation(2, 1) = 0.0;
    M_rotation(2, 2) = 1.0;

    return std::move(M_rotation * (M_nutation * M_precession));
} /* End of dm_RNP() */

/*******************************************************************************
 *   AccelHarmonic
 *
 *   Purpose:
 *
 *       Computes the acceleration of a launch vehicle due to
 *           -The Earth's harmonic gravity field
 *
 *   Input:
 *       r           Launch Vehicle position vector in ECI coordinate
 *       E           Transformation matrix from ECI to ECEF coordinate
 *       GM          Gravitational coefficient
 *       R_ref       Earth mean Radius
 *       CS          Spherical harmonic coefficients (un-normalized)
 *       n_max       Maxium degree
 *       m_max       Maxium orger (m_max<=n_max; m_max=0 for zonals, only)
 *
 *   Output:
 *       acc         Gravitational acceleration
 *
 ********************************************************************************/
void EarthEnvironment::AccelHarmonic(arma::vec &GRAV_ACC, const arma::vec &SBEE_In,
                                          const arma::mat &TEI, int n_max,
                                          int m_max)
{
    /* Local variables */
    int n, m;               /* Loop counters */
    double r_sqr, rho, Fac; /* Auxiliary quantities */
    double x0, y0, z0;      /* Normalized coordinates */
    double ax(0.0), ay(0.0), az(0.0);      /* Acceleration vector */
    double C, S;            /* Gravitational coefficients */

    double V[N_JGM3 + 2][N_JGM3 + 2]; /* Harmonic functions
                                                          */
    double W[N_JGM3 + 2][N_JGM3 + 2]; /* work array
                                                         (0..n_max+1,0..n_max+1)
                                                       */

    /* Auxiliary quantities */
    r_sqr = dot(SBEE_In, SBEE_In); /* Square of distance */
    rho = SMAJOR_AXIS * SMAJOR_AXIS / r_sqr;
    x0 = SMAJOR_AXIS * SBEE_In(0) / r_sqr; /* Normalized  */
    y0 = SMAJOR_AXIS * SBEE_In(1) / r_sqr; /* coordinates */
    z0 = SMAJOR_AXIS * SBEE_In(2) / r_sqr;

    /*******************************
   *
   * Evaluate harmonic functions
   *   V_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * cos(m*lambda)
   * and
   *   W_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * sin(m*lambda)
   * up to degree and order n_max+1
   *
   ********************************/

    /* Calculate zonal terms V(n,0); set W(n,0)=0.0 */

    V[0][0] = SMAJOR_AXIS / sqrt(r_sqr);
    W[0][0] = 0.0;

    V[1][0] = z0 * V[0][0];
    W[1][0] = 0.0;

    for (n = 2; n <= n_max + 1; n++) {
        V[n][0] =
            ((2 * n - 1) * z0 * V[n - 1][0] - (n - 1) * rho * V[n - 2][0]) / n;
        W[n][0] = 0.0;
    }

    /* Calculate tesseral and sectorial terms */
    for (m = 1; m <= m_max + 1; m++) {
        /* Calculate V(m,m) .. V(n_max+1,m) */

        V[m][m] = (2 * m - 1) * (x0 * V[m - 1][m - 1] - y0 * W[m - 1][m - 1]);
        W[m][m] = (2 * m - 1) * (x0 * W[m - 1][m - 1] + y0 * V[m - 1][m - 1]);

        if (m <= n_max) {
            V[m + 1][m] = (2 * m + 1) * z0 * V[m][m];
            W[m + 1][m] = (2 * m + 1) * z0 * W[m][m];
        }

        for (n = m + 2; n <= n_max + 1; n++) {
            V[n][m] =
                ((2 * n - 1) * z0 * V[n - 1][m] - (n + m - 1) * rho * V[n - 2][m]) /
                (n - m);
            W[n][m] =
                ((2 * n - 1) * z0 * W[n - 1][m] - (n + m - 1) * rho * W[n - 2][m]) /
                (n - m);
        }
    }

    for (m = 0; m <= m_max; m++) {
        for (n = m; n <= n_max; n++) {
            if (m == 0) {
                C = GRAV_COEFF_JGM3[n][0]; /* = C_n,0 */
                ax -= C * V[n + 1][1];
                ay -= C * W[n + 1][1];
                az -= (n + 1) * C * V[n + 1][0];
            } else {
                C = GRAV_COEFF_JGM3[n][m];     /* = C_n,m */
                S = GRAV_COEFF_JGM3[m - 1][n]; /* = S_n,m */
                Fac = 0.5 * (n - m + 1) * (n - m + 2);
                ax += +0.5 * (-C * V[n + 1][m + 1] - S * W[n + 1][m + 1]) +
                      Fac * (+C * V[n + 1][m - 1] + S * W[n + 1][m - 1]);
                ay += +0.5 * (-C * W[n + 1][m + 1] + S * V[n + 1][m + 1]) +
                      Fac * (-C * W[n + 1][m - 1] + S * V[n + 1][m - 1]);
                az += (n - m + 1) * (-C * V[n + 1][m] - S * W[n + 1][m]);
            }
        }
    }

    /* Body-fixed acceleration */
    GRAV_ACC(0) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * ax;
    GRAV_ACC(1) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * ay;
    GRAV_ACC(2) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * az;

    GRAV_ACC = trans(TEI) * GRAV_ACC;

    return;
} /* end of AccelHarmonic */
