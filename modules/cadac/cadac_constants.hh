/**
 * \file global_constants.h
 *
 * \brief Defines all global constant parameters for HYPER simulation
 */

#ifndef __CADAC_CONSTANTS_HH__
#define __CADAC_CONSTANTS_HH__

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/// global constants to be used in the simulation

/// physical constants
/**@{*/
double const REARTH = 6378000;  ///< mean earth radius - m
double const WEII1 = 1.20824e-7;
double const WEII2 = -2.944e-9;
double const WEII3 = 7.292115e-5;  ///< angular rotation of earth - rad/s
double const AGRAV =
    9.80665;                            ///< standard value of gravity acceleration - m/s^2
double const G = 6.673e-11;             ///< universal gravitational constant - Nm^2/kg^2
double const EARTH_MASS = 5.973332e24;  ///< mass of the earth - kg
double const GM =
    3.9860044e14;  ///< gravitational parameter=G*EARTH_MASS - m^3/s^2
double const C20 =
    -4.8416685e-4;  ///< second degree zonal gravitational coefficient - ND
double const FLATTENING =
    1 / 298.257223563;  ///< flattening of the Earth (WGS84) - ND  3.3352810665e-3
double const SMAJOR_AXIS =
    6378137;  ///< semi-major axis of Earth's ellipsoid (WGS84) - m
double const GW_CLONG =
    0;                           ///< Greenwich celestial longitude at start of simulation - rad
double const RGAS = 287.053;     ///< ideal gas constant - J/(K*kg)=N*m/(K*kg)
double const KBOLTZ = 1.38e-23;  ///< Boltzmann's constant - Ws/K

/**@}*/

/// conversion factors
/**@{*/
double const RAD = 0.0174532925199432;  ///< conversion factor deg->rad
double const DEG = 57.2957795130823;    ///< conversion factor rad->deg
double const FOOT = 3.280834;           ///< conversion factor m->ft
double const NMILES = 5.399568e-4;      ///< conversion factor m->nm
                                        /**@}*/

/* verify the following array sizes.
 * If too small, dynamic memory allocation will fail!
 */
int const NROUND6 = 600;  ///< size of 'round6' module-variable array
int const NHYPER = 950;   ///< size of 'hyper' module-variable array
int const NEVENT = 20;    ///< max number of events
int const NVAR = 50;      ///< max number of variables to be input at every event
int const NMARKOV = 20;   ///< max number of Markov noise variables
/**@}*/

const double DM_sec2r = 0.000072722052;    /* second to radian */
const double DM_arcsec2r = 4.848136811e-6; /* arcsecond to radian */

const double GMSB = 0.02 * RAD;          // ADIS16488 gyro MSB = 0.02d/s
const double AMSB = 0.8 * 0.001 * 9.81;  // ADIS16488 acc MSB = 0.8mg
const double GLSB = 0.01 * (1.0 / 65536.0);
const double ALSB = 0.01 * (1.0 / 65536.0);

/************************************************************************/
/** GPS constellation **/
const int N_SBF = 5;
const int MAX_SAT = 32;
const int MAX_CHAR = 100;
const int N_DWRD_SBF = 10;
const int CA_SEQ_LEN = 1023;
const int N_DWRD = (N_SBF + 1) * N_DWRD_SBF;
const int EPHEM_ARRAY_SIZE = 13;
const int SECONDS_IN_HOUR = 3600;
const int MAX_CHAN = 12;
const int SECONDS_IN_DAY = 86400;
const double SPEED_OF_LIGHT = 2.99792458e8;
const double LAMBDA_L1 = 0.190293672798365;
const double POW2_M5 = 0.03125;
const double POW2_M19 = 1.907348632812500e-6;
const double POW2_M29 = 1.862645149230957e-9;
const double POW2_M31 = 4.656612873077393e-10;
const double POW2_M33 = 1.164153218269348e-10;
const double POW2_M43 = 1.136868377216160e-13;
const double POW2_M55 = 2.775557561562891e-17;
const double POW2_M50 = 8.881784197001252e-016;
const double POW2_M30 = 9.313225746154785e-010;
const double POW2_M27 = 7.450580596923828e-009;
const double POW2_M24 = 5.960464477539063e-008;
const double WGS84_ECCENTRICITY = 0.0818191908426;
/****************************************************************/
const double TVC_DSP_RESOLUTION = 0.01;
const int TVC_ROTATION_LIMIT_CNT = 800;
#endif  //  __CADAC_CONSTANTS_HH__
