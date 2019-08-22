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

/********************************************************************
 *
 * Earth gravity field JGM3
 * Gravitational coefficients C, S are efficiently stored in a single
 * array CS. The lower triangle matrix CS holds the non-sectorial C
 * coefficients C_n,m (n!=m). Sectorial C coefficients C_n,n are the
 * diagonal elements of CS and the upper triangular matrix stores
 * the S_n,m (m!=0) coefficients in columns, for the same degree n.
 * Mapping of CS to C, S is achieved through
 * C_n,m = CS(n,m), S_n,m = CS(m-1,n)
 *
 *********************************************************************/

#define N_JGM3 20

double CS_JGM3[N_JGM3 + 1][N_JGM3 + 1] = {
    { 1.000000e+00, 0.000000e+00, 1.543100e-09, 2.680119e-07, -4.494599e-07,
      -8.066346e-08, 2.116466e-08, 6.936989e-08, 4.019978e-08, 1.423657e-08,
      -8.128915e-08, -1.646546e-08, -2.378448e-08, 2.172109e-08, 1.443750e-08,
      4.154186e-09, 1.660440e-08, -1.427822e-08, -1.817656e-08, 7.160542e-11,
      2.759192e-09 },
    { 0.000000e+00, 0.000000e+00, -9.038681e-07, -2.114024e-07, 1.481555e-07,
      -5.232672e-08, -4.650395e-08, 9.282314e-09, 5.381316e-09, -2.228679e-09,
      -3.057129e-09, -5.097360e-09, 1.416422e-09, -2.545587e-09, -1.089217e-10,
      -1.045474e-09, 7.856272e-10, 2.522818e-10, 3.427413e-10, -1.008909e-10,
      3.216826e-10 },
    { -1.082627e-03, -2.414000e-10, 1.574536e-06, 1.972013e-07, -1.201129e-08,
      -7.100877e-09, 1.843134e-10, -3.061150e-09, -8.723520e-10, -5.633921e-10,
      -8.989333e-10, -6.863521e-10, 9.154575e-11, 3.005522e-10, 5.182512e-11,
      3.265044e-11, -4.271981e-11, 1.297841e-11, -4.278803e-12, -1.190759e-12,
      3.778260e-11 },
    { 2.532435e-06, 2.192799e-06, 3.090160e-07, 1.005589e-07, 6.525606e-09,
      3.873005e-10, -1.784491e-09, -2.636182e-10, 9.117736e-11, 1.717309e-11,
      -4.622483e-11, -2.677798e-11, 9.170517e-13, -2.960682e-12, -3.750977e-12,
      1.116419e-12, 5.250141e-12, 2.159727e-12, 1.105860e-13, -3.556436e-13,
      -1.178441e-12 },
    { 1.619331e-06, -5.087253e-07, 7.841223e-08, 5.921574e-08, -3.982396e-09,
      -1.648204e-09, -4.329182e-10, 6.397253e-12, 1.612521e-11, -5.550919e-12,
      -3.122269e-12, 1.982505e-12, 2.033249e-13, 1.214266e-12, -2.217440e-13,
      8.637823e-14, -1.205563e-14, 2.923804e-14, 1.040715e-13, 9.006136e-14,
      -1.823414e-14 },
    { 2.277161e-07, -5.371651e-08, 1.055905e-07, -1.492615e-08, -2.297912e-09,
      4.304768e-10, -5.527712e-11, 1.053488e-11, 8.627743e-12, 2.940313e-12,
      -5.515591e-13, 1.346234e-13, 9.335408e-14, -9.061871e-15, 2.365713e-15,
      -2.505252e-14, -1.590014e-14, -9.295650e-15, -3.743268e-15, 3.176649e-15,
      -5.637288e-17 },
    { -5.396485e-07, -5.987798e-08, 6.012099e-09, 1.182266e-09, -3.264139e-10,
      -2.155771e-10, 2.213693e-12, 4.475983e-13, 3.814766e-13, -1.846792e-13,
      -2.650681e-15, -3.728037e-14, 7.899913e-15, -9.747983e-16, -3.193839e-16,
      2.856094e-16, -2.590259e-16, -1.190467e-16, 8.666599e-17, -8.340023e-17,
      -8.899420e-19 },
    { 3.513684e-07, 2.051487e-07, 3.284490e-08, 3.528541e-09, -5.851195e-10,
      5.818486e-13, -2.490718e-11, 2.559078e-14, 1.535338e-13, -9.856184e-16,
      -1.052843e-14, 1.170448e-15, 3.701523e-16, -1.095673e-16, -9.074974e-17,
      7.742869e-17, 1.086771e-17, 4.812890e-18, 2.015619e-18, -5.594661e-18,
      1.459810e-18 },
    { 2.025187e-07, 1.603459e-08, 6.576542e-09, -1.946358e-10, -3.189358e-10,
      -4.615173e-12, -1.839364e-12, 3.429762e-13, -1.580332e-13, 7.441039e-15,
      -7.011948e-16, 2.585245e-16, 6.136644e-17, 4.870630e-17, 1.489060e-17,
      1.015964e-17, -5.700075e-18, -2.391386e-18, 1.794927e-18, 1.965726e-19,
      -1.128428e-19 },
    { 1.193687e-07, 9.241927e-08, 1.566874e-09, -1.217275e-09, -7.018561e-12,
      -1.669737e-12, 8.296725e-13, -2.251973e-13, 6.144394e-14, -3.676763e-15,
      -9.892610e-17, -1.736649e-17, 9.242424e-18, -4.153238e-18, -6.937464e-20,
      3.275583e-19, 1.309613e-19, 1.026767e-19, -1.437566e-20, -1.268576e-20,
      -6.100911e-21 },
    { 2.480569e-07, 5.175579e-08, -5.562846e-09, -4.195999e-11, -4.967025e-11,
      -3.074283e-12, -2.597232e-13, 6.909154e-15, 4.635314e-15, 2.330148e-15,
      4.170802e-16, -1.407856e-17, -2.790078e-19, -6.376262e-20, -1.849098e-19,
      3.595115e-20, -2.537013e-21, 4.480853e-21, 4.348241e-22, 1.197796e-21,
      -1.138734e-21 },
    { -2.405652e-07, 9.508428e-09, 9.542030e-10, -1.409608e-10, -1.685257e-11,
      1.489441e-12, -5.754671e-15, 1.954262e-15, -2.924949e-16, -1.934320e-16,
      -4.946396e-17, 9.351706e-18, -9.838299e-20, 1.643922e-19, -1.658377e-20,
      2.905537e-21, 4.983891e-22, 6.393876e-22, -2.294907e-22, 6.437043e-23,
      6.435154e-23 },
    { 1.819117e-07, -3.068001e-08, 6.380398e-10, 1.451918e-10, -2.123815e-11,
      8.279902e-13, 7.883091e-15, -4.131557e-15, -5.708254e-16, 1.012728e-16,
      -1.840173e-18, 4.978700e-19, -2.108949e-20, 2.503221e-20, 3.298844e-21,
      -8.660491e-23, 6.651727e-24, 5.110031e-23, -3.635064e-23, -1.311958e-23,
      1.534228e-24 },
    { 2.075677e-07, -2.885131e-08, 2.275183e-09, -6.676768e-11, -3.452537e-13,
      1.074251e-12, -5.281862e-14, 3.421269e-16, -1.113494e-16, 2.658019e-17,
      4.577888e-18, -5.902637e-19, -5.860603e-20, -2.239852e-20, -6.914977e-23,
      -6.472496e-23, -2.741331e-23, 2.570941e-24, -1.074458e-24, -4.305386e-25,
      -2.046569e-25 },
    { -1.174174e-07, -9.997710e-09, -1.347496e-09, 9.391106e-11, 3.104170e-13,
      3.932888e-13, -1.902110e-14, 2.787457e-15, -2.125248e-16, 1.679922e-17,
      1.839624e-18, 7.273780e-20, 4.561174e-21, 2.347631e-21, -7.142240e-22,
      -2.274403e-24, -2.929523e-24, 1.242605e-25, -1.447976e-25, -3.551992e-26,
      -7.473051e-28 },
    { 1.762727e-08, 6.108862e-09, -7.164511e-10, 1.128627e-10, -6.013879e-12,
      1.293499e-13, 2.220625e-14, 2.825477e-15, -1.112172e-16, 3.494173e-18,
      2.258283e-19, -1.828153e-21, -6.049406e-21, -5.705023e-22, 1.404654e-23,
      -9.295855e-24, 5.687404e-26, 1.057368e-26, 4.931703e-27, -1.480665e-27,
      2.400400e-29 },
    { -3.119431e-08, 1.356279e-08, -6.713707e-10, -6.451812e-11, 4.698674e-12,
      -9.690791e-14, 6.610666e-15, -2.378057e-16, -4.460480e-17, -3.335458e-18,
      -1.316568e-19, 1.643081e-20, 1.419788e-21, 9.260416e-23, -1.349210e-23,
      -1.295522e-24, -5.943715e-25, -9.608698e-27, 3.816913e-28, -3.102988e-28,
      -8.192994e-29 },
    { 1.071306e-07, -1.262144e-08, -4.767231e-10, 1.175560e-11, 6.946241e-13,
      -9.316733e-14, -4.427290e-15, 4.858365e-16, 4.814810e-17, 2.752709e-19,
      -2.449926e-20, -6.393665e-21, 8.842755e-22, 4.178428e-23, -3.177778e-24,
      1.229862e-25, -8.535124e-26, -1.658684e-26, -1.524672e-28, -2.246909e-29,
      -5.508346e-31 },
    { 4.421672e-08, 1.958333e-09, 3.236166e-10, -5.174199e-12, 4.022242e-12,
      3.088082e-14, 3.197551e-15, 9.009281e-17, 2.534982e-17, -9.526323e-19,
      1.741250e-20, -1.569624e-21, -4.195542e-22, -6.629972e-24, -6.574751e-25,
      -2.898577e-25, 7.555273e-27, 3.046776e-28, 3.696154e-29, 1.845778e-30,
      6.948820e-31 },
    { -2.197334e-08, -3.156695e-09, 7.325272e-10, -1.192913e-11, 9.941288e-13,
      3.991921e-14, -4.220405e-16, 7.091584e-17, 1.660451e-17, 9.233532e-20,
      -5.971908e-20, 1.750987e-21, -2.066463e-23, -3.440194e-24, -1.487095e-25,
      -4.491878e-26, -4.558801e-27, 5.960375e-28, 8.263952e-29, -9.155723e-31,
      -1.237749e-31 },
    { 1.203146e-07, 3.688524e-09, 4.328972e-10, -6.303973e-12, 2.869669e-13,
      -3.011115e-14, 1.539793e-15, -1.390222e-16, 1.766707e-18, 3.471731e-19,
      -3.447438e-20, 8.760347e-22, -2.271884e-23, 5.960951e-24, 1.682025e-25,
      -2.520877e-26, -8.774566e-28, 2.651434e-29, 8.352807e-30, -1.878413e-31,
      4.054696e-32 }
};

EarthEnvironment::EarthEnvironment() : time(time_management::get_instance())
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
    // data_exchang->hset("TEI", VehicleIn->Env->TEI);
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
    /* passing data*/
    // arma::vec3 VBED;
    // arma::vec3 SBII;
    // double alt, alppx, phipx;
    // arma::mat33 TGI;
    // arma::mat33 TBI;
    // arma::mat33 TBD;
    // arma::vec3 VBEE;
    // arma::mat33 TDE;
    // data_exchang->hget("VBED", VBED);
    // data_exchang->hget("SBII", SBII);
    // data_exchang->hget("alt", &alt);
    // data_exchang->hget("TGI", TGI);
    // data_exchang->hget("TBI", TBI);
    // data_exchang->hget("TBD", TBD);
    // data_exchang->hget("alppx", &alppx);
    // data_exchang->hget("phipx", &phipx);
    // data_exchang->hget("VBEE", VBEE);
    // data_exchang->hget("TDE", TDE);

    EarthEnvironment_var *E;
    DM_var *D;
    E = VehicleIn->Env;
    D = VehicleIn->DM;
    double int_step = VehicleIn->dt;

    /******************************/

    E->TEI =
        RNP();  // Calculate Rotation-Nutation-Precession (ECI to ECEF) Matrix

    E->GRAVG = AccelHarmonic(D->SBII, E->TEI, 20, 20);

    atmosphere->set_altitude(D->alt);

    wind->set_altitude(D->alt);
    wind->propagate_VAED(int_step);
    wind->apply_turbulance_if_have(int_step, E->dvba, D->TBD, D->alppx, D->phipx);

    // flight conditionsnorm(VBAD)
    E->VBAB =
        D->TBI * trans(E->TEI) * (D->VBEE - trans(D->TDE) * wind->get_VAED());
    E->dvba = norm(E->VBAB);

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

    // data_exchang->hset("press", atmosphere->get_pressure());
    // data_exchang->hset("rho", atmosphere->get_density());
    // data_exchang->hset("vmach", E->vmach);
    // data_exchang->hset("pdynmc", E->pdynmc);
    // data_exchang->hset("tempk", atmosphere->get_temperature_in_kelvin());
    // data_exchang->hset("dvba", E->dvba);
    // data_exchang->hset("GRAVG", E->GRAVG);
    // data_exchang->hset("TEI", E->TEI);
    // data_exchang->hset("VAED", wind->get_VAED());
    // data_exchang->hset("gravg", norm(E->GRAVG));
}

/* Rotation-Nutation-Precession transfor Matrix (ECI to ECEF) */
arma::mat EarthEnvironment::RNP()
{
    /* double We = 7.2921151467E-5; */
    // GPSR gpsr;/* call gpsr function */
    time_util::UTC_TIME utc_caldate;
    time_util::GPS_TIME tmp_gps;
    unsigned char i;
    double UTC, UT1;
    arma::mat33 M_rotation;
    arma::mat33 M_nutation;
    arma::mat33 M_precession;
    arma::mat33 M_nut_n_pre;
    double t, t2, t3, thetaA, zetaA, zA;
    double epsilonA, epsilonAP, F, D, omega;
    double temps_sideral(0);
    double L, La, gamma, delta_psi, delta_epsilon;
    double dUT1;
    double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;
    double s_delta_psi, c_delta_psi, s_epsilonA, c_epsilonA, s_epsilonAP,
        c_epsilonAP;
    double s2_half_delta_psi, s_delta_epsilon, c_delta_epsilon;
    int index;

    double nutation_coef[106][9] = {
        { 0, 0, 0, 0, 1, -17.1996, -0.01742, 9.2025, 0.00089 },
        { 0, 0, 2, -2, 2, -1.3187, -0.00016, 0.5736, -0.00031 },
        { 0, 0, 2, 0, 2, -0.2274, -0.00002, 0.0977, -0.00005 },
        { 0, 0, 0, 0, 2, 0.2062, 0.00002, -0.0895, 0.00005 },
        { 0, 1, 0, 0, 0, 0.1426, -0.00034, 0.0054, -0.00001 },
        { 1, 0, 0, 0, 0, 0.0712, 0.00001, -0.0007, 0.00000 },
        { 0, 1, 2, -2, 2, -0.0517, 0.00012, 0.0224, -0.00006 },
        { 0, 0, 2, 0, 1, -0.0386, -0.00004, 0.0200, 0.00000 },
        { 1, 0, 2, 0, 2, -0.0301, 0.00000, 0.0129, -0.00001 },
        { 0, -1, 2, -2, 2, 0.0217, -0.00005, -0.0095, 0.00003 },
        { 1, 0, 0, -2, 0, -0.0158, 0.00000, -0.0001, 0.00000 },
        { 0, 0, 2, -2, 1, 0.0129, 0.00001, -0.0070, 0.00000 },
        { -1, 0, 2, 0, 2, 0.0123, 0.00000, -0.0053, 0.00000 },
        { 1, 0, 0, 0, 1, 0.0063, 0.00001, -0.0033, 0.00000 },
        { 0, 0, 0, 2, 0, 0.0063, 0.00000, -0.0002, 0.00000 },
        { -1, 0, 2, 2, 2, -0.0059, 0.00000, 0.0026, 0.00000 },
        { -1, 0, 0, 0, 1, -0.0058, -0.00001, 0.0032, 0.00000 },
        { 1, 0, 2, 0, 1, -0.0051, 0.00000, 0.0027, 0.00000 },
        { 2, 0, 0, -2, 0, 0.0048, 0.00000, 0.0001, 0.00000 },
        { -2, 0, 2, 0, 1, 0.0046, 0.00000, -0.0024, 0.00000 },
        { 0, 0, 2, 2, 2, -0.0038, 0.00000, 0.0016, 0.00000 },
        { 2, 0, 2, 0, 2, -0.0031, 0.00000, 0.0013, 0.00000 },
        { 2, 0, 0, 0, 0, 0.0029, 0.00000, -0.0001, 0.00000 },
        { 1, 0, 2, -2, 2, 0.0029, 0.00000, -0.0012, 0.00000 },
        { 0, 0, 2, 0, 0, 0.0026, 0.00000, -0.0001, 0.00000 },
        { 0, 0, 2, -2, 0, -0.0022, 0.00000, 0.0000, 0.00000 },
        { -1, 0, 2, 0, 1, 0.0021, 0.00000, -0.0010, 0.00000 },
        { 0, 2, 0, 0, 0, 0.0017, -0.00001, 0.0000, 0.00000 },
        { 0, 2, 2, -2, 2, -0.0016, 0.00001, 0.0007, 0.00000 },
        { -1, 0, 0, 2, 1, 0.0016, 0.00000, -0.0008, 0.00000 },
        { 0, 1, 0, 0, 1, -0.0015, 0.00000, 0.0009, 0.00000 },
        { 1, 0, 0, -2, 1, -0.0013, 0.00000, 0.0007, 0.00000 },
        { 0, -1, 0, 0, 1, -0.0012, 0.00000, 0.0006, 0.00000 },
        { 2, 0, -2, 0, 0, 0.0011, 0.00000, 0.0000, 0.00000 },
        { -1, 0, 2, 2, 1, -0.0010, 0.00000, 0.0005, 0.00000 },
        { 1, 0, 2, 2, 2, -0.0008, 0.00000, 0.0003, 0.00000 },
        { 0, -1, 2, 0, 2, -0.0007, 0.00000, 0.0003, 0.00000 },
        { 0, 0, 2, 2, 1, -0.0007, 0.00000, 0.0003, 0.00000 },
        { 1, 1, 0, -2, 0, -0.0007, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 2, 0, 2, 0.0007, 0.00000, -0.0003, 0.00000 },
        { -2, 0, 0, 2, 1, -0.0006, 0.00000, 0.0003, 0.00000 },
        { 0, 0, 0, 2, 1, -0.0006, 0.00000, 0.0003, 0.00000 },
        { 2, 0, 2, -2, 2, 0.0006, 0.00000, -0.0003, 0.00000 },
        { 1, 0, 0, 2, 0, 0.0006, 0.00000, 0.0000, 0.00000 },
        { 1, 0, 2, -2, 1, 0.0006, 0.00000, -0.0003, 0.00000 },
        { 0, 0, 0, -2, 1, -0.0005, 0.00000, 0.0003, 0.00000 },
        { 0, -1, 2, -2, 1, -0.0005, 0.00000, 0.0003, 0.00000 },
        { 2, 0, 2, 0, 1, -0.0005, 0.00000, 0.0003, 0.00000 },
        { 1, -1, 0, 0, 0, 0.0005, 0.00000, 0.0000, 0.00000 },
        { 1, 0, 0, -1, 0, -0.0004, 0.00000, 0.0000, 0.00000 },
        { 0, 0, 0, 1, 0, -0.0004, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 0, -2, 0, -0.0004, 0.00000, 0.0000, 0.00000 },
        { 1, 0, -2, 0, 0, 0.0004, 0.00000, 0.0000, 0.00000 },
        { 2, 0, 0, -2, 1, 0.0004, 0.00000, -0.0002, 0.00000 },
        { 0, 1, 2, -2, 1, 0.0004, 0.00000, -0.0002, 0.00000 },
        { 1, 1, 0, 0, 0, -0.0003, 0.00000, 0.0000, 0.00000 },
        { 1, -1, 0, -1, 0, -0.0003, 0.00000, 0.0000, 0.00000 },
        { -1, -1, 2, 2, 2, -0.0003, 0.00000, 0.0001, 0.00000 },
        { 0, -1, 2, 2, 2, -0.0003, 0.00000, 0.0001, 0.00000 },
        { 1, -1, 2, 0, 2, -0.0003, 0.00000, 0.0001, 0.00000 },
        { 3, 0, 2, 0, 2, -0.0003, 0.00000, 0.0001, 0.00000 },
        { -2, 0, 2, 0, 2, -0.0003, 0.00000, 0.0001, 0.00000 },
        { 1, 0, 2, 0, 0, 0.0003, 0.00000, 0.0000, 0.00000 },
        { -1, 0, 2, 4, 2, -0.0002, 0.00000, 0.0001, 0.00000 },
        { 1, 0, 0, 0, 2, -0.0002, 0.00000, 0.0001, 0.00000 },
        { -1, 0, 2, -2, 1, -0.0002, 0.00000, 0.0001, 0.00000 },
        { 0, -2, 2, -2, 1, -0.0002, 0.00000, 0.0001, 0.00000 },
        { -2, 0, 0, 0, 1, -0.0002, 0.00000, 0.0001, 0.00000 },
        { 2, 0, 0, 0, 1, 0.0002, 0.00000, -0.0001, 0.00000 },
        { 3, 0, 0, 0, 0, 0.0002, 0.00000, 0.0000, 0.00000 },
        { 1, 1, 2, 0, 2, 0.0002, 0.00000, -0.0001, 0.00000 },
        { 0, 0, 2, 1, 2, 0.0002, 0.00000, -0.0001, 0.00000 },
        { 1, 0, 0, 2, 1, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 0, 2, 2, 1, -0.0001, 0.00000, 0.0001, 0.00000 },
        { 1, 1, 0, -2, 1, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 0, 2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 2, -2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, -2, 2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 0, -2, -2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 0, -2, 2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 0, 2, -2, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 0, 0, -4, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 2, 0, 0, -4, 0, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 0, 2, 4, 2, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 0, 2, -1, 2, -0.0001, 0.00000, 0.0000, 0.00000 },
        { -2, 0, 2, 4, 2, -0.0001, 0.00000, 0.0001, 0.00000 },
        { 2, 0, 2, 2, 2, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, -1, 2, 0, 1, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 0, -2, 0, 1, -0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 0, 4, -2, 2, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 0, 0, 2, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, 1, 2, -2, 2, 0.0001, 0.00000, -0.0001, 0.00000 },
        { 3, 0, 2, -2, 2, 0.0001, 0.00000, 0.0000, 0.00000 },
        { -2, 0, 2, 2, 2, 0.0001, 0.00000, -0.0001, 0.00000 },
        { -1, 0, 0, 0, 2, 0.0001, 0.00000, -0.0001, 0.00000 },
        { 0, 0, -2, 2, 1, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 2, 0, 1, 0.0001, 0.00000, 0.0000, 0.00000 },
        { -1, 0, 4, 0, 2, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 2, 1, 0, -2, 0, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 2, 0, 0, 2, 0, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 2, 0, 2, -2, 1, 0.0001, 0.00000, -0.0001, 0.00000 },
        { 2, 0, -2, 0, 1, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 1, -1, 0, -2, 0, 0.0001, 0.00000, 0.0000, 0.00000 },
        { -1, 0, 0, 1, 1, 0.0001, 0.00000, 0.0000, 0.00000 },
        { -1, -1, 0, 2, 1, 0.0001, 0.00000, 0.0000, 0.00000 },
        { 0, 1, 0, 1, 0, 0.0001, 0.00000, 0.0000, 0.00000 }
    };

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
                                 55197.00); /* get dUT1 = Ut1 - UT from table*/
    if ((index >= 0) &&
        (index < Max_DM_UT1_UT_Index)) { /* MJD = 55197.00 (1-1-2010)~
                                          56150.00(8-11-2012) */
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
        gamma = nutation_coef[i][0] * L + nutation_coef[i][1] * La +
                nutation_coef[i][2] * F + nutation_coef[i][3] * D +
                nutation_coef[i][4] * omega; /* unit: radian */
        delta_psi = delta_psi + (nutation_coef[i][5] + nutation_coef[i][6] * t) *
                                    sin(gamma); /* unit: arcsec */
        delta_epsilon =
            delta_epsilon + (nutation_coef[i][7] + nutation_coef[i][8] * t) *
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
        c_delta_epsilon - 2 * s2_half_delta_psi * c_epsilonA * c_epsilonAP;
    M_nutation(1, 2) =
        -s_delta_epsilon - 2 * s2_half_delta_psi * s_epsilonA * c_epsilonAP;
    M_nutation(2, 0) = s_epsilonAP * s_delta_psi;
    M_nutation(2, 1) =
        s_delta_epsilon - 2 * s2_half_delta_psi * c_epsilonA * s_epsilonAP;
    M_nutation(2, 2) =
        c_delta_epsilon - 2 * s2_half_delta_psi * s_epsilonA * s_epsilonAP;

    /*----------------------------------------------------------- */
    /*-------- Matrice of Nutation * Precession --------*/
    /*----------------------------------------------------------- */
    M_nut_n_pre = M_nutation * M_precession;

    /*----------------------------------------------------------- */
    /*------------------- Rotation Matrix --------------------*/
    /*----------------------------------------------------------- */
    temps_sideral =
        UT1 + (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3);
    temps_sideral = temps_sideral * DM_sec2r +
                    delta_psi * cos(epsilonA) * DM_arcsec2r; /* unit: radian */

    M_rotation(0, 0) = cos(temps_sideral);
    M_rotation(0, 1) = sin(temps_sideral);
    M_rotation(0, 2) = 0.0;
    M_rotation(1, 0) = -sin(temps_sideral);
    M_rotation(1, 1) = cos(temps_sideral);
    M_rotation(1, 2) = 0.0;
    M_rotation(2, 0) = 0.0;
    M_rotation(2, 1) = 0.0;
    M_rotation(2, 2) = 1.0;

    return M_rotation * M_nut_n_pre;
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
arma::vec EarthEnvironment::AccelHarmonic(arma::vec3 SBII, arma::mat33 TEI,
                                          int n_max,
                                          int m_max)
{
    /* Local variables */
    int n, m;               /* Loop counters */
    double r_sqr, rho, Fac; /* Auxiliary quantities */
    double x0, y0, z0;      /* Normalized coordinates */
    double ax, ay, az;      /* Acceleration vector */
    double C, S;            /* Gravitational coefficients */

    arma::vec r_bf(3); /* Earth-fixed position */
    arma::vec a_bf(3); /* Earth-fixed acceleration */

    double V[N_JGM3 + 2][N_JGM3 + 2] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    }; /* Harmonic functions
                                                          */
    double
        W[N_JGM3 + 2]
         [N_JGM3 + 2] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; /* work array
                                                         (0..n_max+1,0..n_max+1)
                                                       */
    /* Earth-fixed position */
    r_bf = TEI * SBII;

    /* Auxiliary quantities */
    r_sqr = dot(r_bf, r_bf); /* Square of distance */
    rho = SMAJOR_AXIS * SMAJOR_AXIS / r_sqr;
    x0 = SMAJOR_AXIS * r_bf(0) / r_sqr; /* Normalized  */
    y0 = SMAJOR_AXIS * r_bf(1) / r_sqr; /* coordinates */
    z0 = SMAJOR_AXIS * r_bf(2) / r_sqr;

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

    /* Calculate accelerations ax,ay,az */
    ax = ay = az = 0.0;

    for (m = 0; m <= m_max; m++) {
        for (n = m; n <= n_max; n++) {
            if (m == 0) {
                C = CS_JGM3[n][0]; /* = C_n,0 */
                ax -= C * V[n + 1][1];
                ay -= C * W[n + 1][1];
                az -= (n + 1) * C * V[n + 1][0];
            } else {
                C = CS_JGM3[n][m];     /* = C_n,m */
                S = CS_JGM3[m - 1][n]; /* = S_n,m */
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
    a_bf(0) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * ax;
    a_bf(1) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * ay;
    a_bf(2) = (GM / (SMAJOR_AXIS * SMAJOR_AXIS)) * az;

    arma::vec tmp_vec = trans(TEI) * a_bf;
    return tmp_vec;
} /* end of AccelHarmonic */
