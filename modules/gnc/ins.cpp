#include "ins.hh"

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

double INS_CS_JGM3[N_JGM3 + 1][N_JGM3 + 1] = {
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

INS::INS()
    : time(time_management::Instance()),
      MATRIX_INIT(WEII, 3, 3),
      VECTOR_INIT(EVBI, 3),
      VECTOR_INIT(EVBID, 3),
      VECTOR_INIT(ESBI, 3),
      VECTOR_INIT(ESBID, 3),
      VECTOR_INIT(RICI, 3),
      VECTOR_INIT(RICID, 3),
      MATRIX_INIT(TBIC, 3, 3),
      VECTOR_INIT(TBIC_Q, 4),
      VECTOR_INIT(TBIDC_Q, 4),
      VECTOR_INIT(SBIIC, 3),
      VECTOR_INIT(VBIIC, 3),
      VECTOR_INIT(SBEEC, 3),
      VECTOR_INIT(VBEEC, 3),
      VECTOR_INIT(WBICI, 3),
      VECTOR_INIT(EGRAVI, 3),
      VECTOR_INIT(VBECD, 3),
      MATRIX_INIT(TDCI, 3, 3),
      MATRIX_INIT(TEIC, 3, 3),
      VECTOR_INIT(INS_I_ATT_ERR, 3),
      VECTOR_INIT(TESTV, 3),
      VECTOR_INIT(TMP_old, 3),
      VECTOR_INIT(VBIIC_old, 3),
      VECTOR_INIT(POS_ERR, 3),
      VECTOR_INIT(GRAVGI, 3),
      VECTOR_INIT(TBDQ, 4),
      MATRIX_INIT(TBD, 3, 3),
      MATRIX_INIT(TBICI, 3, 3),
      MATRIX_INIT(TLI, 3, 3),
      VECTOR_INIT(VBIIC_old_old, 3),
      VECTOR_INIT(WBECB, 3),
      VECTOR_INIT(ABICB, 3)
{
    this->default_data();
    this->liftoff = 0;
}

INS::INS(const INS &other)
{
    INS();

    this->grab_SXH = other.grab_SXH;
    this->grab_VXH = other.grab_VXH;
    this->grab_gps_update = other.grab_gps_update;
    this->clear_gps_flag = other.clear_gps_flag;

    /* Propagative Stats */
    this->EVBI = other.EVBI;
    this->EVBID = other.EVBID;
    this->ESBI = other.ESBI;
    this->ESBID = other.ESBID;
    this->RICI = other.RICI;
    this->RICID = other.RICID;

    /* Generating Outputs */
    this->TBIC = other.TBIC;
    this->SBIIC = other.SBIIC;
    this->VBIIC = other.VBIIC;
    this->WBICI = other.WBICI;
    this->EGRAVI = other.EGRAVI;

    this->loncx = other.loncx;
    this->latcx = other.latcx;
    this->altc = other.altc;

    this->VBECD = other.VBECD;
    this->TDCI = other.TDCI;

    this->dbic = other.dbic;
    this->dvbec = other.dvbec;

    this->alphacx = other.alphacx;
    this->betacx = other.betacx;

    this->thtvdcx = other.thtvdcx;
    this->psivdcx = other.psivdcx;

    this->alppcx = other.alppcx;
    this->phipcx = other.phipcx;

    this->phibdcx = other.phibdcx;
    this->thtbdcx = other.thtbdcx;
    this->psibdcx = other.psibdcx;
}

INS &INS::operator=(const INS &other)
{
    if (&other == this)
        return *this;

    this->grab_SXH = other.grab_SXH;
    this->grab_VXH = other.grab_VXH;
    this->grab_gps_update = other.grab_gps_update;
    this->clear_gps_flag = other.clear_gps_flag;

    /* Propagative Stats */
    this->EVBI = other.EVBI;
    this->EVBID = other.EVBID;
    this->ESBI = other.ESBI;
    this->ESBID = other.ESBID;
    this->RICI = other.RICI;
    this->RICID = other.RICID;

    /* Generating Outputs */
    this->TBIC = other.TBIC;
    this->SBIIC = other.SBIIC;
    this->VBIIC = other.VBIIC;
    this->WBICI = other.WBICI;
    this->EGRAVI = other.EGRAVI;

    this->loncx = other.loncx;
    this->latcx = other.latcx;
    this->altc = other.altc;

    this->VBECD = other.VBECD;
    this->TDCI = other.TDCI;

    this->dbic = other.dbic;
    this->dvbec = other.dvbec;

    this->alphacx = other.alphacx;
    this->betacx = other.betacx;

    this->thtvdcx = other.thtvdcx;
    this->psivdcx = other.psivdcx;

    this->alppcx = other.alppcx;
    this->phipcx = other.phipcx;

    this->phibdcx = other.phibdcx;
    this->thtbdcx = other.thtbdcx;
    this->psibdcx = other.psibdcx;

    return *this;
}

arma::mat INS::build_WEII()
{
    arma::mat33 tmp;
    tmp.zeros();
    // WEII(0, 2) = WEII2;
    // WEII(2, 0) = -WEII2;
    tmp(0, 1) = -WEII3;
    tmp(1, 0) = WEII3;
    // WEII(1, 2) = -WEII1;
    // WEII(2, 1) = WEII1;
    return tmp;
}

void INS::default_data()
{
    this->WEII = build_WEII();
    // this->TEIC = calculate_INS_derived_TEI();
    this->EGRAVI.zeros();
}

void INS::initialize()
{
    this->WEII = build_WEII();
    calculate_INS_derived_TEI();
    SBIIC = cad::in_geo84(loncx * RAD, latcx * RAD, altc, TEIC);
    GRAVGI = AccelHarmonic(SBIIC, INS_CS_JGM3, 20, 20, TEIC);
    SBEEC = TEIC * SBIIC;
    VBEEC = TEIC * VBIIC - WEII * SBEEC;
    testindex = 0;
}

void INS::load_location(double lonx, double latx, double alt)
{
    this->loncx = lonx;
    this->latcx = latx;
    this->altc = alt;

    calculate_INS_derived_TEI();
    // converting geodetic lonx, latx, alt to SBII
}

void INS::load_angle(double yaw, double roll, double pitch)
{
    this->psibdcx = yaw;
    this->phibdcx = roll;
    this->thtbdcx = pitch;

    calculate_INS_derived_TEI();

    TBD = build_psi_tht_phi_TM(psibdcx * RAD, thtbdcx * RAD, phibdcx * RAD);
    TLI = build_psi_tht_phi_TM(psibdcx * RAD, thtbdcx * RAD, phibdcx * RAD);

    TDCI = cad::tdi84(loncx * RAD, latcx * RAD, altc, TEIC);

    TBIC = TBD * TDCI;
    this->TBIC_Q = Matrix2Quaternion(
        this->TBIC);  // Convert Direct Cosine Matrix to Quaternion
    this->TBICI = TLI * TDCI;
}

void INS::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe)
{
    // building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    arma::mat33 TDI = cad::tdi84(loncx * RAD, latcx * RAD, altc, TEIC);
    // Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;
    SBIIC = cad::in_geo84(loncx * RAD, latcx * RAD, altc, TEIC);
    VBIIC = trans(TDI) * VBED + trans(TEIC) * (WEII * TEIC * SBIIC);
    VBIIC_old = VBIIC;
    // TESTV = VBIIC;
}

arma::vec INS::build_VBEB(double _alpha0x, double _beta0x, double _dvbe)
{
    double salp = sin(_alpha0x * RAD);
    double calp = cos(_alpha0x * RAD);
    double sbet = sin(_beta0x * RAD);
    double cbet = cos(_beta0x * RAD);
    double vbeb1 = calp * cbet * _dvbe;
    double vbeb2 = sbet * _dvbe;
    double vbeb3 = salp * cbet * _dvbe;
    arma::vec3 VBEB = { vbeb1, vbeb2, vbeb3 };
    return VBEB;
}

void INS::set_ideal()
{
    ideal = 1;

    return;
}

void INS::set_liftoff(unsigned int index) { this->liftoff = index; }

/* frax_algnmnt : Fractn to mod initial INS err state: XXO=XXO(1+frax) */
void INS::set_non_ideal()
{
    ideal = 0;

    return;
}

arma::vec3 INS::calculate_INS_derived_postion(arma::vec3 SBII)
{
    // computing INS derived postion of hyper B wrt center of Earth I
    return ESBI + SBII;
}

arma::vec3 INS::calculate_INS_derived_velocity(arma::vec3 VBII)
{
    // computing INS derived velocity of hyper B wrt inertial frame I
    return EVBI + VBII;
}

arma::vec3 INS::calculate_INS_derived_bodyrate(arma::mat33 TBIC_in,
                                               arma::vec3 WBICB)
{
    // computing INS derived body rates in inertial coordinates
    return trans(TBIC_in) * WBICB;
}

arma::mat33 INS::calculate_INS_derived_TBI(arma::mat33 TBI)
{
    // computed transformation matrix
    arma::mat33 UNI(arma::fill::eye);
    arma::mat33 TIIC = UNI - skew_sym(RICI);
    return TBI * TIIC;
}

void INS::calculate_INS_derived_TEI()
{
    /* double We = 7.2921151467E-5; */
    // GPSR gpsr;/* call gpsr function */
    struct time_util::UTC_TIME utc_caldate;
    struct time_util::GPS_TIME tmp_gps;
    double UTC, UT1;
    arma::mat33 M_rotation;

    arma::mat33 M_nutation;
    M_nutation.eye();

    arma::mat33 M_precession;
    arma::mat33 M_nut_n_pre;
    double t, t2, t3, thetaA, zetaA, zA;
    double epsilonA(0);
    double temps_sideral(0);
    double delta_psi(0);
    double dUT1;
    double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;

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

    index = static_cast<int>(time->get_modified_julian_date().get_mjd() -
                             55197.00); /* get dUT1 = Ut1 - UT from table*/
    if ((index >= 0) &&
        (index < Max_DM_UT1_UT_Index)) { /*MJD = 55197.00 (1-1-2010)~
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

    this->TEIC = M_rotation * M_nut_n_pre;
    // this->TEIC = M_rotation;
}

arma::vec3 INS::calculate_gravity_error(double dbi)
{
    dbic = norm(SBIIC);
    double ed = dbic - dbi;
    double dum = GM / pow(dbic, 3);
    if (dbic != 0) {
        return ESBI * (-dum) - SBIIC * (3 * ed * dum / dbic);
    } else {
        return arma::vec3(arma::fill::zeros);
    }
}

double INS::calculate_INS_derived_dvbe() { return 0; }

bool INS::GPS_update()
{
    // GPS update
    if (grab_gps_update()) {
        // GPS Measurement
        arma::vec3 SXH = grab_SXH();
        arma::vec3 VXH = grab_VXH();

        // updating INS navigation output
        SBIIC = SBIIC - SXH;
        VBIIC = VBIIC - VXH;
        // resetting INS error states
        ESBI = ESBI - SXH;
        EVBI = EVBI - VXH;
        // returning flag to GPS that update was completed
        clear_gps_flag();

        return true;
    }
    return false;
}

double INS::calculate_INS_derived_alpha(arma::vec3 VBECB)
{
    return atan2(VBECB(2), VBECB(0));
}

double INS::calculate_INS_derived_beta(arma::vec3 VBECB)
{
    return asin(VBECB(1) / norm(VBECB));
}

double INS::calculate_INS_derived_alpp(arma::vec3 VBECB)
{
    double dum = VBECB(0) / norm(VBECB);
    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    return acos(dum);
}

double INS::calculate_INS_derived_psivd(arma::vec3 VBECD_in)
{
    if (VBECD_in(0) == 0 && VBECD_in(1) == 0) {
        return 0;
    } else {
        return atan2(VBECD_in(1), VBECD_in(0));
    }
}

double INS::calculate_INS_derived_thtvd(arma::vec3 VBECD_in)
{
    if (VBECD_in(0) == 0 && VBECD_in(1) == 0) {
        return 0;
    } else {
        return atan2(-VBECD_in(2), sqrt(VBECD_in(0) * VBECD_in(0) + VBECD_in(1) * VBECD_in(1)));
    }
}

double INS::calculate_INS_derived_phip(arma::vec3 VBECB)
{
    double phipc(0);

    if (VBECB(1) == 0 && VBECB(2) == 0) {
        phipc = 0.;
    } else if (fabs(VBECB(1)) < EPS) {
        // note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
        //       to prevent oscillations
        if (VBECB(2) > 0)
            phipc = 0;
        if (VBECB(2) < 0)
            phipc = PI;
    } else {
        phipc = atan2(VBECB(1), VBECB(2));
    }

    return phipc;
}

void INS::calculate_INS_derived_euler_angles(arma::mat33 TBD_in)
{
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    double mroll = 0;

    double tbd13 = TBD_in(0, 2);
    double tbd11 = TBD_in(0, 0);
    double tbd33 = TBD_in(2, 2);
    double tbd12 = TBD_in(0, 1);
    double tbd23 = TBD_in(1, 2);

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

    this->psibdcx = DEG * psibdc;
    this->thtbdcx = DEG * thtbdc;
    this->phibdcx = DEG * phibdc;
}

void INS::update(double int_step)
{
    // local variables
    double lonc(0), latc(0);
    arma::vec3 zo(arma::fill::zeros);
    arma::vec3 PHI, PHI_HIGH, PHI_LOW, DELTA_VEL;
    // Gyro Measurement
    arma::vec3 WBICB = grab_computed_WBIB();
    arma::vec3 FSPCB = grab_computed_FSPB();

    // if (ideal == 1) {
    PHI = grab_PHI();
    // } else {;
    //     PHI_HIGH = grab_PHI_HIGH();
    //     PHI_LOW = grab_PHI_LOW();
    // }
    DELTA_VEL = grab_DELTA_VEL();

    calculate_INS_derived_TEI();
    GRAVGI = AccelHarmonic(SBIIC, INS_CS_JGM3, 20, 20, TEIC);

    // if (ideal == 1) {
    TBIC = build_321_rotation_matrix(PHI) * TBIC;
    // } else {
    //   TBIC = build_321_rotation_matrix((PHI_HIGH * GMSB + PHI_LOW * GLSB) *
    //   0.005) * TBIC;
    // }

    arma::mat33 TICB = trans(TBIC);

    // if (testindex == 0) {
    //     VBIIC_old = VBIIC;
    //     testindex = 1;
    // } else {
    VBIIC += trans(TBIC) * DELTA_VEL + GRAVGI * int_step;
    SBIIC += VBIIC_old * int_step;
    VBIIC_old = VBIIC;
    // }
    // computed transformation matrix

    // if (gpsupdate == 1) {
    //     GPS_update();
    // }

    // this->WBICI = calculate_INS_derived_bodyrate(TBIC, WBICB);

    SBEEC = TEIC * SBIIC;
    VBEEC = TEIC * VBIIC - WEII * SBEEC;
    arma::vec3 WEIE;
    WEIE(2) = WEII3;
    WBECB = WBICB - TBIC * trans(TEIC) * WEIE;
    ABICB = FSPCB + TBIC * GRAVGI;

    arma::vec3 VBECB = TBIC * trans(TEIC) * VBEEC;

    this->dvbec = norm(VBECB);
    if (this->liftoff == 1) {
        // computing indidence angles from INS
        this->alphacx = calculate_INS_derived_alpha(VBECB) * DEG;
        this->betacx = calculate_INS_derived_beta(VBECB) * DEG;

        // incidence angles in load factor plane (aeroballistic)
        this->alppcx = calculate_INS_derived_alpp(VBECB) * DEG;
        this->phipcx = calculate_INS_derived_phip(VBECB) * DEG;
    }

    // getting long,lat,alt from INS
    std::tie(lonc, latc, altc) = cad::geo84_in(SBIIC, TEIC);

    loncx = lonc * DEG;
    latcx = latc * DEG;

    // getting T.M. of geodetic wrt inertial coord
    this->TDCI = cad::tdi84(lonc, latc, altc, TEIC);
    // getting Launch site T.M. of inertial coord
    if (this->liftoff == 0)
        TBICI = TLI * TDCI;

    // computing geodetic velocity from INS
    VBECD = TDCI * trans(TEIC) * VBEEC;

    // computing flight path angles
    this->psivdcx = calculate_INS_derived_psivd(VBECD) * DEG;
    this->thtvdcx = calculate_INS_derived_thtvd(VBECD) * DEG;

    // computing Euler angles from INS
    TBD = TBIC * trans(TDCI);
    TBDQ = Matrix2Quaternion(TBD);
    calculate_INS_derived_euler_angles(TBD);
    // error_diagnostics();
}

double INS::get_loncx() { return loncx; }
double INS::get_latcx() { return latcx; }
double INS::get_altc() { return altc; }
double INS::get_dvbec() { return dvbec; }
double INS::get_alphacx() { return alphacx; }
double INS::get_betacx() { return betacx; }
double INS::get_phibdcx() { return phibdcx; }
double INS::get_thtbdcx() { return thtbdcx; }
double INS::get_psibdcx() { return psibdcx; }
double INS::get_thtvdcx() { return thtvdcx; }
double INS::get_phipcx() { return phipcx; }
double INS::get_alppcx() { return alppcx; }

arma::vec3 INS::get_SBIIC() { return SBIIC; }
arma::vec3 INS::get_VBIIC() { return VBIIC; }
arma::vec3 INS::get_SBEEC() { return SBEEC; }
arma::vec3 INS::get_VBEEC() { return VBEEC; }
arma::vec3 INS::get_WBICI() { return WBICI; }
arma::vec3 INS::get_EGRAVI() { return EGRAVI; }
arma::mat33 INS::get_TBIC() { return TBIC; }
arma::mat33 INS::get_TEIC() { return TEIC; }
arma::vec4 INS::get_TBDQ() { return TBDQ; }
arma::mat33 INS::get_TBD() { return TBD; }
arma::mat33 INS::get_TBICI() { return TBICI; }
arma::mat33 INS::get_TDCI() { return TDCI; }
arma::vec3 INS::get_WBECB() { return WBECB; }
arma::vec3 INS::get_ABICB() { return ABICB; }

void INS::set_gps_correction(unsigned int index) { gpsupdate = index; }

void INS::propagate_TBI_Q(double int_step, arma::vec3 WBICB)
{
    arma::vec TBIDC_Q_NEW(4);
    /* Prepare for orthonormalization */
    double quat_metric = TBIC_Q(0) * TBIC_Q(0) + TBIC_Q(1) * TBIC_Q(1) +
                         TBIC_Q(2) * TBIC_Q(2) + TBIC_Q(3) * TBIC_Q(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */
    TBIDC_Q_NEW(0) = 0.5 * (-WBICB(0) * TBIC_Q(1) - WBICB(1) * TBIC_Q(2) -
                            WBICB(2) * TBIC_Q(3)) +
                     50. * erq * TBIC_Q(0);
    TBIDC_Q_NEW(1) = 0.5 * (WBICB(0) * TBIC_Q(0) + WBICB(2) * TBIC_Q(2) -
                            WBICB(1) * TBIC_Q(3)) +
                     50. * erq * TBIC_Q(1);
    TBIDC_Q_NEW(2) = 0.5 * (WBICB(1) * TBIC_Q(0) - WBICB(2) * TBIC_Q(1) +
                            WBICB(0) * TBIC_Q(3)) +
                     50. * erq * TBIC_Q(2);
    TBIDC_Q_NEW(3) = 0.5 * (WBICB(2) * TBIC_Q(0) + WBICB(1) * TBIC_Q(1) -
                            WBICB(0) * TBIC_Q(2)) +
                     50. * erq * TBIC_Q(3);

    this->TBIC_Q = integrate(TBIDC_Q_NEW, this->TBIDC_Q, this->TBIC_Q, int_step);

    this->TBIDC_Q = TBIDC_Q_NEW;

    this->TBIC = Quaternion2Matrix(this->TBIC_Q);  // Convert Quaternion to Matrix
}

// void INS::error_diagnostics() {
//     arma::vec3 SBII  = grab_SBII();
//     arma::vec3 VBII  = grab_VBII();
//     arma::vec3 SBEE = grab_SBEE();
//     arma::vec3 VBEE = grab_VBEE();
//     arma::mat33 TBI = grab_TBI();
//     arma::vec3 angle;
//     double phibdx = grab_phibdx();
//     double thtbdx = grab_thtbdx();
//     double psibdx = grab_psibdx();

//     // diagnostics
//     angle = (euler_angle(TBIC) - euler_angle(TBI)) * DEG;
//     TESTV = VBII - VBIIC;
//     ins_pos_err  = norm(SBII - SBIIC);   // norm(ESBI);
//     ins_vel_err  = norm(VBII - VBIIC);   // norm(EVBI);
//     ins_pose_err  = norm(SBEE - SBEEC);  // norm(ESBI);
//     ins_vele_err  = norm(VBEE - VBEEC);  // norm(EVBI);
//     ins_tilt_err = norm(RICI);
//     ins_phi_err = phibdx - phibdcx;  // angle(0);
//     ins_tht_err = thtbdx - thtbdcx;  // angle(1);
//     ins_psi_err = psibdx - psibdcx;  // angle(2);
//     INS_I_ATT_ERR = angle;
//     POS_ERR = SBIIC - SBII;
// }

arma::vec3 INS::euler_angle(arma::mat33 TBD_in)
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

arma::mat33 INS::build_321_rotation_matrix(arma::vec3 angle)
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
arma::vec INS::AccelHarmonic(arma::vec3 SBII, double CS[21][21], int n_max,
                             int m_max, arma::mat33 TEIC_in)
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
    r_bf = TEIC_in * SBII;

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
                C = CS[n][0]; /* = C_n,0 */
                ax -= C * V[n + 1][1];
                ay -= C * W[n + 1][1];
                az -= (n + 1) * C * V[n + 1][0];
            } else {
                C = CS[n][m];     /* = C_n,m */
                S = CS[m - 1][n]; /* = S_n,m */
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

    /* Inertial acceleration */

    arma::vec tmp_vec = trans(TEIC_in) * a_bf;
    return tmp_vec;
} /* end of AccelHarmonic */
