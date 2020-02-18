#include <iomanip>
#include <tuple>

#include "aux.hh"
#include "cadac_util.hh"
#include "integrate.hh"
#include "matrix_tool.hh"
#include "rocket_flight_dm.hh"
//#include "sim_services/include/simtime.h"
#define NRANSI
#include "nrutil.hh"
#define MAXITS 20000000
#define ALF 1.0e-4
#define EPS 1.0e-7   // 1.0e-7
#define TOLF 1.0e-8  // 1.0e-4
#define TOLX 1.0e-9
#define STPMX 100.0
#define TOLMIN 1.0e-10  // 1.0e-6
#define FREERETURN                    \
    do {                              \
        free_dvector(fvec, 1, n);     \
        free_dvector(xold, 1, n);     \
        free_dvector(w, 1, n);        \
        free_dvector(t, 1, n);        \
        free_dvector(s, 1, n);        \
        free_dmatrix(r, 1, n, 1, n);  \
        free_dmatrix(qt, 1, n, 1, n); \
        free_dvector(p, 1, n);        \
        free_dvector(g, 1, n);        \
        free_dvector(fvcold, 1, n);   \
        free_dvector(d, 1, n);        \
        free_dvector(c, 1, n);        \
        return;                       \
    } while (0);

double *fvec;
int nn;

Rocket_Flight_DM::Rocket_Flight_DM() { reference_point_flag = 0; }

void Rocket_Flight_DM::init(LaunchVehicle *VehicleIn)
{
    DM_var *D;
    Prop_var *P;
    EarthEnvironment_var *E;
    E = VehicleIn->Env;
    P = VehicleIn->Prop;
    D = VehicleIn->DM;

    arma::mat33 TEI = E->TEI;  // cad::tei(get_elapsed_time());
    arma::vec3 XCG = P->XCG;

    reference_point_calc(D, P);
    D->rhoC_1(0) = XCG(0) - D->reference_point;
    D->rhoC_1(1) = 0.0;
    D->rhoC_1(2) = 0.0;

    // converting geodetic lonx, latx, alt to SBII
    D->SBII = cad::in_geo84(D->lonx * RAD, D->latx * RAD, D->alt, TEI);

    // building inertial velocity
    D->TDI = cad::tdi84(D->lonx * RAD, D->latx * RAD, D->alt, TEI);
    D->TGI = cad::tgi84(D->lonx * RAD, D->latx * RAD, D->alt, TEI);
    D->TDE = cad::tde84(D->lonx * RAD, D->latx * RAD, D->alt);
    D->TBD =
        build_psi_tht_phi_TM(D->psibdx * RAD, D->thtbdx * RAD, D->phibdx * RAD);
    D->psibd = D->psibdx * RAD;
    D->thtbd = D->thtbdx * RAD;
    D->phibd = D->phibdx * RAD;
    D->TBI = D->TBD * D->TDI;
    D->TLI = D->TBI;

    D->WBIB = D->WBEB + D->TBI * E->WEIE;

    D->TBI_Q =
        Matrix2Quaternion(D->TBI);  // Convert Direct Cosine Matrix to Quaternion

    arma::mat VBEB = build_VBEB(D->alphax, D->betax, D->_dvbe);
    // Geodetic velocity
    D->VBED = trans(D->TBD) * VBEB;

    D->VBII =
        trans(D->TDI) * D->VBED + trans(TEI) * (E->WEIE_skew * (TEI * D->SBII));
    D->SBIIP = D->SBII - trans(D->TBI) * D->rhoC_1;
    D->VBIIP = D->VBII - trans(D->TBI) * cross(D->WBIB, D->rhoC_1);
    // arma::vec3 GRAVG;
    // data_exchang->hget("GRAVG", GRAVG);
    D->ABII = trans(TEI) * (E->WEIE_skew * E->WEIE_skew * (TEI * D->SBII));
    // FSPB = TBI * (-GRAVG + ABII);  // FSPB: body force include gravity acc
    D->SBEE = TEI * D->SBII;  // Calculate position in ECEF
    D->VBEE =
        TEI * D->VBII - cross(E->WEIE, D->SBEE);  // Calculate velocity in ECEF
    D->NEXT_ACC =
        trans(TEI) * (cross(E->WEIE, cross(E->WEIE, (TEI * (D->SBIIP)))));
    D->Interpolation_Extrapolation_flag = 4;

    if (D->liftoff == 1) {
        arma::vec3 VAED = E->VAED;
        // data_exchang->hget("VAED", VAED);
        D->VBAB = D->TBD * (D->TDE * D->VBEE - VAED);
        D->alphax = calculate_alphax(D->VBAB);
        D->betax = calculate_betax(D->VBAB, norm(D->VBAB));

        D->alppx = calculate_alppx(D->VBAB, norm(D->VBAB));
        D->phipx = calculate_phipx(D->VBAB);
    }

    D->Body_ptr = new Mobilized_body(1, D->SBIIP, D->VBIIP, D->NEXT_ACC, D->TBI_Q, D->WBIB, D->WBIBD, P->vmass, P->IBBB);
}

void Rocket_Flight_DM::set_DOF(int ndof) { DOF = ndof; }

void Rocket_Flight_DM::set_aero_flag(unsigned int in) { Aero_flag = in; }

void Rocket_Flight_DM::set_reference_point_eq_xcg()
{
    reference_point_flag = 1;
}

arma::vec Rocket_Flight_DM::build_VBEB(double _alpha0x, double _beta0x,
                                       double dvbe)
{
    double salp = sin(_alpha0x * RAD);
    double calp = cos(_alpha0x * RAD);
    double sbet = sin(_beta0x * RAD);
    double cbet = cos(_beta0x * RAD);
    double vbeb1 = calp * cbet * dvbe;
    double vbeb2 = sbet * dvbe;
    double vbeb3 = salp * cbet * dvbe;
    arma::vec3 VBEB = { vbeb1, vbeb2, vbeb3 };
    return VBEB;
}

void Rocket_Flight_DM::reference_point_calc(DM_var *D, Prop_var *P)
{
    if (reference_point_flag == 0)
        return;
    arma::vec3 XCG = P->XCG;
    D->reference_point = XCG(0);
}

void Rocket_Flight_DM::algorithm(LaunchVehicle *VehicleIn)
{
    DM_var *D;
    Prop_var *P;
    EarthEnvironment_var *E;
    E = VehicleIn->Env;
    P = VehicleIn->Prop;
    D = VehicleIn->DM;
    double int_step = VehicleIn->dt;

    arma::vec3 VAED = E->VAED;
    arma::mat33 TEI = E->TEI;

    // data_exchang->hget("VAED", VAED);
    reference_point_calc(D, P);

    std::vector<arma::vec> V_State_In;
    std::vector<arma::vec> V_State_Out(4);

    V_State_In.push_back(D->VBIIP);
    V_State_In.push_back(D->SBIIP);
    V_State_In.push_back(D->WBIB);
    V_State_In.push_back(D->TBI_Q);

    IntegratorRK4(V_State_In, V_State_Out, &Rocket_Flight_DM::RK4F, this,
                  VehicleIn, int_step);
    // IntegratorEuler(V_State_In, V_State_Out, &Rocket_Flight_DM::RK4F, this,
    //                VehicleIn, int_step);

    D->VBIIP = V_State_Out[0];
    D->SBIIP = V_State_Out[1];
    D->WBIB = V_State_Out[2];
    D->TBI_Q = V_State_Out[3];
    D->TBI = Quaternion2Matrix(D->TBI_Q);  // Convert Quaternion to Matrix

    arma::vec3 rhoC_IMU;
    arma::vec3 XCG = P->XCG;

    rhoC_IMU(0) = XCG(0) - (D->reference_point);
    rhoC_IMU(1) = 0.0;
    rhoC_IMU(2) = 0.0;

    D->SBII = D->SBIIP + trans(D->TBI) * rhoC_IMU;
    D->VBII = D->VBIIP + trans(D->TBI) * cross(D->WBIB, rhoC_IMU);
    D->ABII = D->NEXT_ACC + trans(D->TBI) * D->ddrhoC_1;

    // Send(D);

    D->TBD = calculate_TBD(VehicleIn);
    aux_calulate(TEI, int_step, D, E);
    if (D->liftoff == 1) {
        propagate_aeroloss(VehicleIn);
        propagate_gravityloss(VehicleIn);
        propagate_control_loss(VehicleIn);
        D->VBAB = D->TBD * (D->TDE * D->VBEE - VAED);
        D->alphax = calculate_alphax(D->VBAB);
        D->betax = calculate_betax(D->VBAB, norm(D->VBAB));

        D->alppx = calculate_alppx(D->VBAB, norm(D->VBAB));
        D->phipx = calculate_phipx(D->VBAB);
    }
    // *incidence angles using wind vector VAED in geodetic coord
    // *diagnostic: calculating the inertial incidence angles
    arma::vec3 VBIB = D->TBI * D->VBII;
    D->alphaix = calculate_alphaix(VBIB);
    D->betaix = calculate_betaix(VBIB);

    D->TBDQ = Matrix2Quaternion(D->TBD);
    Quaternion2Euler(D->TBDQ, D->Roll, D->Pitch, D->Yaw);
}

arma::mat Rocket_Flight_DM::calculate_TBD(LaunchVehicle *VehicleIn)
{
    // _Euler_ angles
    arma::mat33 TEI = VehicleIn->Env->TEI;
    // data_exchang->hget("TEI", TEI);
    arma::mat tdi =
        cad::tdi84(VehicleIn->DM->lonx * RAD, VehicleIn->DM->latx * RAD,
                   VehicleIn->DM->alt, TEI);
    return VehicleIn->DM->TBI * trans(tdi);
}

double Rocket_Flight_DM::calculate_alphaix(arma::vec3 VBIB)
{
    return atan2(VBIB(2), VBIB(0)) * DEG;
}

void Rocket_Flight_DM::propagate_aeroloss(LaunchVehicle *VehicleIn)
{
    double vmass = VehicleIn->Prop->vmass;
    double int_step = VehicleIn->dt;
    // data_exchang->hget("vmass", &vmass);
    // calculate aero loss
    arma::vec3 tmp = VehicleIn->DM->FAPB * (1. / vmass);
    VehicleIn->DM->_aero_loss = VehicleIn->DM->_aero_loss + norm(tmp) * int_step;
}

void Rocket_Flight_DM::propagate_control_loss(LaunchVehicle *VehicleIn)
{
    arma::vec6 Q_TVC = VehicleIn->ACT->Q_TVC;
    double vmass = VehicleIn->Prop->vmass;
    double int_step = VehicleIn->dt;
    // data_exchang->hget("Q_TVC", Q_TVC);
    // data_exchang->hget("vmass", &vmass);
    arma::vec3 A_TVC_BODY;
    A_TVC_BODY(0) = Q_TVC(0) / vmass;
    A_TVC_BODY(1) = Q_TVC(1) / vmass;
    A_TVC_BODY(2) = Q_TVC(2) / vmass;

    A_TVC_BODY = VehicleIn->DM->TBI * A_TVC_BODY;

    VehicleIn->DM->control_loss =
        VehicleIn->DM->control_loss +
        (fabs(A_TVC_BODY(1)) + fabs(A_TVC_BODY(2))) * int_step;
}

void Rocket_Flight_DM::propagate_gravityloss(LaunchVehicle *VehicleIn)
{
    // calculate gravity loss
    double gravg = VehicleIn->Env->gravg;
    double int_step = VehicleIn->dt;
    // data_exchang->hget("gravg", &gravg);

    VehicleIn->DM->gravity_loss =
        VehicleIn->DM->gravity_loss +
        gravg * sin(VehicleIn->DM->_thtvdx * RAD) * int_step;
}

double Rocket_Flight_DM::calculate_betaix(arma::vec3 VBIB)
{
    double dvbi = norm(VBIB);
    return asin(VBIB(1) / dvbi) * DEG;
}

double Rocket_Flight_DM::calculate_alppx(arma::vec3 VBAB_in, double dvba)
{
    // incidence angles in load factor plane (aeroballistic)
    double dum = VBAB_in(0) / dvba;

    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alpp = acos(dum);

    return alpp * DEG;
}

double Rocket_Flight_DM::calculate_phipx(arma::vec3 VBAB_in)
{
    double phip = 0;
    // Changed according to comments, not original code, refer commit:b613a992
    if (VBAB_in(1) == 0 && VBAB_in(2) == 0) {
        phip = 0.;
    } else if (fabs(VBAB_in(1)) < arma::datum::eps) {
        // note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
        //      to prevent oscillations
        if (VBAB_in(2) > 0)
            phip = 0;
        if (VBAB_in(2) < 0)
            phip = PI;
    } else {
        phip = atan2(VBAB_in(1), VBAB_in(2));
    }

    return phip * DEG;
}

double Rocket_Flight_DM::calculate_alphax(arma::vec3 VBAB_in)
{
    double alpha = atan2(VBAB_in(2), VBAB_in(0));
    return alpha * DEG;
}

double Rocket_Flight_DM::calculate_betax(arma::vec3 VBAB_in, double dvba)
{
    double beta = asin(VBAB_in(1) / dvba);
    return beta * DEG;
}

void Rocket_Flight_DM::orbital(DM_var *VarIn)
{
    // calculate orbital elements
    VarIn->cadorbin_flag =
        cad::orb_in(VarIn->_semi_major, VarIn->_eccentricity, VarIn->_inclination,
                    VarIn->_lon_anodex, VarIn->_arg_perix, VarIn->_true_anomx,
                    VarIn->SBII, VarIn->VBII);
    VarIn->_ha = (1. + VarIn->_eccentricity) * VarIn->_semi_major - REARTH;
    VarIn->_hp = (1. - VarIn->_eccentricity) * VarIn->_semi_major - REARTH;
    VarIn->_ref_alt = VarIn->_dbi - REARTH;
}

void Rocket_Flight_DM::aux_calulate(arma::mat33 TEI, double int_step,
                                    DM_var *DMIn, EarthEnvironment_var *EnvIn)
{
    double lon, lat, al;
    arma::mat33 TBL;

    // angular velocity wrt inertial frame in inertial coordinates
    DMIn->WBII = trans(DMIn->TBI) * DMIn->WBIB;

    // angular velocity wrt Earth in body coordinates
    DMIn->WBEB = DMIn->WBIB - DMIn->TBI * EnvIn->WEIE;

    DMIn->SBEE_old = DMIn->SBEE;
    DMIn->VBEE_old = DMIn->VBEE;
    DMIn->ABEE_old = DMIn->ABEE;

    DMIn->ABIB = DMIn->TBI * DMIn->ABII;
    DMIn->SBEE = TEI * DMIn->SBII;  // Calculate position in ECEF
    DMIn->VBEE = TEI * DMIn->VBII -
                  cross(EnvIn->WEIE, DMIn->SBEE);  // Calculate velocity in ECEF
    DMIn->ABEE = TEI * DMIn->ABII - cross(EnvIn->WEIE, DMIn->VBEE) -
                  cross(EnvIn->WEIE, DMIn->VBEE) -
                  cross(EnvIn->WEIE, cross(EnvIn->WEIE, DMIn->SBEE));
    DMIn->JBEE =
        TEI * DMIn->JBII -
        cross(EnvIn->WEIE, cross(EnvIn->WEIE, cross(EnvIn->WEIE, DMIn->SBEE))) -
        cross(EnvIn->WEIE, cross(EnvIn->WEIE, DMIn->VBEE)) -
        cross(EnvIn->WEIE, DMIn->ABEE);

    // Calculate lon lat alt
    std::tie(lon, lat, al) = cad::geo84_in(DMIn->SBII, TEI);
    DMIn->lonx = lon * DEG;
    DMIn->latx = lat * DEG;
    DMIn->alt = al;
    // std::cout<<alt<<std::endl;
    if (DMIn->liftoff == 1)
        assert(DMIn->alt >= 0.0 && " *** Stop: Ground impact detected !! *** ");

    DMIn->TDI = cad::tdi84(lon, lat, al, TEI);
    DMIn->TDE = cad::tde84(lon, lat, al);
    DMIn->TGI = cad::tgi84(lon, lat, al, TEI);
    TBL = DMIn->TBI * trans(DMIn->TLI);
    DMIn->LT_euler = euler_angle(TBL);

    DMIn->ABID = DMIn->TDI * DMIn->ABII;
    DMIn->VBED = DMIn->TDE * DMIn->VBEE;

    arma::vec3 tmp = euler_angle(DMIn->TBD);

    DMIn->thtbdx = tmp(1) * DEG;
    DMIn->psibdx = tmp(2) * DEG;
    DMIn->phibdx = tmp(0) * DEG;

    DMIn->ppx = DMIn->WBEB(0) * DEG;
    DMIn->qqx = DMIn->WBEB(1) * DEG;
    DMIn->rrx = DMIn->WBEB(2) * DEG;

    DMIn->_dbi = norm(DMIn->SBII);
    DMIn->_dvbi = norm(DMIn->VBII);

    arma::vec3 VBED;

    VBED = DMIn->TDI * (DMIn->VBII - EnvIn->WEIE_skew * DMIn->SBII);

    double vbed1 = VBED(0);
    double vbed2 = VBED(1);
    DMIn->_grndtrck += sqrt(vbed1 * vbed1 + vbed2 * vbed2) * int_step * REARTH /
                        norm(DMIn->SBII);
    DMIn->_gndtrkmx = 0.001 * DMIn->_grndtrck;
    DMIn->_gndtrnmx = NMILES * DMIn->_grndtrck;

    DMIn->_ayx = DMIn->FSPB(1) / AGRAV;
    DMIn->_anx = -DMIn->FSPB(2) / AGRAV;

    DMIn->_dvbe = pol_from_cart(VBED)(0);
    DMIn->_psivdx = pol_from_cart(VBED)(1);
    DMIn->_thtvdx = pol_from_cart(VBED)(2);

    if (DMIn->liftoff == 1) {
        // T.M. of geographic velocity wrt geodetic coordinates
        DMIn->TVD =
            build_psivg_thtvg_TM(DMIn->_psivdx * RAD, DMIn->_thtvdx * RAD);
        orbital(DMIn);
    }
}

void Rocket_Flight_DM::RK4F(std::vector<arma::vec> Var_in,
                            std::vector<arma::vec> &Var_out,
                            LaunchVehicle *VehicleIn)
{
    Prop_var *P;
    EarthEnvironment_var *E;
    DM_var *D;
    P = VehicleIn->Prop;
    E = VehicleIn->Env;
    D = VehicleIn->DM;

    double vmass = P->vmass;
    double thrust = P->thrust;
    // data_exchang->hget("vmass", &vmass);
    // data_exchang->hget("thrust", &thrust);
    arma::mat33 TEI = E->TEI;
    arma::vec3 GRAVG = E->GRAVG;
    // data_exchang->hget("GRAVG", GRAVG);
    // data_exchang->hget("TEI", TEI);
    arma::vec4 TBID_Q_NEW;
    arma::vec3 rhoC_IMU;

    D->VBIIP = Var_in[0];
    D->SBIIP = Var_in[1];
    D->WBIB = Var_in[2];
    D->TBI_Q = Var_in[3];

    collect_forces_and_propagate(VehicleIn);

    arma::vec3 XCG = P->XCG;

    // data_exchang->hget("XCG", XCG);
    D->WBIBD = D->ddang_1;

    rhoC_IMU(0) = XCG(0) - (D->reference_point);
    rhoC_IMU(1) = 0.0;
    rhoC_IMU(2) = 0.0;
    arma::vec3 ddrhoC_IMU =
        cross(D->WBIBD, rhoC_IMU) + cross(D->WBIB, cross(D->WBIB, rhoC_IMU));
    D->NEXT_ACC = D->ddrP_1;

    if (D->liftoff == 0) {
        if ((thrust - norm(vmass * GRAVG)) > 0) {
            D->liftoff = 1;
        }
        D->NEXT_ACC =
            trans(TEI) * (cross(E->WEIE, cross(E->WEIE, (TEI * (D->SBIIP)))));
        D->FSPB = QuaternionRotation(
            D->TBI_Q,
            (-GRAVG + D->NEXT_ACC +
             QuaternionRotation(QuaternionInverse(D->TBI_Q),
                                ddrhoC_IMU)));  // Strapdown Analytics 4.3-14
    } else {
        D->FSPB = QuaternionRotation(
            D->TBI_Q,
            (D->NEXT_ACC - GRAVG +
             QuaternionRotation(QuaternionInverse(D->TBI_Q),
                                ddrhoC_IMU)));  //+ TBI * GRAVG;  // FSPB: body
                                                // force include gravity acc
    }
    /* Prepare for orthonormalization */
    double quat_metric = D->TBI_Q(0) * D->TBI_Q(0) + D->TBI_Q(1) * D->TBI_Q(1) +
                         D->TBI_Q(2) * D->TBI_Q(2) + D->TBI_Q(3) * D->TBI_Q(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */  //  Zipfel p.141
    TBID_Q_NEW(0) = 0.5 * (-D->WBIB(0) * D->TBI_Q(1) - D->WBIB(1) * D->TBI_Q(2) -
                           D->WBIB(2) * D->TBI_Q(3)) +
                    50. * erq * D->TBI_Q(0);
    TBID_Q_NEW(1) = 0.5 * (D->WBIB(0) * D->TBI_Q(0) + D->WBIB(2) * D->TBI_Q(2) -
                           D->WBIB(1) * D->TBI_Q(3)) +
                    50. * erq * D->TBI_Q(1);
    TBID_Q_NEW(2) = 0.5 * (D->WBIB(1) * D->TBI_Q(0) - D->WBIB(2) * D->TBI_Q(1) +
                           D->WBIB(0) * D->TBI_Q(3)) +
                    50. * erq * D->TBI_Q(2);
    TBID_Q_NEW(3) = 0.5 * (D->WBIB(2) * D->TBI_Q(0) + D->WBIB(1) * D->TBI_Q(1) -
                           D->WBIB(0) * D->TBI_Q(2)) +
                    50. * erq * D->TBI_Q(3);

    Var_out[0] = D->NEXT_ACC;
    Var_out[1] = D->VBIIP;
    Var_out[2] = D->WBIBD;
    Var_out[3] = TBID_Q_NEW;

    // data_exchang->hset("NEXT_ACC", D->NEXT_ACC);
    // data_exchang->hset("liftoff", D->liftoff);
    // data_exchang->hset("FSPB", D->FSPB);
}

void Rocket_Flight_DM::collect_forces_and_propagate(LaunchVehicle *VehicleIn)
{
    double *ff, *x;
    int check(0);
    ff = new double[7];
    x = new double[7];
    Prop_var *P;
    DM_var *D;
    P = VehicleIn->Prop;
    D = VehicleIn->DM;
    /*****************input from another module*******************/
    D->rhoC_1 = P->XCG;
    // data_exchang->hget("XCG", VarIn->rhoC_1);
    D->dang_1 = D->WBIB;

    D->rhoC_1(0) = D->rhoC_1(0) - D->reference_point;

    calculate_I1(VehicleIn);
    gamma_beta(D);
    Gravity_Q(VehicleIn);
    if (Aero_flag == 1) {
        AeroDynamics_Q(VehicleIn);
    }

    for (int i = 0; i < DOF; i++) {
        x[i + 1] = 0.0;
    }

    broydn(x, DOF, &check, VehicleIn);

    funcv(DOF, x, ff, VehicleIn);
    for (int i = 0; i < 3; i++) {
        D->ddrP_1(i) = x[i + 1];
        D->ddang_1(i) = x[i + 4];
    }

    D->ddrhoC_1 = cross(D->ddang_1, D->rhoC_1) +
                  cross(D->dang_1, cross(D->dang_1, D->rhoC_1));  // Eq.(5-12)

    delete[] ff;
    delete[] x;
}

void Rocket_Flight_DM::gamma_beta(DM_var *VarIn)
{
    // 1st generalized coordinate q1
    // Velocity coefficient
    VarIn->gamma_b1_q1(0) = 1.0;
    VarIn->gamma_b1_q1(1) = 0.0;
    VarIn->gamma_b1_q1(2) = 0.0;

    // 2nd generalized coordinate q2
    VarIn->gamma_b1_q2(0) = 0.0;
    VarIn->gamma_b1_q2(1) = 1.0;
    VarIn->gamma_b1_q2(2) = 0.0;

    // 3rd generalized coordinate q3
    VarIn->gamma_b1_q3(0) = 0.0;
    VarIn->gamma_b1_q3(1) = 0.0;
    VarIn->gamma_b1_q3(2) = 1.0;

    // Angular velocity coefficients
    VarIn->beta_b1_q4(0) = 1.0;
    VarIn->beta_b1_q4(1) = 0.0;
    VarIn->beta_b1_q4(2) = 0.0;

    // Angular velocity coefficients
    VarIn->beta_b1_q5(0) = 0.0;
    VarIn->beta_b1_q5(1) = 1.0;
    VarIn->beta_b1_q5(2) = 0.0;

    // Angular velocity coefficients
    VarIn->beta_b1_q6(0) = 0.0;
    VarIn->beta_b1_q6(1) = 0.0;
    VarIn->beta_b1_q6(2) = 1.0;
}

void Rocket_Flight_DM::AeroDynamics_Q(LaunchVehicle *VehicleIn)
{
    Prop_var *P;
    DM_var *D;
    Aerodynamics_var *A;
    EarthEnvironment_var *E;
    E = VehicleIn->Env;
    A = VehicleIn->Aero;
    P = VehicleIn->Prop;
    D = VehicleIn->DM;
    double pdynmc = E->pdynmc;
    double refa = A->refa;
    double refd = A->refd;
    double cy = A->cy;
    double cll = A->cll;
    double clm = A->clm;
    double cln = A->cln;
    double cx = A->cx;
    double cz = A->cz;
    arma::vec3 XCG = P->XCG;

    arma::vec3 rhoCG;
    rhoCG(0) = XCG(0) - (D->reference_point);
    rhoCG(1) = 0.0;
    rhoCG(2) = 0.0;

    // total non-gravitational forces
    D->FAPB = pdynmc * refa * arma::vec({ cx, cy, cz });

    // aerodynamic moment
    D->FMAB = pdynmc * refa * refd * arma::vec({ cll, clm, cln });

    D->Q_Aero(0) = dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       D->gamma_b1_q1);
    D->Q_Aero(1) = dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       D->gamma_b1_q2);
    D->Q_Aero(2) = dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       D->gamma_b1_q3);

    /* If the reference point is not on the xcg, the value of rhoCG will not be
   * 0*/
    D->Q_Aero(3) = dot(D->FMAB, D->beta_b1_q4) +
                   dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(rhoCG) * D->beta_b1_q4));
    D->Q_Aero(4) = dot(D->FMAB, D->beta_b1_q5) +
                   dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(rhoCG) * D->beta_b1_q5));
    D->Q_Aero(5) = dot(D->FMAB, D->beta_b1_q6) +
                   dot(QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB),
                       -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(rhoCG) * D->beta_b1_q6));

    D->F_Aero = QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->FAPB);
    D->M_Aero = D->FMAB;
}

void Rocket_Flight_DM::Gravity_Q(LaunchVehicle *VehicleIn)
{
    EarthEnvironment_var *E;
    Prop_var *P;
    DM_var *D;
    D = VehicleIn->DM;
    E = VehicleIn->Env;
    P = VehicleIn->Prop;

    arma::vec3 GRAVG = E->GRAVG;
    double vmass = P->vmass;

    // data_exchang->hget("GRAVG", GRAVG);
    // data_exchang->hget("vmass", &vmass);

    // arma::vec3 Fg;
    D->Fg = vmass * GRAVG;

    if (D->liftoff == 1) {
        D->Q_G(0) = dot(D->Fg, D->gamma_b1_q1);
        D->Q_G(1) = dot(D->Fg, D->gamma_b1_q2);
        D->Q_G(2) = dot(D->Fg, D->gamma_b1_q3);
        D->Q_G(3) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q4));
        D->Q_G(4) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q5));
        D->Q_G(5) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q6));
    } else {
        D->Q_G(0) = dot(D->Fg, D->gamma_b1_q1);
        D->Q_G(1) = dot(D->Fg, D->gamma_b1_q2);
        D->Q_G(2) = dot(D->Fg, D->gamma_b1_q3);
        D->Q_G(3) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q4));
        D->Q_G(4) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q5));
        D->Q_G(5) =
            dot(D->Fg, -QuaternionRotation(QuaternionTranspose(D->TBI_Q),
                                           cross_matrix(D->rhoC_1) * D->beta_b1_q6));
    }
}

void Rocket_Flight_DM::calculate_I1(LaunchVehicle *VehicleIn)
{
    double vmass = VehicleIn->Prop->vmass;
    arma::mat33 IBBB = VehicleIn->Prop->IBBB;

    // data_exchang->hget("vmass", &vmass);
    // data_exchang->hget("IBBB", IBBB);

    VehicleIn->DM->I1(0, 0) =
        IBBB(0, 0) +
        (vmass) * (VehicleIn->DM->rhoC_1(1) * VehicleIn->DM->rhoC_1(1) +
                   VehicleIn->DM->rhoC_1(2) * VehicleIn->DM->rhoC_1(2));
    VehicleIn->DM->I1(0, 1) =
        IBBB(0, 1) - (vmass) *VehicleIn->DM->rhoC_1(0) * VehicleIn->DM->rhoC_1(1);
    VehicleIn->DM->I1(0, 2) =
        IBBB(0, 2) - (vmass) *VehicleIn->DM->rhoC_1(0) * VehicleIn->DM->rhoC_1(2);
    VehicleIn->DM->I1(1, 0) =
        IBBB(1, 0) - (vmass) *VehicleIn->DM->rhoC_1(1) * VehicleIn->DM->rhoC_1(0);
    VehicleIn->DM->I1(1, 1) =
        IBBB(1, 1) +
        (vmass) * (VehicleIn->DM->rhoC_1(2) * VehicleIn->DM->rhoC_1(2) +
                   VehicleIn->DM->rhoC_1(0) * VehicleIn->DM->rhoC_1(0));
    VehicleIn->DM->I1(1, 2) =
        IBBB(1, 2) - (vmass) *VehicleIn->DM->rhoC_1(1) * VehicleIn->DM->rhoC_1(2);
    VehicleIn->DM->I1(2, 0) =
        IBBB(2, 0) - (vmass) *VehicleIn->DM->rhoC_1(2) * VehicleIn->DM->rhoC_1(0);
    VehicleIn->DM->I1(2, 1) =
        IBBB(2, 1) - (vmass) *VehicleIn->DM->rhoC_1(2) * VehicleIn->DM->rhoC_1(1);
    VehicleIn->DM->I1(2, 2) =
        IBBB(2, 2) +
        (vmass) * (VehicleIn->DM->rhoC_1(0) * VehicleIn->DM->rhoC_1(0) +
                   VehicleIn->DM->rhoC_1(1) * VehicleIn->DM->rhoC_1(1));
}

void Rocket_Flight_DM::funcv(int n, double *x, double *ff,
                             LaunchVehicle *VehicleIn)
{
    DM_var *D;
    ACT_var *A;
    A = VehicleIn->ACT;
    D = VehicleIn->DM;
    double m1 = VehicleIn->Prop->vmass;
    arma::vec6 Q_TVC = A->Q_TVC;
    arma::vec6 Q_RCS = A->Q_RCS;
    arma::vec6 Q_G = D->Q_G;
    arma::vec6 Q_Aero = D->Q_Aero;
    // data_exchang->hget("vmass", &m1);
    // data_exchang->hget("Q_TVC", Q_TVC);
    // data_exchang->hget("Q_RCS", Q_RCS);

    for (int i = 0; i < 3; i++) {
        D->ddrP_1(i) = x[i + 1];
        D->ddang_1(i) = x[i + 4];
    }

    D->ddrhoC_1 = cross(D->ddang_1, D->rhoC_1) +
                  cross(D->dang_1, cross(D->dang_1, D->rhoC_1));  // Eq.(5-12)
    //  Eq.(5-19)
    D->p_b1_ga =
        m1 * (D->ddrP_1 +
              QuaternionRotation(QuaternionTranspose(D->TBI_Q), D->ddrhoC_1));
    D->p_b1_be =
        D->I1 * D->ddang_1 + cross_matrix(D->dang_1) * D->I1 * D->dang_1 +
        m1 * cross_matrix(D->rhoC_1) * QuaternionRotation(D->TBI_Q, D->ddrP_1);
    D->f(0) = dot(D->p_b1_ga, D->gamma_b1_q1) -
              (Q_G(0) + Q_TVC(0) + Q_Aero(0) + Q_RCS(0));
    D->f(1) = dot(D->p_b1_ga, D->gamma_b1_q2) -
              (Q_G(1) + Q_TVC(1) + Q_Aero(1) + Q_RCS(1));
    D->f(2) = dot(D->p_b1_ga, D->gamma_b1_q3) -
              (Q_G(2) + Q_TVC(2) + Q_Aero(2) + Q_RCS(2));
    D->f(3) = dot(D->p_b1_be, D->beta_b1_q4) -
              (Q_G(3) + Q_TVC(3) + Q_Aero(3) + Q_RCS(3));
    D->f(4) = dot(D->p_b1_be, D->beta_b1_q5) -
              (Q_G(4) + Q_TVC(4) + Q_Aero(4) + Q_RCS(4));
    D->f(5) = dot(D->p_b1_be, D->beta_b1_q6) -
              (Q_G(5) + Q_TVC(5) + Q_Aero(5) + Q_RCS(5));

    for (int i = 0; i < n; i++) {
        ff[i + 1] = D->f(i);
    }
}

void Rocket_Flight_DM::broydn(double x[], int n, int *check,
                              LaunchVehicle *VehicleIn)
{
    int i, j, k, restrt, sing, skip;
    double den, fff, fold, stpmax, sum, temp, test, *c, *d, *fvcold;
    double *g, *p, **qt, **r, *s, *t, *w, *xold;
    c = dvector(1, n);
    d = dvector(1, n);
    fvcold = dvector(1, n);
    g = dvector(1, n);
    p = dvector(1, n);
    qt = dmatrix(1, n, 1, n);
    r = dmatrix(1, n, 1, n);
    s = dvector(1, n);
    t = dvector(1, n);
    w = dvector(1, n);
    xold = dvector(1, n);
    fvec = dvector(1, n);
    nn = n;
    fff = f_min(x, VehicleIn);
    test = 0.0;
    for (i = 1; i <= n; i++)
        if (fabs(fvec[i]) > test)
            test = fabs(fvec[i]);
    if (test < 0.01 * TOLF) {
        *check = 0;
        FREERETURN
    }
    for (sum = 0.0, i = 1; i <= n; i++)
        sum += DSQR(x[i]);
    stpmax = STPMX * DMAX(sqrt(sum), (double) n);
    restrt = 1;
    for (its = 1; its <= MAXITS; its++) {
        if (restrt) {
            fdjac(n, x, fvec, r, VehicleIn);
            qrdcmp(r, n, c, d, &sing);
            if (sing)
                nrerror("singular Jacobian in broydn : qrdcmp");
            for (i = 1; i <= n; i++) {
                for (j = 1; j <= n; j++)
                    qt[i][j] = 0.0;
                qt[i][i] = 1.0;
            }
            for (k = 1; k < n; k++) {
                if (c[k]) {
                    for (j = 1; j <= n; j++) {
                        sum = 0.0;
                        for (i = k; i <= n; i++)
                            sum += r[i][k] * qt[i][j];
                        sum /= c[k];
                        for (i = k; i <= n; i++)
                            qt[i][j] -= sum * r[i][k];
                    }
                }
            }
            for (i = 1; i <= n; i++) {
                r[i][i] = d[i];
                for (j = 1; j < i; j++)
                    r[i][j] = 0.0;
            }
        } else {
            for (i = 1; i <= n; i++)
                s[i] = x[i] - xold[i];
            for (i = 1; i <= n; i++) {
                for (sum = 0.0, j = i; j <= n; j++)
                    sum += r[i][j] * s[j];
                t[i] = sum;
            }
            skip = 1;
            for (i = 1; i <= n; i++) {
                for (sum = 0.0, j = 1; j <= n; j++)
                    sum += qt[j][i] * t[j];
                w[i] = fvec[i] - fvcold[i] - sum;
                if (fabs(w[i]) >= EPS * (fabs(fvec[i]) + fabs(fvcold[i]))) {
                    skip = 0;
                } else {
                    w[i] = 0.0;
                }
            }
            if (!skip) {
                for (i = 1; i <= n; i++) {
                    for (sum = 0.0, j = 1; j <= n; j++)
                        sum += qt[i][j] * w[j];
                    t[i] = sum;
                }
                for (den = 0.0, i = 1; i <= n; i++)
                    den += DSQR(s[i]);
                for (i = 1; i <= n; i++)
                    s[i] /= den;
                qrupdt(r, qt, n, t, s);
                for (i = 1; i <= n; i++) {
                    if (r[i][i] == 0.0)
                        nrerror("r singular in broydn : qrupdt");
                    d[i] = r[i][i];
                }
            }
        }
        for (i = 1; i <= n; i++) {
            for (sum = 0.0, j = 1; j <= n; j++)
                sum += qt[i][j] * fvec[j];
            p[i] = -sum;
        }
        for (i = n; i >= 1; i--) {
            for (sum = 0.0, j = 1; j <= i; j++)
                sum -= r[j][i] * p[j];
            g[i] = sum;
        }
        for (i = 1; i <= n; i++) {
            xold[i] = x[i];
            fvcold[i] = fvec[i];
        }
        fold = fff;
        rsolv(r, n, d, p);
        lnsrch(n, xold, fold, g, p, x, &fff, stpmax, check, VehicleIn);
        test = 0.0;
        for (i = 1; i <= n; i++)
            if (fabs(fvec[i]) > test)
                test = fabs(fvec[i]);
        if (test < TOLF) {
            *check = 0;
            FREERETURN
        }
        if (*check) {
            if (restrt) {
                FREERETURN
            } else {
                test = 0.0;
                den = DMAX(fff, 0.5 * n);
                for (i = 1; i <= n; i++) {
                    temp = fabs(g[i]) * DMAX(fabs(x[i]), 1.0) / den;
                    if (temp > test) {
                        test = temp;
                    }
                }
                if (test < TOLMIN) {
                    FREERETURN
                } else {
                    restrt = 1;
                }
            }
        } else {
            restrt = 0;
            test = 0.0;
            for (i = 1; i <= n; i++) {
                temp = (fabs(x[i] - xold[i])) / DMAX(fabs(x[i]), 1.0);
                if (temp > test)
                    test = temp;
            }
            if (test < EPS)
                FREERETURN
        }
    }
    nrerror("MAXITS exceeded in broydn");
    FREERETURN
}

void Rocket_Flight_DM::rsolv(double **a, int n, double d[], double b[])
{
    int i, j;
    double sum;

    b[n] /= d[n];
    for (i = n - 1; i >= 1; i--) {
        for (sum = 0.0, j = i + 1; j <= n; j++)
            sum += a[i][j] * b[j];
        b[i] = (b[i] - sum) / d[i];
    }
}

void Rocket_Flight_DM::fdjac(int n, double x[], double fvec_in[], double **df,
                             LaunchVehicle *VehicleIn)
{
    int i, j;
    double h, temp, *ff;

    ff = dvector(1, n);
    for (j = 1; j <= n; j++) {
        temp = x[j];
        h = EPS * fabs(temp);
        if (h == 0.0)
            h = EPS;
        x[j] = temp + h;
        h = x[j] - temp;
        funcv(n, x, ff, VehicleIn);
        x[j] = temp;
        for (i = 1; i <= n; i++) {
            df[i][j] = (ff[i] - fvec_in[i]) / h;
        }
    }
    free_dvector(ff, 1, n);
}

double Rocket_Flight_DM::f_min(double x[], LaunchVehicle *VehicleIn)
{
    int i;
    double sum;

    funcv(nn, x, fvec, VehicleIn);
    for (sum = 0.0, i = 1; i <= nn; i++)
        sum += DSQR(fvec[i]);
    return 0.5 * sum;
}

void Rocket_Flight_DM::lnsrch(int n, double xold[], double fold, double g[],
                              double p[], double x[], double *f_in,
                              double stpmax, int *check,
                              LaunchVehicle *VehicleIn)
{
    int i;
    double a, alam, alam2, alamin, b, disc, f2, rhs1, rhs2, slope, sum, temp,
        test, tmplam;

    *check = 0;
    for (sum = 0.0, i = 1; i <= n; i++)
        sum += p[i] * p[i];
    sum = sqrt(sum);
    if (sum > stpmax) {
        for (i = 1; i <= n; i++)
            p[i] *= stpmax / sum;
    }
    for (slope = 0.0, i = 1; i <= n; i++)
        slope += g[i] * p[i];
    if (slope >= 0.0) {
        nrerror("Roundoff problem in lnsrch.");
    }
    test = 0.0;
    for (i = 1; i <= n; i++) {
        temp = fabs(p[i]) / DMAX(fabs(xold[i]), 1.0);
        if (temp > test) {
            test = temp;
        }
    }
    alamin = TOLX / test;
    alam = 1.0;
    for (;;) {
        for (i = 1; i <= n; i++)
            x[i] = xold[i] + alam * p[i];
        *f_in = f_min(x, VehicleIn);
        if (alam < alamin) {
            for (i = 1; i <= n; i++)
                x[i] = xold[i];
            *check = 1;
            return;
        } else if (*f_in <= fold + ALF * alam * slope) {
            return;
        } else {
            if (alam == 1.0) {
                tmplam = -slope / (2.0 * (*f_in - fold - slope));
            } else {
                rhs1 = *f_in - fold - alam * slope;
                rhs2 = f2 - fold - alam2 * slope;
                a = (rhs1 / (alam * alam) - rhs2 / (alam2 * alam2)) / (alam - alam2);
                b = (-alam2 * rhs1 / (alam * alam) + alam * rhs2 / (alam2 * alam2)) /
                    (alam - alam2);
                if (a == 0.0) {
                    tmplam = -slope / (2.0 * b);
                } else {
                    disc = b * b - 3.0 * a * slope;
                    if (disc < 0.0) {
                        tmplam = 0.5 * alam;
                    } else if (b <= 0.0) {
                        tmplam = (-b + sqrt(disc)) / (3.0 * a);
                    } else {
                        tmplam = -slope / (b + sqrt(disc));
                    }
                }
                if (tmplam > 0.5 * alam) {
                    tmplam = 0.5 * alam;
                }
            }
        }
        alam2 = alam;
        f2 = *f_in;
        alam = DMAX(tmplam, 0.1 * alam);
    }
}

void Rocket_Flight_DM::qrdcmp(double **a, int n, double *c, double *d,
                              int *sing)
{
    int i, j, k;
    double scale, sigma, sum, tau;

    *sing = 0;
    for (k = 1; k < n; k++) {
        scale = 0.0;
        for (i = k; i <= n; i++)
            scale = DMAX(scale, fabs(a[i][k]));
        if (scale == 0.0) {
            *sing = 1;
            c[k] = d[k] = 0.0;
        } else {
            for (i = k; i <= n; i++)
                a[i][k] /= scale;
            for (sum = 0.0, i = k; i <= n; i++)
                sum += DSQR(a[i][k]);
            sigma = SIGN(sqrt(sum), a[k][k]);
            a[k][k] += sigma;
            c[k] = sigma * a[k][k];
            d[k] = -scale * sigma;
            for (j = k + 1; j <= n; j++) {
                for (sum = 0.0, i = k; i <= n; i++)
                    sum += a[i][k] * a[i][j];
                tau = sum / c[k];
                for (i = k; i <= n; i++)
                    a[i][j] -= tau * a[i][k];
            }
        }
    }
    d[n] = a[n][n];
    if (d[n] == 0.0)
        *sing = 1;
}

void Rocket_Flight_DM::qrupdt(double **r, double **qt, int n, double u[],
                              double v[])
{
    int i, j, k;

    for (k = n; k >= 1; k--) {
        if (u[k])
            break;
    }
    if (k < 1)
        k = 1;
    for (i = k - 1; i >= 1; i--) {
        rotate(r, qt, n, i, u[i], -u[i + 1]);
        if (u[i] == 0.0) {
            u[i] = fabs(u[i + 1]);
        } else if (fabs(u[i]) > fabs(u[i + 1])) {
            u[i] = fabs(u[i]) * sqrt(1.0 + DSQR(u[i + 1] / u[i]));
        } else {
            u[i] = fabs(u[i + 1]) * sqrt(1.0 + DSQR(u[i] / u[i + 1]));
        }
    }
    for (j = 1; j <= n; j++)
        r[1][j] += u[1] * v[j];
    for (i = 1; i < k; i++)
        rotate(r, qt, n, i, r[i][i], -r[i + 1][i]);
}

void Rocket_Flight_DM::rotate(double **r, double **qt, int n, int i, double a,
                              double b)
{
    int j;
    double c, fact, s, w, y;

    if (a == 0.0) {
        c = 0.0;
        s = (b >= 0.0 ? 1.0 : -1.0);
    } else if (fabs(a) > fabs(b)) {
        fact = b / a;
        c = SIGN(1.0 / sqrt(1.0 + (fact * fact)), a);
        s = fact * c;
    } else {
        fact = a / b;
        s = SIGN(1.0 / sqrt(1.0 + (fact * fact)), b);
        c = fact * s;
    }
    for (j = i; j <= n; j++) {
        y = r[i][j];
        w = r[i + 1][j];
        r[i][j] = c * y - s * w;
        r[i + 1][j] = s * y + c * w;
    }
    for (j = 1; j <= n; j++) {
        y = qt[i][j];
        w = qt[i + 1][j];
        qt[i][j] = c * y - s * w;
        qt[i + 1][j] = s * y + c * w;
    }
}

// int Rocket_Flight_DM::enqueue_to_simgen_buffer(struct icf_ctrlblk_t *C,
//                                                double ext_porlation) {
//   struct simgen_motion_data_t motion_info;
//   double(*pos)[3];
//   double(*vel)[3];
//   double(*accel)[3];
//   pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
//   vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
//   accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
//   motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
//   motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
//   motion_info.vehicle_id = 1;
//   memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
//   memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
//   memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
//   motion_info.jerk_xyz[0] = 0.0;
//   motion_info.jerk_xyz[1] = 0.0;
//   motion_info.jerk_xyz[2] = 0.0;
//   motion_info.heb[0] = psibd;
//   motion_info.heb[1] = thtbd;
//   motion_info.heb[2] = phibd;
//   memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
//   motion_info.angular_acceleration[0] = 0.0;
//   motion_info.angular_acceleration[1] = 0.0;
//   motion_info.angular_acceleration[2] = 0.0;
//   motion_info.angular_jerk[0] = 0.0;
//   motion_info.angular_jerk[1] = 0.0;
//   motion_info.angular_jerk[2] = 0.0;
//   icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info,
//                  sizeof(struct simgen_motion_data_t));
//   return 0;
// }

// int Rocket_Flight_DM::stand_still_motion_data(struct icf_ctrlblk_t *C,
//                                               double ext_porlation) {
//   struct simgen_motion_data_t motion_info;
//   double(*pos)[3];
//   double(*vel)[3];
//   double(*accel)[3];
//   pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
//   vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
//   accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
//   motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
//   motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
//   motion_info.vehicle_id = 1;
//   memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
//   memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
//   memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
//   motion_info.jerk_xyz[0] = 0.0;
//   motion_info.jerk_xyz[1] = 0.0;
//   motion_info.jerk_xyz[2] = 0.0;
//    Heading, Yaw, Z axis, Down
//   motion_info.heb[0] = 1.5707963268;
//   /* Elevation, Pitch, Y axis, East */
//   motion_info.heb[1] = 1.5707963268;
//   /* Bank, Roll, X axis, North */
//   motion_info.heb[2] = 0.0;
//   memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
//   motion_info.angular_acceleration[0] = 0.0;
//   motion_info.angular_acceleration[1] = 0.0;
//   motion_info.angular_acceleration[2] = 0.0;
//   motion_info.angular_jerk[0] = 0.0;
//   motion_info.angular_jerk[1] = 0.0;
//   motion_info.angular_jerk[2] = 0.0;
//   icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info,
//                  sizeof(struct simgen_motion_data_t));
//   return 0;
// }

// void Rocket_Flight_DM::Interpolation_Extrapolation(double T, double int_step,
//                                                    double ext_porlation) {
//   arma::vec3 A, B, C, D, E, F;
//   // double t;
//   // Interpolation_Extrapolation_flag++;
//   // if (Interpolation_Extrapolation_flag == 5) {
//   //     Interpolation_Extrapolation_flag = 0;
//   // }
//   // t = Interpolation_Extrapolation_flag * int_step;

//   A = SBEE_old;
//   B = VBEE_old;
//   // C = ABEE_old / 2.0;
//   // D = (-20.0 * SBEE_old + 20.0 * SBEE - 12.0 * T * VBEE_old - 8.0 * T *
//   VBEE
//   // - 3.0 * T * T * ABEE_old + T * T * ABEE) / (2.0 * T * T * T); E = (30.0
//   *
//   // SBEE_old - 30.0 * SBEE + 16.0 * T * VBEE_old + 14.0 * T * VBEE + 3.0 * T
//   *
//   // T * ABEE_old - 2.0 * T * T * ABEE) / (2.0 * T * T * T * T); F = (-12.0 *
//   // SBEE_old + 12.0 * SBEE - 6.0 * T * VBEE_old - 6.0 * T * VBEE - T * T *
//   // ABEE_old + T * T * ABEE) / (2.0 * T * T * T * T * T);

//   // SBEE_test = A + B * t + C * t * t + D * t * t * t
//   //             + E * t * t * t * t + F * t * t * t * t * t;
//   // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t + 4.0 * E * t * t * t
//   //             + 5.0 * F * t * t * t * t;
//   // ABEE_test = 2.0 * C + 6.0 * D * t + 12.0 * E * t * t + 20.0 * F * t * t
//   *
//   // t;

//   // C = 3.0 * (SBEE - SBEE_old) / (T * T) - (VBEE + 2.0 * VBEE_old) / T;
//   // D = 2.0 * (SBEE_old - SBEE) / (T * T * T) + (VBEE + VBEE_old) / (T * T);

//   // SBEE_test = A + B * t + C * t * t + D * t * t * t;
//   // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t
//   SBEE_test = SBEE + ((SBEE - A) / T) * ext_porlation * 1000.0 * int_step;
//   VBEE_test = VBEE + ((VBEE - B) / T) * ext_porlation * 1000.0 * int_step;
// }

#undef MAXITS
#undef EPS
#undef TOLF
#undef TOLMIN
#undef TOLX
#undef STPMX
#undef FREERETURN
#undef NRANSI
