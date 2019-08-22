
#include <iostream>
#include "aerodynamics.hh"
AeroDynamics::AeroDynamics() {}

void AeroDynamics::load_aerotable(const char *filename)
{
    aerotable = Datadeck(filename);
}

void AeroDynamics::algorithm(LaunchVehicle *VehicleIn)
{
    /* only calculate when rocket liftoff */
    Aerodynamics_var *A;
    DM_var *D;
    EarthEnvironment_var *E;
    Prop_var *P;
    P = VehicleIn->Prop;
    D = VehicleIn->DM;
    A = VehicleIn->Aero;
    E = VehicleIn->Env;

    int liftoff = D->liftoff;
    int thrust_on = 0;
    // data_exchang->hget("liftoff", &liftoff);

    if (liftoff == 1) {
        double vmach = E->vmach;
        double dvba = E->dvba;
        double alppx = D->alppx;
        double phipx = D->phipx;

        arma::vec3 WBIB = D->WBIB;
        arma::vec3 WBEB = D->WBEB;
        arma::vec3 XCG = P->XCG;

        enum THRUST_TYPE thrust_state = VehicleIn->Prop->thrust_state;

        //  transforming body rates from body -> aeroballistic coord.
        double phip = phipx * RAD;
        double cphip = cos(phip);
        double sphip = sin(phip);
        double qqax = WBEB(1) * cphip - WBEB(2) * sphip;

        // looking up axial force coefficients
        A->ca0 = aerotable.look_up("ca0_vs_mach", vmach, 0);
        A->caa = aerotable.look_up("caa_vs_mach", vmach, 0);
        A->ca0b = aerotable.look_up("ca0b_vs_mach", vmach, 0);

        // axial force coefficient
        if ((thrust_state = NO_THRUST))
            thrust_on = 1;
        A->ca = A->ca0 + A->caa * alppx + thrust_on * A->ca0b;

        // looking up normal force coefficients in aeroballistic coord
        A->cn0 = aerotable.look_up("cn0_vs_mach_alpha", vmach, alppx, 0);
        // normal force coefficient
        A->cna = A->cn0;

        // looking up pitching moment coefficients in aeroballistic coord
        A->clm0 = aerotable.look_up("clm0_vs_mach_alpha", vmach, alppx, 0);
        A->clmq = aerotable.look_up("clmq_vs_mach", vmach, 0);
        // pitching moment coefficient
        double clmaref = A->clm0 + A->clmq * qqax * A->refd / (2. * dvba);
        A->clma = clmaref - A->cna * (A->XCP(0) + XCG(0)) / A->refd;

        // converting force and moment coeff to body axes
        // force coefficients in body axes
        A->cx = -A->ca;
        A->cy = -A->cna * sphip;
        A->cz = -A->cna * cphip;
        // moment coefficient in body axes
        A->cll = 0;
        A->clm = A->clma * cphip;
        A->cln = -A->clma * sphip;
    }
}
