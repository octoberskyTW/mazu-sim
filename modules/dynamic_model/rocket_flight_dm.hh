#ifndef __ROCKET_FLIGHT_DM_HH__
#define __ROCKET_FLIGHT_DM_HH__

#include <armadillo>
#include <memory>
#include "aux.hh"
#include "cadac_constants.hh"
#include "numerical_constants.hh"
#include "time_management.hh"
#include "vehicle.hh"
#include "vehicle_var.hh"
#include "Dynamics_System.hh"

class Rocket_Flight_DM : public FH_module
{
    TRICK_INTERFACE(Rocket_Flight_DM);

public:
    Rocket_Flight_DM();
    //struct icf_ctrlblk_t *dm_icf_info_hook;
    // int enqueue_to_simgen_buffer(struct icf_ctrlblk_t *C, double
    // ext_porlation); int stand_still_motion_data(struct icf_ctrlblk_t *C, double
    // ext_porlation);

    virtual void init(LaunchVehicle *VehicleIn);
    virtual void algorithm(LaunchVehicle *VehicleIn);

    void set_reference_point_eq_xcg();

    struct TX_data {
        double SBEE[3];
        double VBEE[3];
        double ABEE[3];
        double JBEE[3];
        double psibd;
        double thtbd;
        double phibd;
        double WBEB[3];
    } TX_data_forward;

    void set_DOF(int ndof);
    void set_aero_flag(unsigned int in);

private:
    // void propagate_position_speed_acceleration(double int_step);
    void propagate_aeroloss(LaunchVehicle *VehicleIn);
    void propagate_gravityloss(LaunchVehicle *VehicleIn);
    void propagate_control_loss(LaunchVehicle *VehicleIn);
    void orbital(DM_var *VarIn);
    void aux_calulate(arma::mat33 TEI, double int_step,
                                    DM_var *DMIn, EarthEnvironment_var *EnvIn);
    void RK4F(std::vector<arma::vec> Var_in, std::vector<arma::vec> &Var_out,
              LaunchVehicle *VehicleIn);
    void reference_point_calc(DM_var *D, Prop_var *P);
    void collect_forces_and_propagate(LaunchVehicle *VehicleIn);

    double calculate_alphaix(arma::vec3 VBIB);
    double calculate_betaix(arma::vec3 VBIB);
    double calculate_alppx(arma::vec3 VBAB_in, double dvba);
    double calculate_phipx(arma::vec3 VBAB_in);
    double calculate_alphax(arma::vec3 VBAB_in);
    double calculate_betax(arma::vec3 VBAB, double dvba);

    arma::vec build_VBEB(double _alpha0x, double _beta0x, double dvbe);
    arma::mat calculate_TBD(LaunchVehicle *VehicleIn);

    void gamma_beta(DM_var *VarIn);
    void Gravity_Q(LaunchVehicle *VehicleIn);
    void AeroDynamics_Q(LaunchVehicle *VehicleIn);
    void calculate_I1(LaunchVehicle *VehicleIn);
    void funcv(int n, double *x, double *ff, LaunchVehicle *VehicleIn);
    void broydn(double x[], int n, int *check, LaunchVehicle *VehicleIn);
    void rsolv(double **a, int n, double d[], double b[]);
    void fdjac(int n, double x[], double fvec_in[], double **df,
               LaunchVehicle *VehicleIn);
    double f_min(double x[], LaunchVehicle *VehicleIn);
    void lnsrch(int n, double xold[], double fold, double g[], double p[],
                double x[], double *f_in, double stpmax, int *check,
                LaunchVehicle *VehicleIn);
    void qrdcmp(double **a, int n, double *c, double *d, int *sing);
    void qrupdt(double **r, double **qt, int n, double u[], double v[]);
    void rotate(double **r, double **qt, int n, int i, double a, double b);

    unsigned int Interpolation_Extrapolation_flag;
    int its;                  /* *o (--) Number of iterations */
    int DOF;                  /* *o (--)  Number of Degree of Freedom */
    int reference_point_flag; /* *o (--)  check if reference point equal to xcg */
    unsigned int Aero_flag;   /* *o (-)  Aerodynamics flag */
    
    Dynamics_Sys *sys_ptr;
};

template <typename T>
void IntegratorRK4(std::vector<arma::vec> V_in, std::vector<arma::vec> &V_out,
                   void (T::*fp)(std::vector<arma::vec> Var_in,
                                 std::vector<arma::vec> &Var_out,
                                 LaunchVehicle *VehicleIn),
                   T *ClassPointer, LaunchVehicle *VehicleIn, double int_step)
{
    {
        std::vector<std::vector<arma::vec>> KMAT;
        for (unsigned int i = 0; i < V_in.size(); i++) {
            std::vector<arma::vec> KROW(4);
            KMAT.push_back(KROW);
        }

        V_out = V_in;

        ((ClassPointer)->*fp)(V_out, KMAT[0], VehicleIn);

        for (unsigned int i = 0; i < V_in.size(); i++) {
            V_out[i] = V_in[i] + KMAT[0][i] * 0.5 * int_step;
        }

        ((ClassPointer)->*fp)(V_out, KMAT[1], VehicleIn);

        for (unsigned int i = 0; i < V_in.size(); i++) {
            V_out[i] = V_in[i] + KMAT[1][i] * 0.5 * int_step;
        }

        ((ClassPointer)->*fp)(V_out, KMAT[2], VehicleIn);

        for (unsigned int i = 0; i < V_in.size(); i++) {
            V_out[i] = V_in[i] + KMAT[2][i] * int_step;
        }

        ((ClassPointer)->*fp)(V_out, KMAT[3], VehicleIn);

        for (unsigned int i = 0; i < V_in.size(); i++) {
            V_out[i] = V_in[i] + (int_step / 6.0) * (KMAT[0][i] + 2.0 * KMAT[1][i] +
                                                     2.0 * KMAT[2][i] + KMAT[3][i]);
        }
    }
}

template <typename T>
void IntegratorEuler(std::vector<arma::vec> V_in, std::vector<arma::vec> &V_out,
                     void (T::*fp)(std::vector<arma::vec> Var_in,
                                   std::vector<arma::vec> &Var_out,
                                   LaunchVehicle *VehicleIn),
                     T *ClassPointer, LaunchVehicle *VehicleIn,
                     double int_step)
{
    std::vector<arma::vec> K_TEMP;

    for (unsigned int i = 0; i < V_in.size(); i++) {
        arma::vec temp;
        K_TEMP.push_back(temp);
    }

    ((ClassPointer)->*fp)(V_in, K_TEMP, VehicleIn);

    for (unsigned int i = 0; i < V_in.size(); i++) {
        V_out[i] = V_in[i] + K_TEMP[i] * int_step;
    }
}
#endif  //  __ROCKET_FLIGHT_DM_HH__
