#include "integrate.hh"
#include "matrix_tool.hh"
#include "tvc.hh"

TVC::TVC() {}

void TVC::init(LaunchVehicle *VehicleIn) {}

void TVC::algorithm(LaunchVehicle *VehicleIn)
{
    DM_var *D;
    ACT_var *A;
    Prop_var *P;
    P = VehicleIn->Prop;
    A = VehicleIn->ACT;
    D = VehicleIn->DM;
    double int_step = VehicleIn->dt;

    // input from other modules
    A->theta_a_cmd = grab_theta_a_cmd();
    A->theta_b_cmd = grab_theta_b_cmd();
    A->theta_c_cmd = grab_theta_c_cmd();
    A->theta_d_cmd = grab_theta_d_cmd();

    double thrust = P->thrust;
    arma::mat33 TBI = D->TBI;
    arma::vec3 XCG = P->XCG;
    switch (A->mtvc) {
    case NO_TVC:
        // return if no tvc
        // thrust forces in body axes
        return;
        break;
    case S1_TVC:
        A->Q_TVC.zeros();
        A->F_TVC.zeros();
        A->M_TVC.zeros();
        VehicleIn->S1_Eng_list[0]->Act_list[0]->Actuate(A->theta_a_cmd,
                                                        int_step);
        VehicleIn->S1_Eng_list[1]->Act_list[0]->Actuate(A->theta_b_cmd,
                                                        int_step);
        VehicleIn->S1_Eng_list[2]->Act_list[0]->Actuate(A->theta_c_cmd,
                                                        int_step);
        VehicleIn->S1_Eng_list[3]->Act_list[0]->Actuate(A->theta_d_cmd,
                                                        int_step);
        
        for (unsigned int i = 0; i < VehicleIn->S1_Eng_list.size(); i++) {
            VehicleIn->S1_Eng_list[i]->calculate_Q(
                VehicleIn->S1_Eng_list[i]->Act_list[0]->ActOuptut,
                thrust, TBI, XCG(0), VehicleIn->S1_Eng_list[i]->type);
        }
        
        for (unsigned int i = 0; i < VehicleIn->S1_Eng_list.size(); i++) {
            A->Q_TVC += VehicleIn->S1_Eng_list[i]->Q;
            A->F_TVC += VehicleIn->S1_Eng_list[i]->F;
            A->M_TVC += VehicleIn->S1_Eng_list[i]->M;
        }
        return;
        break;
    default:
        break;
    }
}
