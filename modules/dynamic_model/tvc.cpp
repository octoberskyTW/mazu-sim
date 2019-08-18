#include "integrate.hh"
#include "matrix_tool.hh"
#include "tvc.hh"

TVC::TVC() {}

void TVC::default_data() {}

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

    double thrust = P->thrust;
    arma::mat33 TBI = D->TBI;
    arma::vec3 XCG = P->XCG;
    // data_exchang->hget("TBI", TBI);
    // data_exchang->hget("thrust", &thrust);
    // data_exchang->hget("XCG", XCG);
    switch (A->mtvc) {
    case NO_TVC:
        // return if no tvc
        // thrust forces in body axes
        // data_exchang->hset("Q_TVC", A->Q_TVC);
        return;
        break;
    case S1_TVC:
        A->Q_TVC.zeros();
        VehicleIn->S1_Eng_list[0]->Act_list[0]->Actuate(A->theta_a_cmd,
                                                        int_step);  // Pitch angle
        VehicleIn->S1_Eng_list[0]->Act_list[1]->Actuate(A->theta_b_cmd,
                                                        int_step);  // Yaw angle

        VehicleIn->S1_Eng_list[0]->calculate_Q(
            VehicleIn->S1_Eng_list[0]->Act_list[1]->ActOuptut,
            VehicleIn->S1_Eng_list[0]->Act_list[0]->ActOuptut, thrust, TBI,
            XCG(0), VehicleIn->S1_Eng_list[0]->type);

        for (unsigned int i = 0; i < VehicleIn->S1_Eng_list.size(); i++) {
            A->Q_TVC += VehicleIn->S1_Eng_list[i]->Q;
        }
        // data_exchang->hset("Q_TVC", A->Q_TVC);
        return;
        break;
    case S2_TVC:
        A->Q_TVC.zeros();
        VehicleIn->S2_Eng_list[0]->Act_list[0]->Actuate(0.0,
                                                        int_step);  // Pitch angle
        VehicleIn->S2_Eng_list[0]->Act_list[1]->Actuate(0.0,
                                                        int_step);  // Yaw angle

        VehicleIn->S2_Eng_list[0]->calculate_Q(
            VehicleIn->S2_Eng_list[0]->Act_list[1]->ActOuptut,
            VehicleIn->S2_Eng_list[0]->Act_list[0]->ActOuptut, thrust, TBI,
            XCG(0), VehicleIn->S2_Eng_list[0]->type);

        for (unsigned int i = 0; i < VehicleIn->S2_Eng_list.size(); i++) {
            A->Q_TVC += VehicleIn->S2_Eng_list[i]->Q;
        }
        // data_exchang->hset("Q_TVC", A->Q_TVC);
        return;
        break;
    case S3_TVC:
        A->Q_TVC.zeros();
        VehicleIn->S3_Eng_list[0]->Act_list[0]->Actuate(0.0, int_step);
        VehicleIn->S3_Eng_list[0]->Act_list[1]->Actuate(0.0, int_step);

        VehicleIn->S3_Eng_list[0]->calculate_Q(
            VehicleIn->S3_Eng_list[0]->Act_list[1]->ActOuptut,
            VehicleIn->S3_Eng_list[0]->Act_list[0]->ActOuptut, thrust, TBI,
            -XCG(0), VehicleIn->S3_Eng_list[0]->type);

        for (unsigned int i = 0; i < VehicleIn->S3_Eng_list.size(); i++) {
            A->Q_TVC += VehicleIn->S3_Eng_list[i]->Q;
        }
        // data_exchang->hset("Q_TVC", A->Q_TVC);
        return;
        break;
    default:
        // data_exchang->hset("Q_TVC", A->Q_TVC);
        break;
    }
}
