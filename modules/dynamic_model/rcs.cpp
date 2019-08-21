#include "rcs.hh"

RCS::RCS() {}

void RCS::init(LaunchVehicle *VehicleIn){};

void RCS::algorithm(LaunchVehicle *VehicleIn)
{
    ACT_var *A;
    DM_var *D;
    D = VehicleIn->DM;
    A = VehicleIn->ACT;
    // arma::mat33 TBI;
    // data_exchang->hget("TBI", TBI);
    double e_roll = grab_e_roll();
    double e_pitch = grab_e_pitch();
    double e_yaw = grab_e_yaw();

    A->Q_RCS.zeros();

    VehicleIn->Thruster_list[0]->calculate_Torque_Q(
        e_roll, D->TBI, VehicleIn->Thruster_list[0]->mode);
    VehicleIn->Thruster_list[1]->calculate_Torque_Q(
        e_pitch, D->TBI, VehicleIn->Thruster_list[1]->mode);
    VehicleIn->Thruster_list[2]->calculate_Torque_Q(
        e_yaw, D->TBI, VehicleIn->Thruster_list[2]->mode);

    for (unsigned int i = 0; i < VehicleIn->Thruster_list.size(); i++) {
        A->Q_RCS += VehicleIn->Thruster_list[i]->Q;
    }

    // data_exchang->hset("Q_RCS", A->Q_RCS);
}
