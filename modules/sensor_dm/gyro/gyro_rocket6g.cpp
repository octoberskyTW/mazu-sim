#include <armadillo>
#include <cstring>
#include <ctime>
#include "gyro_rocket6g.hh"
#include "matrix_tool.hh"
#include "stochastic.hh"



GyroRocket6G::GyroRocket6G()
{
    snprintf(name, sizeof(name), "Rocket6G Gyro Sensor Model");
    srand(static_cast<unsigned int>(time(NULL)));
}

void GyroRocket6G::algorithm(LaunchVehicle *VehicleIn)
{
    arma::vec3 WBIB = VehicleIn->DM->WBIB;

    //-------------------------------------------------------------------------
    // ARW RRW
    double sig(1.0);
    double RRW(0.0130848811);  // 0.4422689813  7.6072577e-3
    double ARW(0.2828427125);  // 0.07071067812  7.90569415e-3
    double Freq(200.0);

    for (int i = 0; i < 3; i++) {
        VehicleIn->Sensor->ITA2_G(i) =
            gauss(0, 1.0) * RRW * RAD;  // distribution(generator) * RRW * RAD;
        VehicleIn->Sensor->BETA_G(i) = 0.9999 * VehicleIn->Sensor->BETA_G(i) +
                                       VehicleIn->Sensor->ITA2_G(i) * VehicleIn->dt;
        VehicleIn->Sensor->ITA1_G(i) = gauss(0, 1.0) *
                                       (ARW * sqrt(Freq) / 60. * (1 / sig)) *
                                       RAD;  // distribution(generator) * (ARW *
                                             // sqrt(Freq) / 60 * (1 / sig)) * RAD;
    }

    // combining all uncertainties
    VehicleIn->Sensor->EWBIB = VehicleIn->Sensor->ITA1_G +
                               VehicleIn->Sensor->BETA_G;  // EMSBG + EUG + EWG;

    VehicleIn->Sensor->WBICB = WBIB + VehicleIn->Sensor->EWBIB;

    return;
}
