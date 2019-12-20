
#include "ins.h"
#include "sensor_calibration.h"

SensorCalibration::SensorCalibration()
{
    double exp = exp2(-15.0);
    printf("Enter function:%s\n", __FUNCTION__);
    this->temper_const = 0.00565;
    this->step_time = 0.05;

    this->gyro_transf.high_const = 0.02;
    this->gyro_transf.low_const = 0.01 * exp;
    this->accel_transf.high_const = 0.8;
    this->accel_transf.low_const = 0.4 * exp;

    this->gyro_transf.step_unit_const = this->step_time * DegtoRad;
    this->accel_transf.step_unit_const = this->step_time * MGtoNsec;
    printf("Leave function:%s\n", __FUNCTION__);
}

SensorCalibration::~SensorCalibration()
{
}

int SensorCalibration::loadGyroTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot)
{
    this->gyro_transf.bias = bias;
    this->gyro_transf.scale_rot = scale_rot;
    return 0;
}

int SensorCalibration::loadAccelTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot)
{
    this->accel_transf.bias = bias;
    this->accel_transf.scale_rot = scale_rot;
    return 0;
}

int SensorCalibration::loadGyroTransf(struct transf_t *transf)
{
    this->gyro_transf.bias = transf->bias;
    this->gyro_transf.scale_rot = transf->scale_rot;
    return 0;
}

int SensorCalibration::loadAccelTransf(struct transf_t *transf)
{
    this->accel_transf.bias = transf->bias;
    this->accel_transf.scale_rot = transf->scale_rot;
    return 0;
}

int SensorCalibration::update(void *pdata, struct calibration_src_t *package)
{
    struct ins_data_t *ins_data = (struct ins_data_t *) pdata;
    arma::vec gyro = arma::vec(3, arma::fill::zeros);
    arma::vec accel = arma::vec(3, arma::fill::zeros);
    std::cout << "Enter update!\n";

    double gyro_high_const = this->gyro_transf.high_const;
    double gyro_low_const = this->gyro_transf.low_const;
    double accel_high_const = this->accel_transf.high_const;
    double accel_low_const = this->accel_transf.low_const;

    gyro(0) = ((int32_t) package->gyroXh) * gyro_high_const + ((int32_t) package->gyroXl) * gyro_low_const;
    gyro(1) = ((int32_t) package->gyroYh) * gyro_high_const + ((int32_t) package->gyroYl) * gyro_low_const;
    gyro(2) = ((int32_t) package->gyroZh) * gyro_high_const + ((int32_t) package->gyroZl) * gyro_low_const;
    accel(0) = ((int32_t) package->accelXh) * accel_high_const + ((int32_t) package->accelXl) * accel_low_const;
    accel(1) = ((int32_t) package->accelYh) * accel_high_const + ((int32_t) package->accelYl) * accel_low_const;
    accel(2) = ((int32_t) package->accelZh) * accel_high_const + ((int32_t) package->accelZl) * accel_low_const;

    ins_data->sensor_gyro = gyro;
    ins_data->sensor_accel = accel;
    ins_data->prev_body_delta_ang = ins_data->body_delta_ang;
    ins_data->prev_body_delta_vel = ins_data->body_delta_vel;
    ins_data->body_delta_ang = this->gyro_transf.step_unit_const * (this->gyro_transf.scale_rot * (gyro - this->gyro_transf.bias));
    ins_data->body_delta_vel = this->accel_transf.step_unit_const * (this->accel_transf.scale_rot * (accel - this->accel_transf.bias));
    ins_data->temper = 25.0 + this->temper_const * package->temper;
    std::cout << "index ok!\n";

    return 0;
}
