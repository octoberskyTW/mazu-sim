#ifndef __NAVIGATION_SENSOR_CALIBRATION_H__
#define __NAVIGATION_SENSOR_CALIBRATION_H__



#define DegtoRad 7   //0.0174532925
#define MGtoNsec 13  //0.001


struct calibration_src_t {
    uint32_t package_count;
    uint32_t gyroXh;
    uint32_t gyroXl;
    uint32_t gyroYh;
    uint32_t gyroYl;
    uint32_t gyroZh;
    uint32_t gyroZl;
    uint32_t accelXh;
    uint32_t accelXl;
    uint32_t accelYh;
    uint32_t accelYl;
    uint32_t accelZh;
    uint32_t accelZl;

    uint16_t gyroX_high[10];
    uint16_t gyroX_low[10];
    uint16_t gyroY_high[10];
    uint16_t gyroY_low[10];
    uint16_t gyroZ_high[10];
    uint16_t gyroZ_low[10];
    uint16_t accelX_high[10];
    uint16_t accelX_low[10];
    uint16_t accelY_high[10];
    uint16_t accelY_low[10];
    uint16_t accelZ_high[10];
    uint16_t accelZ_low[10];
    uint16_t temper;
};


struct transf_t {
    arma::vec3 bias;
    arma::mat33 scale_rot;
    double high_const;
    double low_const;
    double step_unit_const;
};

class SensorCalibration
{
    //function
public:
    SensorCalibration();
    ~SensorCalibration();
    int update(void *pdata, struct calibration_src_t *package);
    int loadGyroTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot);
    int loadAccelTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot);
    int loadGyroTransf(struct transf_t *transf);
    int loadAccelTransf(struct transf_t *transf);

private:
    //data
    double temper_const;
    double step_time;
    struct transf_t gyro_transf;
    struct transf_t accel_transf;
};

#endif  //  __NAVIGATION_SENSOR_CALIBRATION_H__