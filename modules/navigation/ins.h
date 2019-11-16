#ifndef __NAVIGATION_INS_H__
#define __NAVIGATION_INS_H__



//LaunchLat0  = [22 15 43.55]
//LaunchLong0 = [120 53 24.55]
//launchAltitude0 = 6
//Initial Azimuth angle(rad) AI0=90; %deg Toward East side

struct ins_data_t {
    //sdt
    arma::vec3 sensor_gyro;
    arma::vec3 sensor_accel;

    //Fusion
    arma::vec3 body_delta_ang;
    arma::vec3 body_delta_vel;
    arma::vec3 prev_body_delta_ang;
    arma::vec3 prev_body_delta_vel;
    double temper;

    //INS
    double lat;  //step 15 16 17
    double prev_lat;
    double lnt;  //step 15 16 17
    double prev_lnt;
    double prev_rkt_height;
    double rkt_height;  //step14
    double time;

    arma::vec3 prev_ned_rkt_vel;
    arma::vec3 ned_rkt_vel;  //step12

    arma::vec3 prev_delta_ned_cor_vel;
    arma::vec3 delta_ned_cor_vel;  //step8

    arma::vec3 prev_delta_ned_sf_vel;
    arma::vec3 delta_ned_sf_vel;  //step11

    arma::mat33 prev_BtoN_rot;
    arma::mat33 BtoN_rot;  //step11

    arma::vec4 prev_EtoN_qtn;
    arma::vec4 EtoN_qtn;  //step15 16 17

    arma::vec4 prev_NtoB_qtn;
    arma::vec4 NtoB_qtn;  //step18 19 20

    arma::vec4 prevBtoB_qtn;
    arma::vec3 euler_attitude;  //step 18 19 20
};

class INS
{
private:
    //constant
    double Ra;       // m/s^2
    double a1;       // m/s^2
    double a2;       // m/s^2
    double b1;       // 1/s^2
    double b2;       // 1/s^2
    double b3;       // 1/ms^2
    double omega_p;  // rad/s

    double a;
    double b;
    double e_sq;    //e^2
    double m_deno;  //a*(1-e^2)

    double half_pi;
    //double step_time;
public:
    INS();
    ~INS();
    int insDataInit(struct ins_data_t *ins_data,
                    double lat,
                    double lnt,
                    double rkt_height,
                    arma::vec3 const &euler_attitude,
                    arma::vec3 const &ned_rkt_vel,
                    arma::vec3 const &delta_ned_cor_vel,
                    arma::vec3 const &delta_ned_sf_vel,
                    arma::vec4 const &EtoN_qtn,
                    arma::vec4 const &NtoB_qtn,
                    arma::mat33 const &BtoN_rot);

    int update(struct ins_data_t *ins_data, double step_time);

    int geoM_N(double *geoM,
               double *geoN,
               double lat);

    int geoHeight(double *height,
                  arma::vec3 const &vel,
                  double prev_height,
                  double step_time);

    int geoIEAngRate(arma::vec3 &IE_ang_rate,
                     double lat);

    int geoENAngRate(arma::vec3 &EN_ang_rate,
                     arma::vec3 const &ned_rkt_vel,
                     double M,
                     double N,
                     double lat,
                     double lnt,
                     double height);

    int NedDeltaAng(arma::vec3 &ned_delta_ang,
                    arma::vec3 const &IE_ang_rate,
                    arma::vec3 const &EN_ang_rate,
                    double step_time);

    int EcefDelataAng(arma::vec3 &ecef_delta_ang,
                      double step_time);


    int angleToQuaternion(arma::vec4 &qtn,
                          arma::vec3 const &ang);

    int geoLatLnt(double *lat,
                  double *lnt,
                  arma::vec4 &EtoN_qtn_out,
                  arma::vec4 const &prev_EtoN_qtn,
                  arma::vec3 const &ned_delta_ang,
                  arma::vec3 const &ecef_delta_ang);

    int geoGravity(arma::vec3 &gravity,
                   double lat,
                   double lnt,
                   double height);

    int deltaNedCorVel(arma::vec3 &delta_ned_cor_vel_out,
                       arma::vec3 const &gravity,
                       arma::vec3 const &prev_ned_rkt_vel,
                       arma::vec3 const &IE_ang_rate,
                       arma::vec3 const &EN_ang_rate,
                       double step_time);

    int deltaNedSfVel(arma::vec3 &delta_ned_sf_vel_out,
                      arma::mat33 &BtoN_rot_out,
                      arma::vec3 const &prev_delta_ned_sf_vel,
                      arma::mat33 const &prevNtoN_rot,
                      arma::mat33 const &prev_BtoN_rot,
                      arma::vec4 const &prevBtoB_qtn,
                      arma::vec3 const &body_delta_vel,
                      arma::vec3 const &delta_rot_scul_vel);

    int attitude(arma::vec3 &euler_attitude,
                 arma::vec4 &NtoB_qtn_out,
                 arma::vec4 &PrevBtoB_qtn_out,
                 arma::vec4 const &prev_NtoB_qtn,
                 arma::vec3 const &body_delta_ang,
                 arma::vec3 const &prev_body_delta_ang);

    int angleToRotMat(arma::mat33 &rot,
                      arma::vec3 const ang);

    int deltaRotSculVel(arma::vec3 &delta_rot_scul_vel,
                        arma::vec3 const &body_delta_ang,
                        arma::vec3 const &body_delta_vel,
                        arma::vec3 const &prev_body_delta_ang,
                        arma::vec3 const &prev_body_delta_vel);

    int dataRecord(std::ofstream &of,
                   struct ins_data_t *ins_data,
                   struct timeval *prev,
                   struct timeval *now,
                   int32_t step);

    int insDataUpdate(struct ins_data_t *ins_data);

    int SILdebug();

    int csvLoader(std::vector<std::vector<double>> &table, char const *file_path);
};

#endif  // __NAVIGATION_INS_H__
