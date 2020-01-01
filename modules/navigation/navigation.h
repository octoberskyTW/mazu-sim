#ifndef __NAVIGATION_INS_H__
#define __NAVIGATION_INS_H__

#define _USE_MATH_DEFINES
#include <math.h>

#include <armadillo>
#include <iomanip>

#include "dataflow_binding.hh"
#include "matrix_tool.hh"
#include "misc_utils.h"

//calibration
#define AXIS_PERX 2  // imuZ
#define AXIS_PERY 0  // imuX
#define AXIS_PERZ 1  // imuY

#define AXIS_DIRX 1.0
#define AXIS_DIRY 1.0
#define AXIS_DIRZ 1.0

struct transf_t {
    arma::vec3 bias;
    arma::mat33 scale_rot;
};

struct axis_pertubation {
    double x;
    double y;
    double z;
};

struct axis_direction {
    double x;
    double y;
    double z;
};

//filter
#define NUM_OF_CELL 4  //  power of 2
#define GET_RINGCELL_IDX(idx) ((idx) & (NUM_OF_CELL - 1))
#define DegtoRad 0.017453292519943295
#define MGtoNsec 0.00980675

struct filter_buff_t {
    volatile uint32_t idx;
    arma::vec3 buff[NUM_OF_CELL][NUM_OF_CELL];
    double scalar[NUM_OF_CELL];
};


class Navigation
{
private:
    //outer communication
    navi_ingress_packet_t navi_packet;  //for input binding
    navi_egress_packet_t gc_data;       //for output binding

    //inner communication
    calib_to_filter_data_t filter_data;  //from calib to filteri
    ins_data_t ins_data;                 //from filter to ins

    //calibration
    struct transf_t gyro_transf;
    struct transf_t accel_transf;
    struct axis_pertubation axis_per;
    struct axis_direction axis_dir;

    //filter
    struct filter_buff_t gyro_fb;
    struct filter_buff_t accel_fb;
    double gyro_step_unit_const;
    double accel_step_unit_const;
    arma::vec3 gyro_out[10];
    arma::vec3 accel_out[10];

    //ins
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

public:
    Navigation();
    ~Navigation();

    int update(double step_time);
    int update(navi_egress_packet_t *gc_Data,
               navi_ingress_packet_t *navi_packet,
               double step_time);
    int navInit();

    // setup 3
    int filterSetScalar(struct filter_buff_t &fb, arma::vec &v);
    int filterSetScalar(arma::vec &gyro, arma::vec &accel);
    void filterSetStepUnitConst(double const gyro, double const accel);

    //calibration 10
    int calibInit();
    int calibLoadGyroTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot);
    int calibLoadAccelTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot);
    int calibLoadGyroTransf(struct transf_t *transf);
    int calibLoadAccelTransf(struct transf_t *transf);
    int calibLoadAxisPertubation(const int &x, const int &y, const int &z);
    int calibLoadAxisDirection(const double &x, const double &y, const double &z);
    int calibUpdate(calib_to_filter_data_t *filter_data,
                    navi_ingress_packet_t *navi_packet);
    int calibGyroTransf(arma::vec3 &out, arma::vec3 const &in);
    int calibAccelTransf(arma::vec3 &out, arma::vec3 const &in);

    //filter 6
    int filterInit();
    int filterUpdate(ins_data_t *ins_data,
                     calib_to_filter_data_t *src_data);
    int filterClear(struct filter_buff_t &fb);
    int filterLoadScalar(struct filter_buff_t &fb, double *scalar);
    int filterPush(arma::vec3 &out,
                   struct filter_buff_t &fb,
                   arma::vec3 const &in);
    int filterIntegral(arma::vec3 &out,
                       arma::vec3 *in,
                       double const &unit_const);
    //ins 4
    int insInit();

    int insDataInit(ins_data_t *ins_data,
                    double lat,
                    double lnt,
                    arma::vec3 const &euler_attitude,
                    arma::vec3 const &local_rkt_vel,
                    arma::vec3 const &local_rkt_pos,
                    arma::vec4 const &LtoI_qtn,
                    arma::vec4 const &BtoL_qtn);

    int insUpdate(navi_egress_packet_t *gc_data,
                  ins_data_t *ins_data,
                  double step_time);

    int insNextStep(ins_data_t *ins_data);

    //math 7
    int norm_quat(arma::vec4 &q_out,
                  arma::vec4 const &q_in);
    int quatprod(arma::vec4 &q_out,
                 arma::vec4 const &q_in1,
                 arma::vec4 const &q_in2);
    int rvec2quat(arma::vec4 &qtn,
                  arma::vec3 const &ang);
    int localGravity(arma::vec3 &gravity,
                     double lat,
                     double lnt,
                     double height);
    int quat2euler(arma::vec3 &euler_ang,
                   arma::vec4 const &q_in);
    int quat2rotM(arma::mat33 &rot_out,
                  arma::vec4 const &qn);
    int f_dms2deg(double *dms, arma::vec3 const &launch);

    int build_psivg_thtvg_TM(arma::mat33 &AMAT,
                             const double &yaw,
                             const double &pitch);

    int euler2quat(arma::vec4 &q_out,
                   double Roll,
                   double Pitch,
                   double Yaw);
    //Alignment
    int gravityAlignment(arma::vec4 &qtn_attitude,
                         arma::vec3 &euler_attitude,
                         arma::vec3 const &avgGravity);

    //debug
    int debugCsvInit(std::ifstream &src_fp,
                     std::ofstream &tgt_fp,
                     const char *src_path,
                     const char *tgt_path);
    int debugCsvReadLine(std::ifstream &fp,
                         std::vector<double> &data);
    int debugCsvWriteLine(std::ofstream &fp,
                          std::vector<double> &data);
    int debugCalib(void);
    int debugFilter(void);

    int SILdebug();
};

#endif  // __NAVIGATION_INS_H__
