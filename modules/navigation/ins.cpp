#include "ins.h"


#define ARMALIST arma::vec(ARMALIST_NUM)
#define ARMALIST_NUM 30
std::vector<std::string> split(const std::string &str, const std::string &delim);


INS::INS()
{
    //constant
    this->Ra = 9.7803267715;         // m/s^2
    this->a1 = 0.0052790414;         // m/s^2
    this->a2 = 0.0000232718;         // m/s^2
    this->b1 = -0.000003087691089;   // 1/s^2
    this->b2 = 0.000000004397731;    // 1/s^2
    this->b3 = 0.000000000000721;    // 1/ms^2
    this->omega_p = 0.000072921158;  // rad/s

    this->a = 6378137;
    this->b = 6356752.3142;
    this->e_sq = 0.0066943799900396166;  //e^2
    this->m_deno = 6377851.1655542115;   //a*(1-e^2)

    //this->step_time = 0.05;
    this->half_pi = M_PI / 2.0;
}

INS::~INS()
{
}

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
                arma::mat33 const &BtoN_rot)
{
    arma::vec3 zero3(arma::fill::zeros);
    arma::vec4 zero4(arma::fill::zeros);
    arma::mat33 zero33(arma::fill::zeros);

    //sdt
    ins_data->sensor_gyro = zero3;
    ins_data->sensor_accel = zero3;
    //Fusion
    ins_data->body_delta_ang = zero3;
    ins_data->body_delta_vel = zero3;
    ins_data->prev_body_delta_ang = zero3;
    ins_data->prev_body_delta_vel = zero3;
    ins_data->temper = 0;
    //INS
    ins_data->lat = lat;  //step 15 16 17
    ins_data->prev_lat = 0;
    ins_data->lnt = lnt;  //step 15 16 17
    ins_data->prev_lnt = 0;
    ins_data->prev_rkt_height = 0;
    ins_data->rkt_height = rkt_height;  //step14

    ins_data->prev_ned_rkt_vel = zero3;
    ins_data->ned_rkt_vel = ned_rkt_vel;  //step12

    ins_data->prev_delta_ned_cor_vel = zero3;
    ins_data->delta_ned_cor_vel = delta_ned_cor_vel;  //step8

    ins_data->prev_delta_ned_sf_vel = zero3;
    ins_data->delta_ned_sf_vel = delta_ned_sf_vel;  //step11


    ins_data->prev_BtoN_rot = zero33;
    ins_data->BtoN_rot = BtoN_rot;  //step11
    ins_data->prev_EtoN_qtn = zero4;
    ins_data->EtoN_qtn = EtoN_qtn;  //step15 16 17
    ins_data->prev_NtoB_qtn = zero4;
    ins_data->NtoB_qtn = NtoB_qtn;  //step18 19 20

    ins_data->euler_attitude = euler_attitude;  //step 18 19 20
    return 0;
}

int INS::update(struct ins_data_t *ins_data, double step_time)
{
    //step1
    double geoM = 0, geoN = 0;
    double prev_lat = ins_data->prev_lat;
    double prev_lnt = ins_data->prev_lnt;
    geoM_N(&geoM, &geoN, prev_lat);

    //step2
    arma::vec3 geo_ned_rkt_vel(arma::fill::zeros);
    arma::vec3 prev_ned_rkt_vel = ins_data->prev_ned_rkt_vel;
    arma::vec3 prev_delta_ned_cor_vel = ins_data->prev_delta_ned_cor_vel;
    arma::vec3 prev_delta_ned_sf_vel = ins_data->prev_delta_ned_sf_vel;
    geo_ned_rkt_vel = prev_ned_rkt_vel + 0.5 * (prev_delta_ned_cor_vel + prev_delta_ned_cor_vel);

    //step3
    double geo_height = 0.0;
    double prev_rkt_height = ins_data->prev_rkt_height;
    geoHeight(&geo_height, geo_ned_rkt_vel, prev_rkt_height, step_time);

    //step4
    arma::vec3 geo_IE_ang_rate(arma::fill::zeros);
    arma::vec3 geo_EN_ang_rate(arma::fill::zeros);
    geoIEAngRate(geo_IE_ang_rate, prev_lat);
    geoENAngRate(geo_EN_ang_rate, geo_ned_rkt_vel, geoM, geoN, prev_lat, prev_lnt, geo_height);

    //step5
    double geo_lat = 0, geo_lnt = 0;
    arma::vec4 geo_EtoN_qtn(arma::fill::zeros);
    arma::vec4 prev_EtoN_qtn = ins_data->prev_EtoN_qtn;
    arma::vec3 geo_ned_delta_ang(arma::fill::zeros);
    arma::vec3 geo_ecef_delta_ang(arma::fill::zeros);

    EcefDelataAng(geo_ecef_delta_ang, step_time);
    NedDeltaAng(geo_ned_delta_ang, geo_IE_ang_rate, geo_EN_ang_rate, step_time);
    geoLatLnt(&geo_lat,
              &geo_lnt,
              geo_EtoN_qtn,
              prev_EtoN_qtn,
              geo_ned_delta_ang,
              geo_ecef_delta_ang);

    //step6
    arma::vec3 geo_gravity(arma::fill::zeros);
    geoGravity(geo_gravity,
               geo_lat,
               geo_lnt,
               geo_height);

    //step7
    geoIEAngRate(geo_IE_ang_rate, prev_lat);
    geoENAngRate(geo_EN_ang_rate, geo_ned_rkt_vel, geoM, geoN, geo_lat, geo_lnt, geo_height);

    //step8
    arma::vec3 delta_ned_cor_vel(arma::fill::zeros);
    deltaNedCorVel(delta_ned_cor_vel,
                   geo_gravity,
                   prev_ned_rkt_vel,
                   geo_IE_ang_rate,
                   geo_EN_ang_rate,
                   step_time);
    ins_data->delta_ned_cor_vel = delta_ned_cor_vel;

    //step9
    arma::mat33 prevNtoN_rot;
    angleToRotMat(prevNtoN_rot,
                  geo_ned_delta_ang);

    //step10
    arma::vec3 delta_rot_scul_vel;
    arma::vec3 body_delta_ang = ins_data->body_delta_ang;
    arma::vec3 body_delta_vel = ins_data->body_delta_ang;
    arma::vec3 prev_body_delta_ang = ins_data->prev_body_delta_ang;
    arma::vec3 prev_body_delta_vel = ins_data->prev_body_delta_ang;

    deltaRotSculVel(delta_rot_scul_vel,
                    body_delta_ang,
                    body_delta_vel,
                    prev_body_delta_ang,
                    prev_body_delta_vel);

    //step 18 19 20
    arma::vec3 euler_attitude(arma::fill::zeros);
    arma::vec4 NtoB_qtn(arma::fill::zeros);
    arma::vec4 prevBtoB_qtn(arma::fill::eye);
    arma::vec4 prev_NtoB_qtn = ins_data->prev_NtoB_qtn;

    attitude(euler_attitude,
             NtoB_qtn,
             prevBtoB_qtn,
             prev_NtoB_qtn,
             body_delta_ang,
             prev_body_delta_ang);

    ins_data->NtoB_qtn = NtoB_qtn;
    ins_data->euler_attitude = euler_attitude;

    //step 11
    arma::vec3 delta_ned_sf_vel(arma::fill::zeros);
    arma::mat33 BtoN_rot(arma::fill::zeros);
    arma::mat33 prev_BtoN_rot = ins_data->prev_BtoN_rot;

    deltaNedSfVel(delta_ned_sf_vel,
                  BtoN_rot,
                  prev_delta_ned_sf_vel,
                  prevNtoN_rot,
                  prev_BtoN_rot,
                  prevBtoB_qtn,
                  body_delta_vel,
                  delta_rot_scul_vel);
    ins_data->delta_ned_sf_vel = delta_ned_sf_vel;
    ins_data->BtoN_rot = BtoN_rot;


    //step12
    arma::vec3 ned_rkt_vel(arma::fill::zeros);
    ned_rkt_vel = prev_ned_rkt_vel + delta_ned_cor_vel + delta_ned_sf_vel;
    ins_data->ned_rkt_vel = ned_rkt_vel;

    //step13
    geo_ned_rkt_vel = 0.5 * (ned_rkt_vel + prev_ned_rkt_vel);


    //step14
    double rkt_height = 0;
    geoHeight(&geo_height, geo_ned_rkt_vel, prev_rkt_height, step_time);
    rkt_height = prev_rkt_height - step_time * geo_ned_rkt_vel(2);
    ins_data->rkt_height = rkt_height;

    //step15 16 17
    double lat = 0, lnt = 0;
    arma::vec4 EtoN_qtn(arma::fill::zeros);
    EcefDelataAng(geo_ecef_delta_ang, step_time);
    NedDeltaAng(geo_ned_delta_ang, geo_IE_ang_rate, geo_EN_ang_rate, step_time);
    geoLatLnt(&lat,
              &lnt,
              EtoN_qtn,
              prev_EtoN_qtn,
              geo_ned_delta_ang,
              geo_ecef_delta_ang);

    ins_data->lat = lat;
    ins_data->lnt = lnt;
    ins_data->EtoN_qtn = EtoN_qtn;

    return 0;
}

int INS::geoM_N(double *geoM,
                double *geoN,
                double lat)
{
    double s = 0;
    s = std::sin(lat);
    s = std::sqrt(1 - this->e_sq * s * s);
    *geoM = this->m_deno / (s * s * s);
    *geoN = this->a / s;
    return 0;
}

int INS::geoHeight(double *height,
                   arma::vec3 const &vel,
                   double prev_height,
                   double step_time)
{
    *height = prev_height - 0.5 * step_time * vel(2);
    return 0;
}

int INS::geoIEAngRate(arma::vec3 &IE_ang_rate,
                      double lat)
{
    double c = std::cos(lat);
    double s = std::sin(lat);
    IE_ang_rate(0) = c * this->omega_p;
    IE_ang_rate(1) = 0.0;
    IE_ang_rate(2) = -(s * this->omega_p);
    return 0;
}

int INS::geoENAngRate(arma::vec3 &EN_ang_rate,
                      arma::vec3 const &ned_rkt_vel,
                      double M,
                      double N,
                      double lat,
                      double lnt,
                      double height)
{
    UNUSED(lnt);
    double t = std::tan(lat);
    EN_ang_rate(0) = ned_rkt_vel(1) / (N + height);
    EN_ang_rate(1) = -ned_rkt_vel(0) / (M + height);
    EN_ang_rate(2) = -(ned_rkt_vel(1) * t) / (N + height);
    return 0;
}

int INS::NedDeltaAng(arma::vec3 &ned_delta_ang,
                     arma::vec3 const &IE_ang_rate,
                     arma::vec3 const &EN_ang_rate,
                     double step_time)
{
    ned_delta_ang = step_time * (IE_ang_rate + EN_ang_rate);
    return 0;
}

int INS::EcefDelataAng(arma::vec3 &ecef_delta_ang,
                       double step_time)
{
    ecef_delta_ang(0) = 0;
    ecef_delta_ang(1) = 0;
    ecef_delta_ang(2) = this->omega_p * step_time;
    return 0;
}


int INS::geoLatLnt(double *lat,
                   double *lnt,
                   arma::vec4 &EtoN_qtn_out,
                   arma::vec4 const &prev_EtoN_qtn,
                   arma::vec3 const &ned_delta_ang,
                   arma::vec3 const &ecef_delta_ang)
{
    //printf("%s begin\n",__FUNCTION__);
    arma::vec4 prevNtoN_qtn(arma::fill::zeros);
    arma::vec4 EtoprevE_qtn(arma::fill::zeros);
    arma::vec4 EtoN_qtn(arma::fill::zeros);

    angleToQuaternion(EtoprevE_qtn, -ecef_delta_ang);
    angleToQuaternion(prevNtoN_qtn, ned_delta_ang);

    QuaternionMultiply(EtoN_qtn, EtoprevE_qtn, prev_EtoN_qtn);
    QuaternionMultiply(EtoN_qtn, EtoN_qtn, prevNtoN_qtn);


    *lat = -2.0 * std::atan(EtoN_qtn(2) / EtoN_qtn(0)) - this->half_pi;
    *lnt = 2.0 * std::atan(EtoN_qtn(3) / EtoN_qtn(0));
    EtoN_qtn_out = EtoN_qtn;
    //printf("%s end\n",__FUNCTION__);
    return 0;
}

int INS::angleToQuaternion(arma::vec4 &qtn, arma::vec3 const &ang)
{
    double ang0, ang1, ang2;
    double sq, c_sq, s_sq, h_sq;
    ang0 = ang(0);
    ang1 = ang(1);
    ang2 = ang(2);
    sq = std::sqrt(ang0 * ang0 + ang1 * ang1 + ang2 * ang2);
    h_sq = 0.5 * sq;
    c_sq = std::cos(h_sq);
    s_sq = std::sin(h_sq);

    qtn(0) = c_sq;
    if (sq != 0.0) {
        qtn(1) = s_sq * ang0 / sq;
        qtn(2) = s_sq * ang1 / sq;
        qtn(3) = s_sq * ang2 / sq;
    } else {
        qtn(1) = 0.0;
        qtn(2) = 0.0;
        qtn(3) = 0.0;
    }
    return 0;
}

int INS::geoGravity(arma::vec3 &gravity,
                    double lat,
                    double lnt,
                    double height)
{
    //printf("%s begin\n",__FUNCTION__);
    UNUSED(lnt);
    double sq = 0;
    sq = std::sin(lat);
    sq *= sq;
    gravity(0) = 0;
    gravity(1) = 1;
    gravity(2) = 0.5 * (this->Ra * (1 + sq * (this->a1 + sq * this->a2)) + height * (this->b1 + this->b2 * sq + this->b3 * height));
    //printf("%s end\n",__FUNCTION__);
    return 0;
}


int INS::deltaNedCorVel(arma::vec3 &delta_ned_cor_vel_out,
                        arma::vec3 const &gravity,
                        arma::vec3 const &prev_ned_rkt_vel,
                        arma::vec3 const &IE_ang_rate,
                        arma::vec3 const &EN_ang_rate,
                        double step_time)
{
    arma::vec3 delta_ned_cor_vel(arma::fill::zeros);
    delta_ned_cor_vel = 2 * IE_ang_rate + EN_ang_rate;
    delta_ned_cor_vel = arma::cross(delta_ned_cor_vel, prev_ned_rkt_vel);
    delta_ned_cor_vel = step_time * (gravity - delta_ned_cor_vel);
    delta_ned_cor_vel_out = delta_ned_cor_vel;
    return 0;
}

int INS::angleToRotMat(arma::mat33 &rot,
                       arma::vec3 const ang)
{
    rot(0, 0) = 1;
    rot(0, 1) = ang(2);
    rot(0, 2) = -ang(1);
    rot(1, 0) = -ang(2);
    rot(1, 1) = 1;
    rot(1, 2) = ang(0);
    rot(2, 0) = ang(1);
    rot(2, 1) = -ang(0);
    rot(2, 2) = 1;
    return 0;
}

int INS::deltaRotSculVel(arma::vec3 &delta_rot_scul_vel,
                         arma::vec3 const &body_delta_ang,
                         arma::vec3 const &body_delta_vel,
                         arma::vec3 const &prev_body_delta_ang,
                         arma::vec3 const &prev_body_delta_vel)
{
    arma::vec3 p0(arma::fill::zeros);
    arma::vec3 p1(arma::fill::zeros);
    arma::vec3 p2(arma::fill::zeros);

    p0 = arma::cross(body_delta_ang, body_delta_vel);
    p1 = arma::cross(prev_body_delta_ang, body_delta_vel);
    p2 = arma::cross(prev_body_delta_vel, body_delta_ang);

    delta_rot_scul_vel = 0.5 * p0 + (p1 + p2) / 12;
    return 0;
}


int INS::attitude(arma::vec3 &euler_attitude,
                  arma::vec4 &NtoB_qtn_out,
                  arma::vec4 &PrevBtoB_qtn_out,
                  arma::vec4 const &prev_NtoB_qtn,
                  arma::vec3 const &body_delta_ang,
                  arma::vec3 const &prev_body_delta_ang)
{
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    arma::vec3 conning(arma::fill::zeros);
    arma::vec4 prevBtoB_qtn(arma::fill::zeros);
    arma::vec4 NtoB_qtn(arma::fill::zeros);

    conning = arma::cross(prev_body_delta_ang, body_delta_ang);
    conning = body_delta_ang + conning / 12.0;

    //take ned frame as inertial frame
    angleToQuaternion(prevBtoB_qtn, conning);

    QuaternionMultiply(NtoB_qtn, prev_NtoB_qtn, prevBtoB_qtn);
    Quaternion2Euler(NtoB_qtn, roll, pitch, yaw);

    PrevBtoB_qtn_out = prevBtoB_qtn;
    NtoB_qtn_out = NtoB_qtn;
    euler_attitude(0) = roll;
    euler_attitude(1) = pitch;
    euler_attitude(2) = yaw;
    return 0;
}



int INS::deltaNedSfVel(arma::vec3 &delta_ned_sf_vel_out,
                       arma::mat33 &BtoN_rot_out,
                       arma::vec3 const &prev_delta_ned_sf_vel,
                       arma::mat33 const &prevNtoN_rot,
                       arma::mat33 const &prev_BtoN_rot,
                       arma::vec4 const &prevBtoB_qtn,
                       arma::vec3 const &body_delta_vel,
                       arma::vec3 const &delta_rot_scul_vel)
{
    UNUSED(prev_delta_ned_sf_vel);
    arma::vec3 delta_ned_sf_vel(arma::fill::zeros);
    arma::mat33 BtoN_rot(arma::fill::zeros);

    arma::vec4 BtoprevB_qtn(arma::fill::zeros);
    arma::mat33 BtoprevB_rot(arma::fill::zeros);

    BtoprevB_qtn = QuaternionInverse(prevBtoB_qtn);
    BtoprevB_rot = Quaternion2Matrix(BtoprevB_qtn);

    BtoN_rot = prevNtoN_rot * prev_BtoN_rot;
    delta_ned_sf_vel = delta_rot_scul_vel + body_delta_vel;
    delta_ned_sf_vel_out = BtoN_rot * delta_ned_sf_vel;
    BtoN_rot_out = BtoN_rot * BtoprevB_rot;
    return 0;
}

int INS::insDataUpdate(struct ins_data_t *ins_data)
{
    ins_data->prev_lat = ins_data->lat;
    ins_data->prev_lnt = ins_data->lnt;
    ins_data->prev_rkt_height = ins_data->rkt_height;

    ins_data->prev_ned_rkt_vel = ins_data->ned_rkt_vel;
    ins_data->prev_delta_ned_cor_vel = ins_data->delta_ned_cor_vel;
    ins_data->prev_delta_ned_sf_vel = ins_data->delta_ned_sf_vel;

    ins_data->prev_BtoN_rot = ins_data->BtoN_rot;
    ins_data->prev_EtoN_qtn = ins_data->EtoN_qtn;
    ins_data->prev_NtoB_qtn = ins_data->NtoB_qtn;

    return 0;
}

int f_dms2deg(double *dms, arma::vec3 const &launch)
{
    int seg = 1;
    arma::vec3 abs_launch = arma::abs(launch);

    if (launch.min() < 0)
        seg = -1;

    *dms = seg * (abs_launch(1) + abs_launch(2) / 60.0 + abs_launch(3) / 3600.0);
    return 0;
}


int INS::csvLoader(std::vector<std::vector<double>> &table, char const *file_path)
{
    std::ifstream fp;
    fp.open(file_path);
    std::string title;
    getline(fp, title);


    for (std::string line; getline(fp, line, '\n');) {
        std::vector<std::string> str;
        std::vector<double> list;
        str = split(line, ",");
        for (auto &ele : str) {
            list.push_back(std::atof(ele.c_str()));
        }
        table.push_back(list);
    }
    fp.close();
    return 0;
}


int INS::SILdebug()
{
    struct ins_data_t ins_data;
    memset(&ins_data, 0, sizeof(struct ins_data_t));
    std::vector<std::vector<double>> src_table;
    std::vector<arma::vec> ans_table;
    std::ofstream of;

    arma::vec3 zero(arma::fill::zeros);
    printf("Before csvloader!\n");
    csvLoader(src_table, "INS0_Export.csv");
    printf("After csvloader!\n");
    std::cout << src_table.size() << std::endl;
    of.open("test.csv", std::ios::out | std::ios::trunc);

    //  arma::vec init_list =
    std::vector<double> init_list;
    init_list = src_table[0];

    ins_data.prev_body_delta_vel(0) = init_list[1];
    ins_data.prev_body_delta_vel(1) = init_list[2];
    ins_data.prev_body_delta_vel(2) = init_list[3];

    ins_data.prev_ned_rkt_vel(0) = init_list[4];
    ins_data.prev_ned_rkt_vel(1) = init_list[5];
    ins_data.prev_ned_rkt_vel(2) = init_list[6];
    ins_data.prev_rkt_height = init_list[7];

    //8 9 10 delta body angle
    ins_data.prev_NtoB_qtn(0) = init_list[11];
    ins_data.prev_NtoB_qtn(1) = init_list[12];
    ins_data.prev_NtoB_qtn(2) = init_list[13];
    ins_data.prev_NtoB_qtn(3) = init_list[14];
    ins_data.prev_lat = init_list[15];

    //initial N to B
    ins_data.prev_BtoN_rot = Quaternion2Matrix(ins_data.prev_NtoB_qtn);
    //ins_data.prev_BtoN_rot = arma::mat33(arma::fill::zeros);
    //ins_data.prev_BtoN_rot(0,0) = 1.0;
    //ins_data.prev_BtoN_rot(1,1) = 1.0;
    //ins_data.prev_BtoN_rot(2,2) = 1.0;

    ins_data.prev_EtoN_qtn(0) = init_list[16];
    ins_data.prev_EtoN_qtn(1) = init_list[17];
    ins_data.prev_EtoN_qtn(2) = init_list[18];
    ins_data.prev_EtoN_qtn(3) = init_list[19];

    //20 21 22 rkt attitude
    ins_data.prev_delta_ned_cor_vel = zero;
    ins_data.prev_delta_ned_sf_vel = zero;
    ins_data.prev_body_delta_ang = zero;

    printf("Before loop !\n");


    src_table.erase(src_table.begin());
    for (auto ele : src_table) {
        std::vector<double> src_list = ele;
        ins_data.time = src_list[0];
        ins_data.body_delta_ang(0) = src_list[8];
        ins_data.body_delta_ang(1) = src_list[9];
        ins_data.body_delta_ang(2) = src_list[10];

        ins_data.body_delta_vel(0) = src_list[1];
        ins_data.body_delta_vel(1) = src_list[2];
        ins_data.body_delta_vel(2) = src_list[3];

        update(&ins_data, 0.05);

        //ans lat height   0
        of << std::fixed << std::setprecision(16) << src_list[15] << ",";
        of << std::fixed << std::setprecision(16) << src_list[7] << ",";

        //ans euler attitude  2
        of << std::fixed << std::setprecision(16) << src_list[20] << ",";
        of << std::fixed << std::setprecision(16) << src_list[21] << ",";
        of << std::fixed << std::setprecision(16) << src_list[22] << ",";

        //ans NtoB_qtn 5
        of << std::fixed << std::setprecision(16) << src_list[11] << ",";
        of << std::fixed << std::setprecision(16) << src_list[12] << ",";
        of << std::fixed << std::setprecision(16) << src_list[13] << ",";
        of << std::fixed << std::setprecision(16) << src_list[14] << ",";

        //ans vel 9
        of << std::fixed << std::setprecision(16) << src_list[4] << ",";
        of << std::fixed << std::setprecision(16) << src_list[5] << ",";
        of << std::fixed << std::setprecision(16) << src_list[6] << ",";

        //ans NtoE_qtn 12
        of << std::fixed << std::setprecision(16) << src_list[16] << ",";
        of << std::fixed << std::setprecision(16) << src_list[17] << ",";
        of << std::fixed << std::setprecision(16) << src_list[18] << ",";
        of << std::fixed << std::setprecision(16) << src_list[19] << ",";

        //lat height 16
        of << std::fixed << std::setprecision(16) << ins_data.lat << ",";
        of << std::fixed << std::setprecision(16) << ins_data.rkt_height << ",";

        //euler_attitude 18
        of << std::fixed << std::setprecision(16) << ins_data.euler_attitude(0) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.euler_attitude(1) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.euler_attitude(2) << ",";

        //NtoB_qtn  21
        of << std::fixed << std::setprecision(16) << ins_data.NtoB_qtn(0) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.NtoB_qtn(1) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.NtoB_qtn(2) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.NtoB_qtn(3) << ",";

        //vel 25
        of << std::fixed << std::setprecision(16) << ins_data.ned_rkt_vel(0) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.ned_rkt_vel(1) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.ned_rkt_vel(2) << ",";

        //ans NtoE_qtn 28
        of << std::fixed << std::setprecision(16) << ins_data.EtoN_qtn(0) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.EtoN_qtn(1) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.EtoN_qtn(2) << ",";
        of << std::fixed << std::setprecision(16) << ins_data.EtoN_qtn(3) << "\r\n";

        insDataUpdate(&ins_data);
    }

    of.close();
    return 0;
}

std::vector<std::string> split(const std::string &str, const std::string &delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == std::string::npos)
            pos = str.length();
        std::string token = str.substr(prev, pos - prev);
        if (!token.empty())
            tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}
