#include <navigation.h>



std::vector<std::string> split(const std::string &str, const std::string &delim);

Navigation::Navigation()
{
    navInit();
}

Navigation::~Navigation()
{
}

int Navigation::navInit()
{
    calibInit();
    filterInit();
    insInit();

    return 0;
}

//For Mazu
int Navigation::update(double step_time)
{
    this->calibUpdate(&(this->filter_data), &(this->navi_packet));
    this->filterUpdate(&(this->ins_data), &(this->filter_data));
    this->insUpdate(&(this->gc_data), &(this->ins_data), step_time);
    return 0;
}


//For FC
int Navigation::update(navi_egress_packet_t *egress,
                       navi_ingress_packet_t *ingress,
                       double step_time)
{
    this->calibUpdate(&(this->filter_data), ingress);
    this->filterUpdate(&(this->ins_data), &(this->filter_data));
    this->insUpdate(egress, &(this->ins_data), step_time);
    return 0;
}

//calibration
int Navigation::calibInit()
{
    arma::vec3 zero3(arma::fill::zeros);
    arma::mat33 idn33(arma::fill::zeros);
    idn33(0, 0) = 1.0;
    idn33(1, 1) = 1.0;
    idn33(2, 2) = 1.0;

    this->calibLoadAxisPertubation(AXIS_PERX, AXIS_PERY, AXIS_PERZ);
    this->calibLoadAxisDirection(AXIS_DIRX, AXIS_DIRY, AXIS_DIRZ);
    this->calibLoadGyroTransf(zero3, idn33);
    this->calibLoadAccelTransf(zero3, idn33);

    return 0;
}


int Navigation::calibLoadGyroTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot)
{
    this->gyro_transf.bias(0) = axis_dir.x * bias(axis_per.x);
    this->gyro_transf.bias(1) = axis_dir.y * bias(axis_per.y);
    this->gyro_transf.bias(2) = axis_dir.z * bias(axis_per.z);
    this->gyro_transf.scale_rot = scale_rot;
    return 0;
}

int Navigation::calibLoadAccelTransf(arma::vec3 const &bias, arma::mat33 const &scale_rot)
{
    this->accel_transf.bias(0) = axis_dir.x * bias(axis_per.x);
    this->accel_transf.bias(1) = axis_dir.y * bias(axis_per.y);
    this->accel_transf.bias(2) = axis_dir.z * bias(axis_per.z);
    this->accel_transf.scale_rot = scale_rot;
    return 0;
}

int Navigation::calibLoadGyroTransf(struct transf_t *transf)
{
    this->gyro_transf.bias = transf->bias;
    this->gyro_transf.scale_rot = transf->scale_rot;
    return 0;
}

int Navigation::calibLoadAccelTransf(struct transf_t *transf)
{
    this->accel_transf.bias = transf->bias;
    this->accel_transf.scale_rot = transf->scale_rot;
    return 0;
}

int Navigation::calibLoadAxisPertubation(const int &x, const int &y, const int &z)
{
    this->axis_per.x = x;
    this->axis_per.y = y;
    this->axis_per.z = z;
    return 0;
}

int Navigation::calibLoadAxisDirection(const double &x, const double &y, const double &z)
{
    this->axis_dir.x = x;
    this->axis_dir.y = y;
    this->axis_dir.z = z;
    return 0;
}

int Navigation::calibUpdate(calib_to_filter_data_t *filter,
                            navi_ingress_packet_t *ingress)
{
    for (int i = 0; i < 10; ++i) {
        this->calibGyroTransf(filter->gyro[i], arma::vec3(ingress->gyro[i]));
        this->calibAccelTransf(filter->accel[i], arma::vec3(ingress->accel[i]));
    }
    return 0;
}


int Navigation::calibGyroTransf(arma::vec3 &out, arma::vec3 const &in)
{
    arma::vec3 vec(arma::fill::zeros);
    vec(0) = axis_dir.x * in(axis_per.x);
    vec(1) = axis_dir.y * in(axis_per.y);
    vec(2) = axis_dir.z * in(axis_per.z);
    out = this->gyro_transf.scale_rot * (vec - this->gyro_transf.bias);
    return 0;
}

int Navigation::calibAccelTransf(arma::vec3 &out, arma::vec3 const &in)
{
    arma::vec3 vec(arma::fill::zeros);
    vec(0) = axis_dir.x * in(axis_per.x);
    vec(1) = axis_dir.y * in(axis_per.y);
    vec(2) = axis_dir.z * in(axis_per.z);
    out = this->accel_transf.scale_rot * (vec - this->accel_transf.bias);
    return 0;
}

//filter 6
int Navigation::filterInit()
{
    double gyro_scalar_init[NUM_OF_CELL] = { 1.0, 0.0, 0.0, 0.0 };
    double accel_scalar_init[NUM_OF_CELL] = { 1.0, 0.0, 0.0, 0.0 };
    arma::vec3 zero3(arma::fill::zeros);
    filterClear(this->gyro_fb);
    filterClear(this->accel_fb);
    filterLoadScalar(this->gyro_fb, gyro_scalar_init);
    filterLoadScalar(this->accel_fb, accel_scalar_init);
    this->gyro_step_unit_const = 0.005 * DegtoRad;
    this->accel_step_unit_const = 0.005 * MGtoNsec;

    return 0;
}

int Navigation::filterSetScalar(struct filter_buff_t &fb, arma::vec &v)
{
    double scalar[NUM_OF_CELL];

    if (v.size() != NUM_OF_CELL) {
        return -1;
    }

    for (size_t i = 0; i < NUM_OF_CELL; ++i) {
        scalar[i] = v(i);
    }

    filterLoadScalar(fb, scalar);
    return 0;
}

int Navigation::filterSetScalar(arma::vec &gyro, arma::vec &accel)
{
    int ret = 0;
    if ((ret = filterSetScalar(this->gyro_fb, gyro)) < 0)
        return ret;
    if ((ret = filterSetScalar(this->accel_fb, accel)) < 0)
        return ret;
    return ret;
}

void Navigation::filterSetStepUnitConst(double const gyro, double const accel)
{
    this->gyro_step_unit_const = gyro;
    this->accel_step_unit_const = accel;
}

int Navigation::filterUpdate(ins_data_t *out,
                             calib_to_filter_data_t *src_data)
{
    this->filterPush(gyro_out[0], this->gyro_fb, src_data->gyro[0]);
    this->filterPush(gyro_out[1], this->gyro_fb, src_data->gyro[1]);
    this->filterPush(gyro_out[2], this->gyro_fb, src_data->gyro[2]);
    this->filterPush(gyro_out[3], this->gyro_fb, src_data->gyro[3]);
    this->filterPush(gyro_out[4], this->gyro_fb, src_data->gyro[4]);
    this->filterPush(gyro_out[5], this->gyro_fb, src_data->gyro[5]);
    this->filterPush(gyro_out[6], this->gyro_fb, src_data->gyro[6]);
    this->filterPush(gyro_out[7], this->gyro_fb, src_data->gyro[7]);
    this->filterPush(gyro_out[8], this->gyro_fb, src_data->gyro[8]);
    this->filterPush(gyro_out[9], this->gyro_fb, src_data->gyro[9]);
    this->filterPush(accel_out[0], this->accel_fb, src_data->accel[0]);
    this->filterPush(accel_out[1], this->accel_fb, src_data->accel[1]);
    this->filterPush(accel_out[2], this->accel_fb, src_data->accel[2]);
    this->filterPush(accel_out[3], this->accel_fb, src_data->accel[3]);
    this->filterPush(accel_out[4], this->accel_fb, src_data->accel[4]);
    this->filterPush(accel_out[5], this->accel_fb, src_data->accel[5]);
    this->filterPush(accel_out[6], this->accel_fb, src_data->accel[6]);
    this->filterPush(accel_out[7], this->accel_fb, src_data->accel[7]);
    this->filterPush(accel_out[8], this->accel_fb, src_data->accel[8]);
    this->filterPush(accel_out[9], this->accel_fb, src_data->accel[9]);

    out->body_ang_rate_last = gyro_out[9] * DegtoRad;
    out->body_accel_last = accel_out[9] * MGtoNsec;
    out->prev_body_delta_ang = out->body_delta_ang;
    out->prev_body_delta_vel = out->body_delta_vel;

    this->filterIntegral(out->body_delta_ang,
                         this->gyro_out,
                         this->gyro_step_unit_const);
    this->filterIntegral(out->body_delta_vel,
                         this->accel_out,
                         this->accel_step_unit_const);
    return 0;
}

int Navigation::filterClear(struct filter_buff_t &fb)
{
    int i, j;
    arma::vec3 zero3(arma::fill::zeros);
    for (i = 0; i < NUM_OF_CELL; i++) {
        for (j = 0; j < NUM_OF_CELL; j++) {
            fb.buff[i][j] = zero3;
        }
    }
    fb.idx = 0;
    return 0;
}

int Navigation::filterLoadScalar(struct filter_buff_t &fb, double *scalar)
{
    int i;
    for (i = 0; i < NUM_OF_CELL; i++) {
        fb.scalar[i] = scalar[i];
    }
    return 0;
}

int Navigation::filterPush(arma::vec3 &out,
                           struct filter_buff_t &fb,
                           arma::vec3 const &in)
{
    int i, j;
    arma::vec3 zero3(arma::fill::zeros);
    uint32_t idx = fb.idx;
    out = zero3;

    for (i = 0; i < NUM_OF_CELL; i++) {
        fb.buff[idx][i] = in * fb.scalar[i];
    }

    for (i = 0; i < NUM_OF_CELL; i++) {
        j = GET_RINGCELL_IDX(idx + i);
        out += fb.buff[j][i];
    }

    fb.idx = GET_RINGCELL_IDX(idx + 1);
    return 0;
}

int Navigation::filterIntegral(arma::vec3 &out,
                               arma::vec3 *in,
                               double const &unit_const)
{
    arma::vec3 vec(arma::fill::zeros);
    vec += in[0];
    vec += in[1];
    vec += in[2];
    vec += in[3];
    vec += in[4];
    vec += in[5];
    vec += in[6];
    vec += in[7];
    vec += in[8];
    vec += in[9];
    out = unit_const * vec;
    return 0;
}

//INS.h


int Navigation::insInit()
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

    this->half_pi = M_PI / 2.0;

    arma::vec3 zero3(arma::fill::zeros);
    arma::vec4 idn4(arma::fill::zeros);
    idn4(0) = 1.0;

    insDataInit(&(this->ins_data), 0, 0, zero3, zero3, zero3, idn4, idn4);
    return 0;
}


int Navigation::insDataInit(ins_data_t *out,
                            double lat,
                            double lnt,
                            arma::vec3 const &euler_attitude,
                            arma::vec3 const &local_rkt_vel,
                            arma::vec3 const &local_rkt_pos,
                            arma::vec4 const &LtoI_qtn,
                            arma::vec4 const &BtoL_qtn)
{
    arma::vec3 zero3(arma::fill::zeros);
    arma::vec4 zero4(arma::fill::zeros);
    arma::mat33 zero33(arma::fill::zeros);


    //sensorFilter
    out->body_ang_rate_last = zero3;
    out->body_accel_last = zero3;
    out->body_delta_ang = zero3;
    out->body_delta_vel = zero3;
    out->prev_body_delta_ang = zero3;
    out->prev_body_delta_vel = zero3;
    out->temper = 0.0;

    //INS
    out->lat = lat;
    out->prev_lat = 0.0;
    out->lnt = lnt;
    out->prev_lnt = 0.0;
    out->prev_rkt_height = 0.0;
    out->rkt_height = local_rkt_pos(0);
    out->time = 0.0;

    out->r1 = local_rkt_pos;
    out->r = zero3;
    out->v1 = local_rkt_vel;
    out->v = zero3;

    out->dv_L1 = zero3;
    out->dv_L = zero3;

    out->q_bL1 = BtoL_qtn;
    out->q_bL = zero4;

    out->q_Le1 = LtoI_qtn;
    out->q_Le = zero4;

    quat2rotM(out->C_bL1, BtoL_qtn);
    quat2rotM(out->C_Le1, LtoI_qtn);

    //fix term
    out->C_Lb1 = out->C_bL1.t();
    out->C_Lb = zero33;
    out->rkt_accel_last = zero3;
    out->rkt_ang_rate_last = zero3;
    return 0;
}

int Navigation::insUpdate(navi_egress_packet_t *egress,
                          ins_data_t *ins_data_in,
                          double step_time)
{
    insNextStep(ins_data_in);
    //step1
    arma::vec3 beta(arma::fill::zeros);
    arma::vec3 scul(arma::fill::zeros);

    arma::vec3 body_delta_ang = ins_data_in->body_delta_ang;
    arma::vec3 body_delta_vel = ins_data_in->body_delta_vel;
    arma::vec3 prev_body_delta_ang = ins_data_in->prev_body_delta_ang;
    arma::vec3 prev_body_delta_vel = ins_data_in->prev_body_delta_vel;
    arma::vec3 body_ang_rate_last = ins_data_in->body_ang_rate_last;
    arma::vec3 body_accel_last = ins_data_in->body_accel_last;

    beta = arma::cross(prev_body_delta_ang, body_delta_ang) / 12.0;
    scul = 0.5 * arma::cross(body_delta_ang, body_delta_vel) + (arma::cross(prev_body_delta_ang, body_delta_vel) + arma::cross(prev_body_delta_vel, body_delta_ang)) / 12.0;

    //step2
    arma::vec4 q(arma::fill::zeros);
    arma::vec4 q_bL1(arma::fill::zeros);
    arma::vec3 conning(arma::fill::zeros);
    arma::vec3 rkt_accel_last(arma::fill::zeros);
    arma::vec3 euler_attitude(arma::fill::zeros);
    arma::mat33 C_bL1(arma::fill::zeros);
    arma::mat33 C_Lb1(arma::fill::zeros);
    arma::vec4 q_bL = ins_data_in->q_bL;

    conning = beta + body_delta_ang;
    rvec2quat(q, conning);
    quatprod(q_bL1, q_bL, q);
    norm_quat(q_bL1, q_bL1);

    quat2rotM(C_bL1, q_bL1);
    quat2euler(euler_attitude, q_bL1);
    C_Lb1 = C_bL1.t();

    //step3
    arma::vec3 local_gravity(arma::fill::zeros);
    arma::vec3 dv_g_cor(arma::fill::zeros);
    arma::vec3 dv_f_n(arma::fill::zeros);
    arma::vec3 dv_L1(arma::fill::zeros);
    arma::vec3 r1(arma::fill::zeros);
    arma::vec3 v1(arma::fill::zeros);

    arma::mat33 C_bL = ins_data_in->C_bL;
    arma::vec3 r = ins_data_in->r;
    arma::vec3 v = ins_data_in->v;
    // double lat = ins_data_in->lat;
    // double lnt = ins_data_in->lnt;

    //fix gravity term
    local_gravity(0) = -9.816778854403626;
    rkt_accel_last = body_accel_last + C_Lb1 * local_gravity;

    dv_g_cor = local_gravity * step_time;
    dv_f_n = C_bL * (body_delta_vel + scul);
    dv_L1 = dv_f_n + dv_g_cor;
    v1 = v + dv_L1;
    r1 = r + (1.5 * v1 - 0.5 * v) * step_time;  // Adams-Bashforth method


    //Ins data update
    ins_data_in->beta = beta;
    ins_data_in->scul = scul;
    ins_data_in->r1 = r1;
    ins_data_in->v1 = v1;
    ins_data_in->dv_L1 = dv_L1;
    ins_data_in->q_bL1 = q_bL1;
    ins_data_in->C_bL1 = C_bL1;
    ins_data_in->rkt_attitude = euler_attitude;
    ins_data_in->rkt_accel_last = rkt_accel_last;
    ins_data_in->rkt_ang_rate_last = body_ang_rate_last;

    //To guidance packet
    egress->rkt_attitude = euler_attitude;
    egress->rkt_angular_rate = body_ang_rate_last;
    egress->rkt_acc = rkt_accel_last;
    egress->rkt_vel = v1;
    egress->rkt_pos = r1;
    return 0;
}

int Navigation::norm_quat(arma::vec4 &q_out, arma::vec4 const &q_in)
{
    double r = 0.0;
    r = sqrt(q_in(0) * q_in(0) + q_in(1) * q_in(1) + q_in(2) * q_in(2) + q_in(3) * q_in(3));
    q_out = q_in / r;
    return 0;
}

int Navigation::quatprod(arma::vec4 &q_out, arma::vec4 const &q_in1, arma::vec4 const &q_in2)
{
    q_out(0) = q_in1(0) * q_in2(0) - q_in1(1) * q_in2(1) - q_in1(2) * q_in2(2) - q_in1(3) * q_in2(3);
    q_out(1) = q_in1(0) * q_in2(1) + q_in1(1) * q_in2(0) + q_in1(2) * q_in2(3) - q_in1(3) * q_in2(2);
    q_out(2) = q_in1(0) * q_in2(2) - q_in1(1) * q_in2(3) + q_in1(2) * q_in2(0) + q_in1(3) * q_in2(1);
    q_out(3) = q_in1(0) * q_in2(3) + q_in1(1) * q_in2(2) - q_in1(2) * q_in2(1) + q_in1(3) * q_in2(0);
    return 0;
}


int Navigation::rvec2quat(arma::vec4 &qtn, arma::vec3 const &ang)
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

int Navigation::localGravity(arma::vec3 &gravity,
                             double lat,
                             double lnt,
                             double height)
{
    UNUSED(lnt);
    double sq = 0;
    sq = std::sin(lat);
    sq *= sq;
    gravity(0) = 0.5 * (this->Ra * (1 + sq * (this->a1 + sq * this->a2)) + height * (this->b1 + this->b2 * sq + this->b3 * height));
    gravity(1) = 0;
    gravity(2) = 0;

    return 0;
}

int Navigation::insNextStep(ins_data_t *ins_data_in)
{
    ins_data_in->prev_lat = ins_data_in->lat;
    ins_data_in->prev_lnt = ins_data_in->lnt;
    ins_data_in->prev_rkt_height = ins_data_in->rkt_height;

    ins_data_in->r = ins_data_in->r1;
    ins_data_in->v = ins_data_in->v1;
    ins_data_in->q_bL = ins_data_in->q_bL1;
    ins_data_in->q_Le = ins_data_in->q_Le1;
    ins_data_in->C_bL = ins_data_in->C_bL1;
    ins_data_in->C_Lb = ins_data_in->C_Lb1;
    return 0;
}



int Navigation::quat2euler(arma::vec3 &euler_ang, arma::vec4 const &q_in)
{
    //ROLL
    double sinr_cosp = 0.0, cosr_cosp = 0.0;
    sinr_cosp = 2.0 * (q_in(0) * q_in(1) + q_in(2) * q_in(3));
    cosr_cosp = 1.0 - 2.0 * (q_in(1) * q_in(1) + q_in(2) * q_in(2));
    euler_ang(0) = std::atan2(sinr_cosp, cosr_cosp);

    //PITCH
    double sinp = 0.0;
    sinp = 2.0 * (q_in(0) * q_in(2) - q_in(3) * q_in(1));
    if (std::abs(sinp) >= 1.0)
        euler_ang(1) = std::copysign(half_pi, sinp);
    else
        euler_ang(1) = std::asin(sinp);

    //YAW
    double siny_cosp = 0.0, cosy_cosp = 0.0;
    siny_cosp = 2.0 * (q_in(0) * q_in(3) + q_in(1) * q_in(2));
    cosy_cosp = 1.0 - 2.0 * (q_in(2) * q_in(2) + q_in(3) * q_in(3));
    euler_ang(2) = std::atan2(siny_cosp, cosy_cosp);

    return 0;
}

int Navigation::quat2rotM(arma::mat33 &rot_out, arma::vec4 const &qn)
{
    rot_out(0, 0) = qn(0) * qn(0) + qn(1) * qn(1) - qn(2) * qn(2) - qn(3) * qn(3);
    rot_out(1, 0) = 2. * (qn(1) * qn(2) + qn(0) * qn(3));
    rot_out(2, 0) = 2. * (qn(1) * qn(3) - qn(0) * qn(2));
    rot_out(0, 1) = 2. * (qn(1) * qn(2) - qn(0) * qn(3));
    rot_out(1, 1) = qn(0) * qn(0) - qn(1) * qn(1) + qn(2) * qn(2) - qn(3) * qn(3);
    rot_out(2, 1) = 2. * (qn(2) * qn(3) + qn(0) * qn(1));
    rot_out(0, 2) = 2. * (qn(1) * qn(3) + qn(0) * qn(2));
    rot_out(1, 2) = 2. * (qn(2) * qn(3) - qn(0) * qn(1));
    rot_out(2, 2) = qn(0) * qn(0) - qn(1) * qn(1) - qn(2) * qn(2) + qn(3) * qn(3);

    return 0;
}

int Navigation::f_dms2deg(double *dms, arma::vec3 const &launch)
{
    int seg = 1;
    arma::vec3 abs_launch = arma::abs(launch);

    if (launch.min() < 0)
        seg = -1;

    *dms = seg * (abs_launch(1) + abs_launch(2) / 60.0 + abs_launch(3) / 3600.0);
    return 0;
}

int Navigation::build_psivg_thtvg_TM(arma::mat33 &AMAT, const double &yaw, const double &pitch)
{
    AMAT(0, 2) = -std::sin(pitch);
    AMAT(1, 0) = -sin(yaw);
    AMAT(1, 1) = cos(yaw);
    AMAT(2, 2) = cos(pitch);
    AMAT(0, 0) = (AMAT(2, 2) * AMAT(1, 1));
    AMAT(0, 1) = (-AMAT(2, 2) * AMAT(1, 0));
    AMAT(2, 0) = (-AMAT(0, 2) * AMAT(1, 1));
    AMAT(2, 1) = (AMAT(0, 2) * AMAT(1, 0));
    AMAT(1, 2) = 0.0;
    return 0;
}

int Navigation::euler2quat(arma::vec4 &q_out,
                           double Roll,
                           double Pitch,
                           double Yaw)
{
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);

    q_out(0) = cy * cr * cp + sy * sr * sp;
    q_out(1) = cy * sr * cp - sy * cr * sp;
    q_out(2) = cy * cr * sp + sy * sr * cp;
    q_out(3) = sy * cr * cp - cy * sr * sp;

    return 0;
}


//altitude initialization
int Navigation::gravityAlignment(arma::vec4 &qtn_attitude,
                                 arma::vec3 &euler_attitude,
                                 arma::vec3 const &avgGravity)
{
    double r = std::sqrt(avgGravity(0) * avgGravity(0) + avgGravity(2) * avgGravity(2));
    double yaw = std::atan2(avgGravity(1), r);  //atan(x,y) = x/y
    double pitch = std::atan2(-avgGravity(2), avgGravity(0));
    euler2quat(qtn_attitude, 0, pitch, yaw);
    quat2euler(euler_attitude, qtn_attitude);
    return 0;
}

//debug
int Navigation::debugCsvInit(std::ifstream &src_fp,
                             std::ofstream &tgt_fp,
                             const char *src_path,
                             const char *tgt_path)
{
    src_fp.open(src_path, std::ifstream::in);
    tgt_fp.open(tgt_path, std::ofstream::out | std::ofstream::trunc);
    if (src_fp.is_open())
        printf("src_fp ok!\n");
    if (tgt_fp.is_open())
        printf("tgt_fp ok!\n");
    std::string title;
    getline(src_fp, title);
    tgt_fp << title;

    return 0;
}


int Navigation::debugCsvReadLine(std::ifstream &fp, std::vector<double> &data)
{
    uint32_t ret = 0;
    int size = 0;
    std::string line;
    std::getline(fp, line);
    std::cout << "0: " << line << "\n";
    ret = line.size();
    if (ret != 0) {
        std::vector<std::string> str;
        std::vector<double> list;
        str = split(line, ",");
        data.clear();
        for (auto &ele : str) {
            data.push_back(std::atof(ele.c_str()));
            size++;
        }
    }
    return size;
}

int Navigation::debugCsvWriteLine(std::ofstream &fp, std::vector<double> &data)
{
    for (auto &ele : data) {
        fp << std::fixed << std::setprecision(16) << ele << ",";
    }
    fp << std::fixed << "\r\n";

    return 0;
}

int Navigation::SILdebug()
{
    ins_data_t ins_data_in;
    navi_egress_packet_t egress;
    arma::vec3 zero3(arma::fill::zeros);
    arma::vec4 idn4(arma::fill::zeros);
    arma::mat33 idn33(arma::fill::zeros);

    idn4(0) = 1.0;
    idn33(0, 0) = 1.0;
    idn33(1, 1) = 1.0;
    idn33(2, 2) = 1.0;

    std::ifstream src_fp;
    std::ofstream tgt_fp;
    std::vector<double> src_list;
    std::vector<double> tgt_list;
    const char src_path[] = "./tests/INS0_Export.csv";
    const char tgt_path[] = "./tests/INS0_Output.csv";
    int ret = 0;

    insDataInit(&ins_data_in,
                0,
                0,
                zero3,
                zero3,
                zero3,
                idn4,
                idn4);


    this->debugCsvInit(src_fp, tgt_fp, src_path, tgt_path);
    do {
        src_list.clear();
        tgt_list.clear();
        ret = this->debugCsvReadLine(src_fp, src_list);
        if (ret == 0)
            break;
        ins_data_in.time = src_list[0];
        ins_data_in.body_delta_ang(0) = src_list[16];
        ins_data_in.body_delta_ang(1) = src_list[17];
        ins_data_in.body_delta_ang(2) = src_list[18];

        ins_data_in.body_delta_vel(0) = src_list[13];
        ins_data_in.body_delta_vel(1) = src_list[14];
        ins_data_in.body_delta_vel(2) = src_list[15];

        //this->update(&egress, &ins_data_in, 0.05);

        tgt_list.push_back(ins_data_in.rkt_attitude[0]);
        tgt_list.push_back(ins_data_in.rkt_attitude[1]);
        tgt_list.push_back(ins_data_in.rkt_attitude[2]);

        tgt_list.push_back(ins_data_in.r1[0]);
        tgt_list.push_back(ins_data_in.r1[1]);
        tgt_list.push_back(ins_data_in.r1[2]);

        tgt_list.push_back(ins_data_in.v1[0]);
        tgt_list.push_back(ins_data_in.v1[1]);
        tgt_list.push_back(ins_data_in.v1[2]);

        this->debugCsvWriteLine(tgt_fp, tgt_list);
    } while (ret != 0);

    src_fp.close();
    tgt_fp.close();
    return 0;
}


int Navigation::debugCalib(void)
{
    this->navInit();
    std::ifstream src_fp;
    std::ofstream tgt_fp;
    char src_path[] = "";
    char tgt_path[] = "";
    debugCsvInit(src_fp, tgt_fp, src_path, tgt_path);

    return 0;
}

int Navigation::debugFilter(void)
{
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
