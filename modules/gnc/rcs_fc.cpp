#include "rcs_fc.hh"

RCS_FC::RCS_FC() { this->default_data(); }

RCS_FC::RCS_FC(const RCS_FC &other)
{
    this->default_data();

    this->thtbdcomx = other.thtbdcomx;
    this->psibdcomx = other.psibdcomx;
    this->phibdcomx = other.phibdcomx;

    this->rcs_type = other.rcs_type;
    this->rcs_mode = other.rcs_mode;

    this->rcs_tau = other.rcs_tau;

    this->e_roll = other.e_roll;
    this->e_pitch = other.e_pitch;
    this->e_yaw = other.e_yaw;
}

RCS_FC &RCS_FC::operator=(const RCS_FC &other)
{
    if (&other == this)
        return *this;

    this->thtbdcomx = other.thtbdcomx;
    this->psibdcomx = other.psibdcomx;
    this->phibdcomx = other.phibdcomx;

    this->rcs_type = other.rcs_type;
    this->rcs_mode = other.rcs_mode;

    this->rcs_tau = other.rcs_tau;

    this->e_roll = other.e_roll;
    this->e_pitch = other.e_pitch;
    this->e_yaw = other.e_yaw;

    return *this;
}

void RCS_FC::default_data() {}

void RCS_FC::initialize() {}

void RCS_FC::enable_rcs() { rcs_type = ON_OFF_RCS; }

void RCS_FC::disable_rcs() { rcs_type = NO_RCS; }

bool RCS_FC::isEnabled() { return rcs_type > 0; }

void RCS_FC::set_mode(int in) { rcs_mode = static_cast<RCS_MODE>(in); }

///////////////////////////////////////////////////////////////////////////////
// RCS_FC thruster module
// Member function of class 'Hyper'
// Calls thrusters dynamic subroutine
//
// rcs_mode = 0 no control
//          = 1 all geodetic Euler angle control
//          = 2 thrust vector direction and roll angle control
//          = 3 incidence and roll angle controml
//          = 4 geodetic pitch angle control
//
//
// 040302 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void RCS_FC::actuate()
{
    arma::vec WBICB = grab_WBICB();
    double alphacomx = 0.0;
    double betacomx = 0.0;
    arma::vec3 UTBC = grab_UTBC();

    double qqcx = WBICB(1) * DEG;
    double ppcx = WBICB(0) * DEG;
    double rrcx = WBICB(2) * DEG;

    double alphacx = 0.0;
    double betacx = 0.0;

    double thtbdcx = grab_thtbdcx();
    double psibdcx = grab_psibdcx();
    double phibdcx = grab_phibdcx();

    if (this->rcs_type == NO_RCS)
        return;

    // on-off moment thrusters (Schmitt trigger)
    // roll angle control (always)
    e_roll = phibdcomx - (rcs_tau * ppcx + phibdcx);

    // on/off output of Schmitt trigger
    switch (rcs_mode) {
    case NO_CONTROL:
        break;

    case ALL_GEODETIC_EULUR_ANGLE_CONTROL:
        e_pitch = thtbdcomx - (rcs_tau * qqcx + thtbdcx);
        e_yaw = psibdcomx - (rcs_tau * rrcx + psibdcx);
        break;

    case THRUST_VECTOR_DIRECTION_AND_ROLL_ANGLE_CONTROL:
        e_pitch = -rcs_tau * qqcx - UTBC[2] * DEG;
        e_yaw = -rcs_tau * rrcx + UTBC[1] * DEG;
        break;

    case INCIDENCE_AND_ROLL_ANGLE_CONTROL:
        e_pitch = alphacomx - (rcs_tau * qqcx + alphacx);
        e_yaw = -betacomx - (rcs_tau * rrcx - betacx);
        break;

    case GEODETIC_YAW_ANGLE_CONTROL:
        // e_yaw=psibdcomx-(rcs_tau*rrcx+psibdcx);
        e_pitch = thtbdcomx - (rcs_tau * qqcx + thtbdcx);
        break;
    }
}

void RCS_FC::set_rcs_tau(double in) { rcs_tau = in; }
void RCS_FC::set_thtbdcomx(double in) { thtbdcomx = in; }
void RCS_FC::set_psibdcomx(double in) { psibdcomx = in; }
void RCS_FC::set_phibdcomx(double in) { phibdcomx = in; }

enum RCS_FC::RCS_MODE RCS_FC::get_rcs_mode() { return rcs_mode; }

double RCS_FC::get_e_roll() { return e_roll; }
double RCS_FC::get_e_pitch() { return e_pitch; }
double RCS_FC::get_e_yaw() { return e_yaw; }
