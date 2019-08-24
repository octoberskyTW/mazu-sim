#include "control.hh"

Control::Control()
    : VECTOR_INIT(GAINFP, 3),
      VECTOR_INIT(GAINFY, 3),
      VECTOR_INIT(IBBB0, 3),
      VECTOR_INIT(IBBB1, 3),
      VECTOR_INIT(IBBB2, 3)
{
}

void Control::initialize()
{
    fmasse = -mdot * 0.05;
    theta_a_cmd = 0.0;
    theta_b_cmd = 0.0;
    theta_c_cmd = 0.0;
    theta_d_cmd = 0.0;
}

void Control::control(double int_step)
{
    calculate_xcg_thrust(int_step);

    switch (maut) {
    case NO_CONTROL:
        return;
        break;

    case ACC_CONTROL_ON:
        // calculate_xcg_thrust(int_step);
        aerodynamics_der();
        delecx = control_normal_accel(ancomx, int_step);
        delrcx = control_yaw_accel(alcomx, int_step);
        theta_a_cmd = delecx;
        theta_b_cmd = delrcx;
        break;
    default:
        break;
    }
}

void Control::load_aerotable(const char *filename)
{
    aerotable = Datadeck(filename);
}

void Control::atmosphere_use_nasa()
{
    atmosphere = new cad::Atmosphere_nasa2002();
}

void Control::atmosphere_use_public() { atmosphere = new cad::Atmosphere76(); }

void Control::calculate_xcg_thrust(double int_step)
{
    fmasse += mdot * int_step;
    mass_ratio = fmasse / fmass0;
    xcg = xcg_0 + (xcg_1 - xcg_0) * mass_ratio;
    thrust = isp * mdot * AGRAV / eng_num;
    IBBB2 = IBBB0 + (IBBB1 - IBBB0) * mass_ratio;
    vmass = vmass0 - fmasse;
}

void Control::aerodynamics_der()
{
    // from other modules
    double dvbec = grab_dvbec();
    double gtvc(1.0);
    double altc = grab_altc();
    double alppcx = grab_alppcx();
    //-------------------------------------------------------------------------
    // MOI components
    double ibbb11 = IBBB2(0);
    double ibbb22 = IBBB2(1);
    double ibbb33 = IBBB2(2);

    atmosphere->set_altitude(altc);
    // Dynamics pressure
    pdynmc = 0.5 * atmosphere->get_density() * dvbec * dvbec;
    // mach number
    vmach = fabs(dvbec / atmosphere->get_speed_of_sound());

    double clmq = aerotable.look_up("clmq_vs_mach", vmach, 0);

    // Non-dimensional derivatives
    // look up coeff at +- 3 deg, but not beyond tables
    double alplx = alppcx + 3.0;
    double alpmx = alppcx - 3.0;
    if (alpmx < 0.)
        alpmx = 0.0;

    // calculating normal force dim derivative wrt alpha 'cla'

    double cn0p = aerotable.look_up("cn0_vs_mach_alpha", vmach, alplx, 0);
    double cn0m = aerotable.look_up("cn0_vs_mach_alpha", vmach, alpmx, 0);

    // replacing value from previous cycle, only if within max alpha limit
    if (alplx < 20.0)
        cla = (cn0p - cn0m) / (alplx - alpmx);

    // calculating pitch moment dim derivative wrt alpha 'cma'

    double clm0p = aerotable.look_up("clm0_vs_mach_alpha", vmach, alplx, 0);
    double clm0m = aerotable.look_up("clm0_vs_mach_alpha", vmach, alpmx, 0);

    // replacing value from previous cycle, only if within max alpha limit
    if (alppcx < 20.0)
        cma = (clm0p - clm0m) / (alplx - alpmx) - cla * (xcp - xcg) / refd;

    // converting output to be compatible with 'aerodynamics_der()'
    // force
    clde = 0.0;
    cyb = -cla;
    cydr = 0.0;
    // roll
    cllda = 0.0;
    cllp = 0.0;
    // pitch
    cmde = 0.0;
    cmq = clmq;
    // yaw
    cnb = -cma;
    cndr = 0.0;
    cnr = clmq;

    // Dimensional derivatives for pitch plane (converted to 1/rad where required)
    double duml = (pdynmc * refa / vmass) / RAD;
    dla = duml * cla;
    dlde = duml * clde;
    dnd = duml * cn0;
    double dumm = pdynmc * refa * refd / ibbb22;
    dma = dumm * cma / RAD;
    dmq = dumm * (refd / (2. * dvbec)) * cmq;
    dmde = dumm * cmde / RAD;

    // Dimensional derivatives in plane (converted to 1/rad where required)
    double dumy = pdynmc * refa / vmass;
    dyb = dumy * cyb / RAD;
    dydr = dumy * cydr / RAD;
    double dumn = pdynmc * refa * refd / ibbb33;
    dnb = dumn * cnb / RAD;
    dnr = dumn * (refd / (2. * dvbec)) * cnr;
    dndr = dumn * cndr / RAD;

    // Dimensional derivatives in roll (converted to 1/rad where required)
    double dumll = pdynmc * refa * refd / ibbb11;
    dllp = dumll * (refd / (2. * dvbec)) * cllp;
    dllda = dumll * cllda / RAD;

    // TVC control derivatives
    // pitch plane
    dlde = gtvc * thrust / vmass;
    dmde = -(reference_point - xcg) * gtvc * thrust / ibbb33;
    // yaw plane
    dydr = dlde;
    dndr = dmde;
    // static margin in pitch (per chord length 'refd')
    if (cla)
        stmarg_pitch = -cma / cla;

    // static margin in yaw (per span length 'refd')
    if (cyb)
        stmarg_yaw = -cnb / cyb;

    // diagnostics: pitch plane roots
    double a11 = dmq;
    double a12(0);
    if (dla)
        a12 = dma / dla;
    double a21 = dla;
    double a22 = -dla / dvbec;

    double arg = pow((a11 + a22), 2.) - 4. * (a11 * a22 - a12 * a21);
    if (arg >= 0.) {
        wnp = 0.;
        zetp = 0.;
        double dum = a11 + a22;
        realp1 = (dum + sqrt(arg)) / 2.;
        realp2 = (dum - sqrt(arg)) / 2.;
        rpreal = (realp1 + realp2) / 2.;
    } else {
        realp1 = 0.;
        realp2 = 0.;
        wnp = sqrt(a11 * a22 - a12 * a21);
        zetp = -(a11 + a22) / (2. * wnp);
        rpreal = -zetp * wnp;
    }
    // diagnostics: yaw plane roots
    a11 = dnr;
    if (dyb)
        a12 = dnb / dyb;
    else
        a12 = 0.;
    a21 = -dyb;
    a22 = dyb / dvbec;
    arg = pow((a11 + a22), 2.) - 4. * (a11 * a22 - a12 * a21);
    if (arg >= 0.) {
        wny = 0.;
        zety = 0.;
        double dum = a11 + a22;
        realy1 = (dum + sqrt(arg)) / 2.;
        realy2 = (dum - sqrt(arg)) / 2.;
        ryreal = (realy1 + realy2) / 2.;
    } else {
        realy1 = 0.;
        realy2 = 0.;
        wny = sqrt(a11 * a22 - a12 * a21);
        zety = -(a11 + a22) / (2. * wny);
        ryreal = -zety * wny;
    }
}

double Control::control_normal_accel(double ancomx_in, double int_step)
{
    double gainfb1(0);
    double gainfb2(0);
    double gainfb3(0);

    // input from other modules
    double dvbec = grab_dvbec();
    arma::vec3 WBICB = grab_computed_WBIB();
    arma::vec3 FSPCB = grab_FSPCB();
    //-------------------------------------------------------------------------
    // calculating online close loop poles
    waclp = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1. + factwaclp);
    paclp = 0.7 + 1e-5 * (pdynmc - 20e3) * (1. + factwaclp);

    // calculating three feedback gains

    gainfb3 = waclp * waclp * paclp / (dla * dmde);
    gainfb2 = (2. * zaclp * waclp + paclp + dmq - dla / dvbec) / dmde;
    gainfb1 = (waclp * waclp + 2. * zaclp * waclp * paclp + dma +
               dmq * dla / dvbec - gainfb2 * dmde * dla / dvbec) /
                  (dla * dmde) -
              gainp;

    // gainfb3=zaclp*zaclp*waclp*waclp*paclp/(dmde*dla);
    // gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
    // gainfb1=(zaclp*zaclp*waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbec-gainfb2*dmde*dla/dvbec)-gainp;

    // pitch loop acceleration control, pitch control command
    double fspb3 = FSPCB(2);
    double zzd_new = AGRAV * ancomx_in + fspb3;
    zz = integrate(zzd_new, zzd, zz, int_step);
    zzd = zzd_new;
    double dqc =
        -gainfb1 * (-fspb3) - gainfb2 * WBICB(1) + gainfb3 * zz + gainp * zzd;

    // diagnostic output
    GAINFP = arma::vec3({ gainfb1, gainfb2, gainfb3 });
    //--------------------------------------------------------------------------
    // loading module-variables

    return dqc;
}

double Control::control_yaw_accel(double alcomx_in, double int_step)
{
    // input from other modules
    double dvbec = grab_dvbec();
    arma::vec3 WBICB = grab_computed_WBIB();
    arma::vec3 FSPCB = grab_FSPCB();

    //-------------------------------------------------------------------------
    // calculating close loop poles
    wacly = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1. + factwacly);
    pacly = 0.7 + 1e-5 * (pdynmc - 20e3) * (1. + factwacly);

    // calculating three feedback gains
    double gainfb3 = -wacly * wacly * pacly / (dyb * dndr);
    double gainfb2 = (2. * zacly * wacly + pacly + dnr + dyb / dvbec) / dndr;
    double gainfb1 = (-wacly * wacly - 2. * zacly * wacly * pacly + dnb +
                      dnr * dyb / dvbec - gainfb2 * dndr * dnb / dvbec) /
                         (dyb * dndr) -
                     gainy;

    // yaw loop acceleration controller, yaw control command
    double fspb2 = FSPCB(1);
    double yyd_new = AGRAV * alcomx_in - fspb2;
    yy = integrate(yyd_new, yyd, yy, int_step);
    yyd = yyd_new;
    double drc =
        -gainfb1 * fspb2 - gainfb2 * WBICB(2) + gainfb3 * yy + gainy * yyd;
    // double drcx = drc * DEG;

    // diagnostic output
    GAINFY = arma::vec3({ gainfb1, gainfb2, gainfb3 });
    //--------------------------------------------------------------------------

    return drc;
}

void Control::set_ancomx(double in) { ancomx = in; }
void Control::set_alcomx(double in) { alcomx = in; }
void Control::set_close_loop_pole(double in1, double in2)
{
    zaclp = in1;
    zacly = in2;
}

void Control::set_factor(double in1, double in2)
{
    factwaclp = in1;
    factwacly = in2;
}

double Control::get_delecx() { return delecx; }
double Control::get_delrcx() { return delrcx; }
double Control::get_theta_a_cmd() { return theta_a_cmd; }
double Control::get_theta_b_cmd() { return theta_b_cmd; }
double Control::get_theta_c_cmd() { return theta_c_cmd; }
double Control::get_theta_d_cmd() { return theta_d_cmd; }

void Control::set_controller_var(double in1, double in2, double in3, double in4,
                                 double in5, double in6, double in7)
{
    vmass0 = in1;
    mdot = in2;
    fmass0 = in3;
    xcg_1 = in4;
    xcg_0 = in5;
    isp = in6;
    fmasse = in7;
}
void Control::set_IBBB0(double in1, double in2, double in3)
{
    IBBB0(0) = in1;
    IBBB0(1) = in2;
    IBBB0(2) = in3;
}
void Control::set_IBBB1(double in1, double in2, double in3)
{
    IBBB1(0) = in1;
    IBBB1(1) = in2;
    IBBB1(2) = in3;
}
void Control::set_NO_CONTROL() { maut = NO_CONTROL; }
void Control::set_acc_control() { maut = ACC_CONTROL_ON; }
void Control::set_aero_coffe(double in1, double in2, double in3)
{
    refd = in1;
    refa = in2;
    xcp = in3;
}
void Control::set_feedforward_gain(double in1) { gainp = in1; }
void Control::set_engnum(double in) { eng_num = in; }
