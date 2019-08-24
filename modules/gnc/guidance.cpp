#include "guidance.hh"

Guidance::Guidance()
    : VECTOR_INIT(UTBC, 3),
      VECTOR_INIT(UTIC, 3),
      VECTOR_INIT(RBIAS, 3),
      VECTOR_INIT(RGRAV, 3),
      VECTOR_INIT(RGO, 3),
      VECTOR_INIT(VGO, 3),
      VECTOR_INIT(SDII, 3),
      VECTOR_INIT(UD, 3),
      VECTOR_INIT(UY, 3),
      VECTOR_INIT(UZ, 3),
      VECTOR_INIT(ULAM, 3),
      VECTOR_INIT(LAMD, 3)
{
    this->default_data();
}

void Guidance::set_guidance_var(double in1, int in2, double in3, double in4,
                                double in5, double in6, double in7, double in8,
                                double in9, double in10, double in11,
                                double in12, double in13, double in14)
{
    ltg_step = in1;
    num_stages = in2;
    dbi_desired = in3;
    dvbi_desired = in4;
    thtvdx_desired = in5;
    delay_ignition = in6;
    amin = in7;
    lamd_limit = in8;
    exhaust_vel1 = in9;
    exhaust_vel2 = in10;
    burnout_epoch1 = in11;
    burnout_epoch2 = in12;
    char_time1 = in13;
    char_time2 = in14;
}

void Guidance::set_ltg_guidance() { mguide = 5; };

void Guidance::default_data()
{
    init_flag = 1;
    inisw_flag = 1;
    skip_flag = 1;
    ipas_flag = 1;
    ipas2_flag = 1;
    print_flag = 1;

    /* This should be zero? */
    ltg_count = 0;
}

void Guidance::initialize() {}

///////////////////////////////////////////////////////////////////////////////
// Guidance module
// Member function of class 'Guidance'
//
//       mguide =  0 no guidance
//              =  5 linear tangent guidance law (LTG) for rocket ascent
//
// 030616 Created by Peter H Zipfel
// 040330 Added LTG guidance, PZi
// 091214 Modified for ROCKET6, PZi
//////////////////////////////////////////////////////////////////////////////

void Guidance::guidance(double int_step)
{
    // local module variable

    arma::mat33 TBIC = grab_TBIC();
    //-------------------------------------------------------------------------
    // zeroing UTBC,if no guidance
    if (mguide == 0)
        UTBC.zeros();

    // LTG guidance, starting LTG clock
    if (mguide == 5) {
        // start LTG clock
        if (init_flag) {
            init_flag = 0;
            time_ltg = 0;
        } else {
            time_ltg += int_step;
        }
        // calling LTG guidance after every ltg_step
        if (time_ltg > ltg_step * ltg_count) {
            // std::cout << "Called : guidance_ltg" << '\n';
            ltg_count++;
            UTIC = guidance_ltg(time_ltg);
            UTBC = TBIC * UTIC;
        }
    }
    //-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
// Linear tangent guidance (LTG) for ascent
// mguide=5
//
// Ref: Jaggers, R.F."Multistage Linear Tangent Ascent Guidance
//     as Baselined for the Space Shuttle Vehicle", NASA MSC,
//     Internal Note MSC-EG-72-39, June 1972
//
// Uses optimal linear tangent guidance law for up to 3 stages
// of constant rocket thrust
//
// Return output:
//        UTIC(3) = unit thrust vector command in inertial coor. - ND
// Parameter input
//        int_step = integration step
//        time_ltg = time elapsed since start of LTG (mguide=1)
// Nomenclature
//        Capitalized variables are 3x1 vectors, unless a capital
//        'N' is appended then the array is used to store information for each
//        of the n stages (max n=3)
//
// 040319 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

arma::vec3 Guidance::guidance_ltg(double time_ltg_in)
{
    // local variables
    // Parameter output 'guidance_ltg_crct()'
    arma::vec3 VMISS;  // velocity miss - m/s
    // Parameter input 'guidance_ltg_crct()'
    arma::vec3 SPII;  // predicted inertial position vector - m
    arma::vec3 VPII;  // predicted inertial velocity vector - m/s
    // Paramter output of 'guidance_ltg_tgo()'
    double tgop(0);      // time-to-go of previous computation cycle - s
    arma::vec3 BURNTN;   // burn time duration left in n-th stage
                         // -> used in '_igrl()' - s
    arma::vec3 L_IGRLN;  // velocity to-be-gained by n-th stage at 'time_ltg_in'
                         // -> used in '_igrl()' - m/s
    arma::vec3 TGON;     // burn durations remaining, summed over n-th and
                         // previous stages, towards reaching end-state - s
    double l_igrl(0);    // L-integral (velocity to be gained - m/s)
    int nstmax(0);       // maximum # of stages needed , used in '_igrl()'
    // Parameter input/output of 'guidance_ltg_tgo()'
    arma::vec3 TAUN;  // characteristic time of stages - s
    // Parameter input of 'guidance_ltg_tgo()'
    arma::vec3 VEXN;  // exhaust velocity of n-th stage - m/s
    // burn out time of stages - s
    arma::vec4 BOTN;  // burn-out time epoch of n-th stage,
                      // in 'time_ltg_in' clock time - s
    // (0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
    double vgom(0);   // desired velocity to be gained (absolute value) - m/s
    double amag1(0);  // absolute value of current vehicle acceleration - m/s^2
    // Parameter output 'guidance_ltg_igrl()'
    double s_igrl(0);    // thrust integral, postion - m
    double j_igrl(0);    // thrust integral, velocit*time - m
    double q_igrl(0);    // thrust integral, position*time - m*s
    double h_igrl(0);    // thrust integral, velocity*time^2 - m*s
    double p_igrl(0);    // thrust integral, position*time^2 - m*s^2
    double j_over_l(0);  // j_over_l= time remaining - s
    double tlam(0);      // time of thrust integraton - s
    double qprime(0);    // conditioned q-integral - m*s

    // input from other modules
    double thtvdx = grab_thtvdx();
    // double fmassr = grab_fmassr();
    arma::vec3 VBIIC = grab_VBIIC();
    arma::vec3 SBIIC = grab_SBIIC();
    double dbi = norm(SBIIC);
    double dvbi = norm(VBIIC);

    arma::mat33 TBIC = grab_TBIC();

    arma::vec3 FSPCB = grab_FSPCB();
    //-------------------------------------------------------------------------
    // preparing necessary variables
    arma::vec3 ABII = trans(TBIC) * FSPCB;
    amag1 = norm(ABII);

    // initializing
    if (inisw_flag) {
        inisw_flag = 0;

        // initializing predicted state to current state
        SPII = SBIIC;
        VPII = VBIIC;

        // calling corrector for initialization
        guidance_ltg_crct(SDII, UD, UY, UZ, VMISS,  // output
                          VGO,                      // input-output
                          dbi_desired, dvbi_desired, thtvdx_desired, SPII, VPII,
                          SBIIC, VBIIC);  // input
    } else {
        // updating velocity to go
        VGO = VGO - ABII * ltg_step;
    }
    // velocity-to-go magnitude
    vgom = norm(VGO);

    // building data vector of the stages
    TAUN = arma::vec3({ char_time1, char_time2, char_time3 });
    VEXN = arma::vec3({ exhaust_vel1, exhaust_vel2, exhaust_vel3 });

    // building array of burn-out time epochs of n-th stage, in 'time_ltg_in' clock
    // time - s
    // burn-out epoch vector starts with dummy o-th stage then stages 1,2,3
    BOTN[0] = 0;
    BOTN[1] = burnout_epoch1;
    BOTN[2] = burnout_epoch2;
    BOTN[3] = burnout_epoch3;

    // calling time-to-go function
    guidance_ltg_tgo(tgop, BURNTN, L_IGRLN, TGON, l_igrl, nstmax,  // output
                     tgo_save, nst_save, TAUN,                     // input/output
                     VEXN, BOTN, delay_ignition, vgom, amag1, amin, time_ltg_in,
                     num_stages);  // input
    // calling thrust integral function
    guidance_ltg_igrl(s_igrl, j_igrl, q_igrl, h_igrl, p_igrl, j_over_l, tlam,
                      qprime,  // output
                      nst_save, nstmax, BURNTN, L_IGRLN, TGON, TAUN, VEXN, l_igrl,
                      time_ltg_in);  // input
    // calling turning rate function
    guidance_ltg_trate(ULAM, LAMD, RGO,  // output
                       ipas2_flag,       // input-output
                       VGO, s_igrl, q_igrl, j_over_l, lamd_limit, vgom,
                       tgo_save, tgop, SDII, SBIIC, VBIIC, RBIAS, UD, UY, UZ,
                       RGRAV);  // throughput to '_rtgo(()'

    // calculating thrust command vector in inertial coordinates
    arma::vec3 TC =
        ULAM + LAMD * (time_ltg_in - tlam);  // same as: TC=ULAM+LAMD*(-j_over_l)

    // calculating output thrust unit vector after skipping 10 'guid_step' delay
    // (settling of transients)
    if (skip_flag) {
        skip_flag++;
        if (skip_flag == 10)
            skip_flag = 0;
    } else {
        UTIC = normalise(TC);
    }
    // calling end-state predictor and corrector
    guidance_ltg_pdct(SPII, VPII, RGRAV, RBIAS  // output
                      ,
                      LAMD, ULAM, l_igrl, s_igrl, j_igrl, q_igrl, h_igrl, p_igrl,
                      j_over_l, qprime  // input
                      ,
                      SBIIC, VBIIC, RGO, tgo_save);
    guidance_ltg_crct(SDII, UD, UY, UZ, VMISS  // output
                      ,
                      VGO  // input-output
                      ,
                      dbi_desired, dvbi_desired, thtvdx_desired, SPII, VPII,
                      SBIIC, VBIIC);  // input

    // motor burning while fuel available
    // if (fmassr > 0) mprop = 4;

    // cut-off logic
    if (tgo_save < ltg_step) {
        beco_flag = 1;
        mguide = 0;
    }
    // boost engine cut-off print-out
    double ddb(0);
    double dvdb(0);
    double thtvddbx(0);
    if (beco_flag && print_flag) {
        print_flag = 0;
        ddb = dbi_desired - dbi;
        dpd = 0.0;
        dbd = 0.0;
        dvdb = dvbi_desired - dvbi;
        thtvddbx = thtvdx_desired - thtvdx;
        std::cout << " *** Boost engine cut-off ***\n";
        std::cout << "     Orbital position dbi = " << dbi
                  << " m \tInertial speed dvbi = " << dvbi
                  << " m/s \tFlight path angle thtvdx = " << thtvdx << " deg\n";
        std::cout << "     Position error   ddb = " << ddb
                  << " m \t\tSpeed error    dvdb = " << dvdb
                  << " m/s\tAngle error     thtvddbx = " << thtvddbx << " deg\n";
    }
    //-------------------------------------------------------------------------
    return UTIC;
}

///////////////////////////////////////////////////////////////////////////////
// Time-to-go to desired end-state
//
// Parameter output:
//    tgop = time-to-go of previous computation cycle - s
//    BURNTN(3) = burn time duration left in n-th stage -> used in '_igrl()' - s
//    L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg_in'
//                 -> used in '_igrl()' - m/s
//    TGON(3) = burn durations remaining, summed over n-th and previous
//              stages, towards reaching end-state - s
//    l_igrl = L-integral (velocity-to-be-gained by remaining stages - m/s)
//             nstmax = maximum # of stages needed , used in '_igrl()'
//
// Parameter input/output:
//    tgo = time-to-go to desired end-state - s
//    nst = n-th stage number -> used in '_igrl()'
//    TAUN(3) = characteristic time of n-th stage - s
//
// Parameter input:
//    VEXN(3) = exhaust velocity of n-th stage - m/s
//    BOTN(4) = burn-out time epoch of n-th stage, in 'time_ltg_in' clock time - s
//          (0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
//    delay_ignition = delay of booster ignition after stating (input) -s
//    vgom = desired velocity to be gained (absolute value) - m/s
//    amag1 = absolute value of current vehicle acceleration - m/s^2
//    amin = minimum accleration for TAUN calcualtions (input) - m/s^2
//    time_ltg_in = time clock started when LTG is engaged -s
//    num_stages = number of stages (initialized in input)
//
// 040322 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_tgo(double &tgop, arma::mat &BURNTN,
                                arma::mat &L_IGRLN, arma::mat &TGON,
                                double &l_igrl, int &nstmax, double &tgo,
                                int &nst, arma::mat &TAUN, arma::mat VEXN,
                                arma::mat BOTN, double delay_ignition_in,
                                double vgom, double amag1, double amin_in,
                                double time_ltg_in, int num_stages_in)
{
    // local variables
    int i(0);

    //-------------------------------------------------------------------------
    // initializing to first stage
    if (ipas_flag)
        nst = 1;
    // save previous time-to-go for use in '_rtgo()' ( initially: tgo=0)
    tgop = tgo;

    // initialize for-loop
    tgo = 0;
    l_igrl = 0;
    nstmax = num_stages_in;

    // advancing stage counter (outside for-loop, stage counter is nst=1,2,3)
    if ((time_ltg_in >= BOTN[nst]))
        nst++;

    // looping through remaining stages (inside for-loop, stage counter is
    // i=0,1,2)
    for (i = nst - 1; i < nstmax; i++) {
        // if current stage, re-calculating characteristic time
        if (i == (nst - 1)) {
            // first stage: no change (BOTN[0]=0)
            TAUN[nst - 1] = TAUN[nst - 1] - (time_ltg_in - BOTN[nst - 1]);
        }

        // if sufficient accel.and ignition of stage has occured then
        // re-calculate
        if ((amag1 >= amin_in) && (time_ltg_in > (BOTN[nst - 1] + delay_ignition_in))) {
            // re-calculating 'tau' using current acceleration
            TAUN[nst - 1] = VEXN[nst - 1] * (1 / amag1);
        }

        // calculating remaining boost times
        if (i == (nst - 1))
            // remaining burn time left in current stage
            BURNTN[i] = BOTN[i + 1] - time_ltg_in;
        else
            // burn time in other stages (assuming 'delay_ignition_in'=0)
            BURNTN[i] = BOTN[i + 1] - BOTN[i];

        // calculating velocity gained of (n=i+1)-th stage ('L_IGRLN[i]')
        double dum1 = TAUN[i];
        double dum2 = BURNTN[i];
        L_IGRLN[i] = -VEXN[i] * log(1 - dum2 / dum1);

        // accumulating velocity to be gained for the remainder of the boost
        // phase
        l_igrl += L_IGRLN[i];
        // if velocity-to-be-gained is less than desired
        if (l_igrl < vgom) {
            // update tgo for remaining burn times left
            tgo += BURNTN[i];

            // load into array: time to reach end-state for each stage (i=0,1,2)
            TGON[i] = tgo;
        } else {
            i++;
            break;
        }
    }
    // recomputing the velocity-to-be-gained to adjust to vgom
    nstmax = i;  // After an ordinary or 'break' termination of the for-loop,
                 // 'i' is increased by 'one',
    // and is now equal to the number of stages needed to meet the end-state
    // conditons.
    // However, for the following arrays, where 'i' is the offset index,it must
    // be decreased
    // by 'one' to designate the last required stage

    // subtracting from velocity-to-be-gained integral the last stage value
    l_igrl = l_igrl - L_IGRLN[i - 1];

    // re-calculating last stage boost time
    double almx = vgom - l_igrl;
    L_IGRLN[i - 1] = almx;
    double dum3 = VEXN[i - 1];
    BURNTN[i - 1] = TAUN[i - 1] * (1 - exp(-almx / dum3));

    // updating tgo
    tgo += BURNTN[i - 1];  // accumulating burn intervals
                           // remaining until end-state
    TGON[i - 1] = tgo;     // loading burn duration left

    // setting integral 'l_igrl' to desired velocity-to-be-gained
    l_igrl = vgom;

    if (ipas_flag) {
        // initializing 'tgop', to be used in '_rtgo()'
        tgop = tgo;
        // canceling initialization flag
        ipas_flag = 0;
    }
    //-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
// LTG thrust integrals
//
// Parameter output:
//    s_igrl = thrust integral, postion - m
//    j_igrl = thrust integral, velocit*time - m
//    q_igrl = thrust integral, position*time - m*s
//    h_igrl = thrust integral, velocity*time^2 - m*s
//    p_igrl = thrust integral, position*time^2 - m*s^2
//    j_over_l= time remaining - s
//    tlam = time of thrust integration - s
//    qprime = conditioned q-integral - m*s
// Parameter input:
//    nst = n-th stage number - ND
//    num_stages = number of stages (input) - ND
//    BURNTN(3) = burn time duration left in n-th stage - s
//    L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg_in' - m/s
//    TGON(3) = burn durations remaining, summed over n-th and previous
//              stages, towards reaching end-state - s
//    TAUN(3) = characteristic time of n-th stage - s
//    VEXN(3) = exhaust velocity of n-th stage - m/s
//    l_igrl = L-integral (velocity to be gained - m/s)
//    time_ltg_in = time clock started when LTG is engaged -s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_igrl(double &s_igrl, double &j_igrl, double &q_igrl,
                                 double &h_igrl, double &p_igrl,
                                 double &j_over_l, double &tlam, double &qprime,
                                 int nst, int nstmax, arma::mat BURNTN,
                                 arma::mat L_IGRLN, arma::mat TGON,
                                 arma::mat TAUN, arma::mat VEXN, double l_igrl,
                                 double time_ltg_in)
{
    // local variables
    double ls_igrl(0);

    // calculating integrals for remaining boost stages
    for (int i = nst - 1; i < nstmax; i++) {
        double tb = BURNTN[i];
        double tga = TGON[i];
        double dummy = TAUN[i];
        double x = tb / dummy;

        double a1(0);
        if (x == 2)
            // factor 1.001 used to prevent division by zero
            a1 = 1 / (1 - 0.5 * x * (1.001));
        else
            a1 = 1 / (1 - 0.5 * x);

        double a2(0);
        if (x == 1) {
            // x=1 occurs when in '_tgo()':
            // BURNTN[i-1]=TAUN[i-1]*(1-exp(-almx/dum3)); the exponent is very
            // large
            //  and therefore BURNTN[i-1]=TAUN[i-1] -> caused by non-convergence
            //  of solution
            std::cerr << " *** LTG Terminator: end-state cannot be reached *** \n";
            // exit(1); //XXX: random exitpoint
        } else {
            a2 = 1 / (1 - x);
        }

        double dummo = TAUN[i];
        double aa =
            VEXN[i] / dummo;  // longitudinal acceleration of booster - m's^2

        double ll_igrl = L_IGRLN[i];
        double a1x = 4 * a1 - a2 - 3;
        double a2xsq = 2 * a2 - 4 * a1 + 2;
        double sa = (aa * tb * tb / 2) * (1 + a1x / 3 + a2xsq / 6);
        double ha;
        // ha = (aa * tb * tb * tb / 3) * (1. + a1x * 0.75 + a2xsq * 0.6);
        // ha - unused value assignment
        double ja = (aa * tb * tb / 2) * (1 + a1x * (2 / 3) + a2xsq / 2);
        double qa = (aa * tb * tb * tb / 6) * (1 + a1x / 2 + a2xsq * 0.3);
        double pa = (aa * tb * tb * tb * tb / 12) * (1 + a1x * 0.6 + a2xsq * 0.4);

        // if not the current stage
        if (i != nst - 1) {
            double t1 = TGON[i - 1];
            ja = ja + t1 * ll_igrl;
            pa = pa + 2 * t1 * qa + t1 * t1 * sa;
            qa = qa + t1 * sa;
        }
        ha = ja * tga - qa;
        sa = sa + ls_igrl * tb;
        qa = qa + j_igrl * tb;
        pa = pa + h_igrl * tb;
        s_igrl = s_igrl + sa;
        q_igrl = q_igrl + qa;
        p_igrl = p_igrl + pa;
        h_igrl = h_igrl + ha;
        ls_igrl = ls_igrl + ll_igrl;
        j_igrl = j_igrl + ja;
    }
    j_over_l = j_igrl / l_igrl;
    tlam = time_ltg_in + j_over_l;
    qprime = q_igrl - s_igrl * j_over_l;
}

///////////////////////////////////////////////////////////////////////////////
// Turning rate and thrust vector calculations
//
// Parameter output:
//    ULAM_in(3) = unit thrust vector in direction of VGO - ND
//    LAMD(3) = derivative of unit thrust vector - 1/s
//    RGO_in(3) = range-to-go - m
// Parameter input/output:
//    ipas2_flag = initialization flag - ND
// Parameter input:
//    VGO(3) = velocity still to be gained - m/s
//    ipas2_flag = initialization flag - ND
//    s_igrl = thrust integral, postion - m
//    q_igrl = thrust integral, position*time - m*s
//    j_over_l = time remaining - s
//    lamd_limit = LAMD components cannot exceed this limit (input) - 1/s
//    vgom = velocity to be gained - m/s
//    time_ltg_in = time clock started when LTG is engaged -s
// Parameter throughput to 'guidance_ltg_trate_rtgo(()':
//    tgo = time-to-go - s
//    tgop = previous time-to-go calculation - s
//    SDII(3) = desired range, defined in '_crct()' - m
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RBIAS_in(3) = position bias vector, calculated in '_pdct()' - m
//    UD(3) = unit vector of SPII and SDII - ND
//    UY(3) = unit vector normal to trajectory plane - ND
//    UZ(3) = unit vector in traj plane,
//            normal to desired inertial pos - ND
// Parameter modified in 'guidance_ltg_trate_rtgo(()':
//    RGRAV_in(3) = Postion loss due to gravity (modified in '_rtgo()') - m
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_trate(
    arma::mat &ULAM_in, arma::mat &LAMD_in, arma::mat &RGO_in, int &ipas2_flag_in,
    arma::mat VGO_in, double s_igrl, double q_igrl, double j_over_l,
    double lamd_limit_in, double vgom, double tgo, double tgop,
    arma::mat SDII_in, arma::mat SBIIC, arma::mat VBIIC, arma::mat RBIAS_in,
    arma::mat UD_in, arma::mat UY_in, arma::mat UZ_in, arma::mat &RGRAV_in)
{
    //-------------------------------------------------------------------------
    // return if velocity-to-be-gained is zero
    if (vgom == 0)
        return;

    // unit thrust vector in VGO direction
    ULAM_in = normalise(VGO_in);
    LAMD_in.zeros();

    // initializing RGO_in at first pass
    if (ipas2_flag_in) {
        ipas2_flag_in = 0;
        RGO_in = ULAM_in * s_igrl;
    }
    // calling function to get range-to-go (SBII_GO)
    guidance_ltg_trate_rtgo(RGO_in,    // output
                            RGRAV_in,  // input-output
                            tgo, tgop, SDII_in, SBIIC, VBIIC, RBIAS_in, ULAM_in, UD_in, UY_in,
                            UZ_in, s_igrl);  // input
    double denom = (q_igrl - s_igrl * j_over_l);
    if (denom != 0)
        LAMD_in = (RGO_in - ULAM_in * s_igrl) * (1 / denom);
    else
        LAMD_in.zeros();

    // setting limits on LAMD_in
    lamd = norm(LAMD_in);
    if (lamd >= lamd_limit_in) {
        arma::mat ULMD = normalise(LAMD_in);
        LAMD_in = ULMD * lamd_limit_in;
    }
    // diagnostics:
    lamd = norm(LAMD_in);
    //-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
// Range-to-go calculation, called from 'guidance_ltg_trate()'
// * downrange is left free
// * corrects for gravity effects and adds bias corrections
//
// Parameter output:
//    RGO_in(3) = range-to-go vector - m
// Parameter input/output:
//    RGRAV_in(3) = Postion loss due to gravity - m
// Parameter input:
//    tgo = time-to-go - s
//    tgop = previous time-to-go calculation - s
//    VGO(3) = velocity to be achieved - m/s
//    SDII_in(3) = desired range, defined in '_crct()' - m
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RBIAS_in(3) = position bias vector, calculated in '_pdct()' - m
//    ULAM_in(3) = unit thrust vector - ND
//    UD_in(3) = unit vector of SPII and SDII_in - ND
//    UY_in(3) = unit vector normal to trajectory plane - ND
//    UZ_in(3) = unit vector in traj plane, normal to desired inertial pos - ND
//    s_igrl = thrust integral, postion - m
//
// 040326 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_trate_rtgo(arma::mat &RGO_in, arma::mat &RGRAV_in,
                                       double tgo, double tgop, arma::mat SDII_in,
                                       arma::mat SBIIC, arma::mat VBIIC,
                                       arma::mat RBIAS_in, arma::mat ULAM_in,
                                       arma::mat UD_in, arma::mat UY_in, arma::mat UZ_in,
                                       double s_igrl)
{
    // correcting range-due-to-gravity
    RGRAV_in = RGRAV_in * (tgo / tgop) * (tgo / tgop);
    arma::mat RGO_LOCAL = SDII_in - (SBIIC + VBIIC * tgo + RGRAV_in) - RBIAS_in;

    // calculating range-to-go vector component in (UD_in,UY_in)-plane
    double rgox = dot(RGO_LOCAL, UD_in);
    double rgoy = dot(RGO_LOCAL, UY_in);
    arma::mat RGOXY = UD_in * rgox + UY_in * rgoy;

    // replacing down-range z-component
    double num = dot(RGOXY, ULAM_in);
    double denom = dot(ULAM_in, UZ_in);
    if (denom == 0) {
        std::cerr << " *** Warning: divide by zero in 'guidance_ltg_trate_rtgo'; "
                     "previous 'RGO' used *** \n";
        return;
    }
    double rgoz = (s_igrl - num) / denom;

    // (RGO is in the direction of ULAM_in with magnitude of s_igrl)
    RGO_in = RGOXY + UZ_in * rgoz;
}
///////////////////////////////////////////////////////////////////////////////
// End-state predictor calculations
// * corrects for gravity effects and adds bias corrections
//
// Parameter output:
//    SPII(3) = predicted inertial position vector - m
//    VPII(3) = predicted inertial velocity vector - m/s
//    RGRAV_in(3) = position loss due to gravity - m
//    RBIAS_in(3) = position bias -> used in '_rtgo()' - m
// Parameter input:
//    LAMD(3) = derivative of unit thrust vector - 1/s
//    ULAM_in(3) = unit thrust vector in direction of VGO - ND
//    l_igrl = L-integral (velocity to be gained - m/s)
//    s_igrl = thrust integral, postion - m
//    j_igrl = thrust integral, velocit*time - m
//    q_igrl = thrust integral, position*time - m*s
//    h_igrl = thrust integral, velocity*time^2 - m*s
//    p_igrl = thrust integral, position*time^2 - m*s^2
//    j_over_l= time remaining - s
//    qprime = conditioned q-integral - m*s
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RGO(3) = range-to-go - m
//    tgo = time-to-go - s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_pdct(arma::mat &SPII, arma::mat &VPII,
                                 arma::mat &RGRAV_in, arma::mat &RBIAS_in,
                                 arma::mat LAMD_in, arma::mat ULAM_in, double l_igrl,
                                 double s_igrl, double j_igrl, double q_igrl,
                                 double h_igrl, double p_igrl, double j_over_l,
                                 double qprime, arma::mat SBIIC,
                                 arma::mat VBIIC, arma::mat RGO_in, double tgo)
{
    // local variables
    arma::vec3 SBIIC2;
    arma::vec3 VBIIC2;

    // velocity gained due to thrust
    double lmdsq = dot(LAMD_in, LAMD_in);
    arma::mat VTHRUST =
        ULAM_in *
        (l_igrl - 0.5 * lmdsq * (h_igrl - j_igrl * j_over_l));  // Jackson, p.8

    // displacement gained due to thrust
    arma::mat RTHRUST =
        ULAM_in * (s_igrl - 0.5 * lmdsq * (p_igrl - j_over_l * (q_igrl + qprime))) +
        LAMD_in * qprime;  // Jackson, p.8

    // bias used in '-rtgo()'
    RBIAS_in = RGO_in - RTHRUST;

    // offsetting SBIIC, VBIIC to SBIIC1, VBIIC1 for gravity calculations
    arma::mat SBIIC1 =
        SBIIC - RTHRUST * 0.1 - VTHRUST * (tgo / 30);  // Jackson, p.23
    arma::mat VBIIC1 =
        VBIIC + RTHRUST * (1.2 / tgo) - VTHRUST * 0.1;  // Jackson, p.23

    // calling Kepler utility to project to end state (two options available)
    int flag = cad::kepler(SBIIC2, VBIIC2, SBIIC1, VBIIC1, tgo);
    //    int flag=cad_kepler1(SBIIC2,VBIIC2,SBIIC1,VBIIC1,tgo);
    if (flag) {
        std::cerr << " *** Warning: bad Kepler projection in 'guidance_ltg_pdct()' "
                     "*** \n";
    }
    // gravity corrections
    arma::mat VGRAV = VBIIC2 - VBIIC1;
    RGRAV_in = SBIIC2 - SBIIC1 - VBIIC1 * tgo;

    // predicted state with gravity and thrust corrections
    SPII = SBIIC + VBIIC * tgo + RGRAV_in + RTHRUST;
    VPII = VBIIC + VGRAV + VTHRUST;
}
///////////////////////////////////////////////////////////////////////////////
// End-state corrector calculations
// * first called for intialization in 'guidance_ltg()'
//
// Parameter output:
//    SDII_in(3) = desired inertial position, defined here - m
//    UD(3) = unit vector of SPII and SDII_in - ND
//    UY_in(3) = unit vector normal to trajectory plane - ND
//    UZ_in(3) = unit vector in traj plane, normal to desired inertial pos - ND
//    VMISS(3) = velocity miss - m/s
//
// Parameter input/output:
//    VGO(3) = velocity to be achieved - m/s
//
// Parameter input:
//    dbi_desired = desired orbital end position (input) - m
//    dvbi_desired = desired end velocity (input) - m/s
//    SPII(3) = predicted inertial position vector - m
//    VPII(3) = predicted inertial velocity vector - m/s
//    thtvdx_desired = desired flight path angle - deg
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_crct(arma::mat &SDII_in, arma::mat &UD_in, arma::mat &UY_in,
                                 arma::mat &UZ_in, arma::mat &VMISS,
                                 arma::mat &VGO_in, double dbi_desired_in,
                                 double dvbi_desired_in, double thtvdx_desired_in,
                                 arma::mat SPII, arma::mat VPII,
                                 arma::mat SBIIC, arma::mat VBIIC)
{
    // local variables
    arma::vec3 VDII;

    // desired position
    UD_in = normalise(SPII);
    SDII_in = UD_in * dbi_desired_in;

    // generating unit base vectors ('%' overloaded arma::mat operator)
    UY_in = cross(VBIIC, SBIIC) / norm(cross(VBIIC, SBIIC));
    UZ_in = cross(UD_in, UY_in) / norm(cross(UD_in, UY_in));

    // velocity-to-be-gained
    VDII = (UD_in * sin(thtvdx_desired_in * RAD) + UZ_in * cos(thtvdx_desired_in * RAD)) *
           dvbi_desired_in;
    VMISS = VPII - VDII;
    VGO_in = VGO_in - VMISS;

    // diagnostics:
    // displacement of P wrt D (both lie always on the UD_in vector)
    double dpi = norm(SPII);
    double ddi = norm(SDII_in);
    dpd = dpi - ddi;
    // displacement of vehicle B wrt the desired end-point D
    double dbi = norm(SBIIC);
    dbd = dbi - ddi;

    //-------------------------------------------------------------------------
}

arma::vec3 Guidance::get_UTBC() { return UTBC; }
double Guidance::get_alphacomx() { return alphacomx; }
double Guidance::get_betacomx() { return betacomx; }
int Guidance::get_beco_flag() { return beco_flag; }
void Guidance::set_degree(double a, double b)
{
    alphacomx = a;
    betacomx = b;
}
