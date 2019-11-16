#include "mathutils/stochastic.hh"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>

///////////////////////////////////////////////////////////////////////////////
// Generating an exponential distribution with a given mean density
// Ref:
// Tybrin, "CADAC Program documentation", June 2000 and source code CADX3.FOR
// Numerical Recipies, p 287, 1992 Cambridge University Press
// Function unituni() is a CADAC++ utility
//
// parameter input:
//    density = # of events per unit of variable (in the mean)
// return output:
//    value = units of variable to be traversed until next event occurs
//
// The variance is density^2
//
// 010919 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double exponential(double density)
{
    double value;

    assert(density &&
           " *** Error: density not given a non-zero value in 'exponential()' "
           "*** ");

    value = -log(unituni());
    return value / density;
}  ///////////////////////////////////////////////////////////////////////////////
// Generating a standard distribution with 'mean' and 'sig' std deviation
// Ref Numerical Recipies, p 289, 1992 Cambridge University Press
// Function unituni() is a CADAC++ utility
//
// parameter input:
//    min = standard deviation of Gaussian distribution - unit of variable
//    mean = mean value of Gaussian distribution - unit of variable
// return output:
//    value = value of variable - unit of variable
//
// 010913 Created by Peter H Zipfel
// 010914 Normalized gauss tested with a 2000 sample: mean=0.0054, sigma=0.9759
///////////////////////////////////////////////////////////////////////////////
double gauss(double mean, double sig)
{
    static int iset = 0;
    static double gset;
    double fac, rsq, v1, v2, value;

    if (iset == 0) {
        do {
            v1 = 2. * unituni() - 1.;
            v2 = 2. * unituni() - 1.;
            rsq = v1 * v1 + v2 * v2;
        } while (rsq >= 1.0 || rsq == 0);

        fac = sqrt(-2. * log(rsq) / rsq);
        gset = v1 * fac;
        iset = 1;
        value = v2 * fac;
    } else {
        iset = 0;
        value = gset;
    }
    return value * sig + mean;
}
///////////////////////////////////////////////////////////////////////////////
// Generating a time-correlated Gaussian variable with zero mean
// Ref: CADAC Subroutine CNT_GAUSS
// Function gauss() is CADAC++ utility
//
// parameter input:
//    sigma = standard deviation of Gaussian distribution - unit of
//            variable
//    bcor = beta time correlation coefficient - 1/s (Hz)
//    time = simulation time - s
//    intstep = integration step size - s
//    value_saved = value of previous integration step
// return output:
//    value = value of variable - unit of variable
//
// 010914 Created by Peter H Zipfel
// 020723 Replaced static variable by '&value_saved', PZi
///////////////////////////////////////////////////////////////////////////////
double markov(double sigma,
              double bcor,
              double time,
              double intstep,
              double &value_saved)
{
    double value = 0;

    value = gauss(0., sigma);
    if (time == 0.) {
        value_saved = value;
    } else {
        if (bcor != 0.) {
            double dum = exp(-bcor * intstep);
            double dumsqrd = dum * dum;
            value = value * sqrt(1. - dumsqrd) + value_saved * dum;
            value_saved = value;
        }
    }
    return value;
}
///////////////////////////////////////////////////////////////////////////////
// Generating a Rayleigh distribution with peak value of pdf = 'mode'
// Ref: Tybrin, "CADAC Program documentation", June 2000 and source code
//      CADX3.FOR
// Function unituni() is a CADAC++ utility
//
// parameter input:
//    mode = mode (peak value of pdf) of Rayleigh distribution - unit of
//           variable
// return output:
//    value=value of variable - unit of variable
//
// The mean of the distribution is: mean = mode * (pi/2)
// The variance is: variance = mode^2 * (2 - pi/2)
//
// 010918 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double rayleigh(double mode)
{
    double value;

    value = sqrt(2. * (-log(unituni())));
    return value * mode;
}

///////////////////////////////////////////////////////////////////////////////
// Generating uniform random distribution between 'min' and 'max'
//
// 010913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double uniform(double min, double max)
{
    double value;
    value = min + (max - min) * unituni();
    return value;
}
///////////////////////////////////////////////////////////////////////////////
// Generating uniform random distribution between 0-1 based on C function rand()
//
// 010913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double unituni()
{
    std::random_device r;
    std::default_random_engine e(r());
    std::uniform_real_distribution<double> uniform_dist(0, 1);
    return uniform_dist(e);
}
