#ifndef __STOCHASTIC_UTIL_HH__
#define __STOCHASTIC_UTIL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (RNG)
LIBRARY DEPENDENCY:
      ((../src/stochastic.cpp))
*******************************************************************************/

/**
 * \brief Generating an exponential distribution with a given mean density.
 * Ref:
 *   Tybrin, "CADAC Program documentation", June 2000 and source code CADX3.FOR
 *   Numerical Recipies, p 287, 1992 Cambridge University Press
 * Function unituni() is a CADAC++ utility
 *   parameter input:
 *     density = # of events per unit of variable (in the mean)
 * return output:
 *     value = units of variable to be traversed until next event occurs
 *
 * The variance is density^2
 */
double exponential(double density);

/**
 * \brief Generating a standard distribution with 'mean' and 'sig' std
 *        deviation.
 * Ref Numerical Recipies, p 289, 1992 Cambridge University Press
 * Function unituni() is a CADAC++ utility
 *
 * parameter input:
 *   min = standard deviation of Gaussian distribution - unit of variable
 *   mean = mean value of Gaussian distribution - unit of variable
 * return output:
 *   value = value of variable - unit of variable
 */
double gauss(double mean, double sig);

/**
 * \brief Generating a time-correlated Gaussian variable with zero mean.
 * Ref: CADAC Subroutine CNT_GAUSS
 * Function gauss() is CADAC++ utility
 *
 * parameter input:
 *   sigma = standard deviation of Gaussian distribution - unit of variable
 *   bcor = beta time correlation coefficient - 1/s (Hz)
 *   time = simulation time - s
 *   intstep = integration step size - s
 * return output:
 *   value = value of variable - unit of variable
 */
double markov(double sigma,
              double bcor,
              double time,
              double intstep,
              double &value_saved);

/**
 * \brief Generating a Rayleigh distribution with peak value of pdf = 'mode'.
 * Ref: Tybrin, "CADAC Program documentation", June 2000 and source code
 *      CADX3.FOR
 * Function unituni() is a CADAC++ utility
 *
 * parameter input:
 *   mode= mode (peak value of pdf) of Rayleigh distribution - unit of variable
 * return output:
 *   value=value of variable - unit of variable
 *
 * The mean of the distribution is: mean = mode * (pi/2)
 * The variance is: variance = mode^2 * (2 - pi/2)
 */
double rayleigh(double mode);

/**
 * \brief Generating uniform random distribution between 'min' and 'max'.
 */
double uniform(double min, double max);

/**
 * \brief Generating uniform random distribution between 0-1 based on
 *        C function rand().
 */
double unituni();



#endif  // __STOCHASTIC_UTIL_HH__
