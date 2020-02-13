#ifndef __INTEGRATE_UTIL_HH__
#define __INTEGRATE_UTIL_HH__

#include <armadillo>
#include <cassert>
#include <vector>

#define INTEGRATE(in, diff)                             \
    do {                                                \
        double in##d_new = diff;                        \
        in = integrate(in##d_new, in##d, in, int_step); \
        in##d = in##d_new;                              \
    } while (0)

#define INTEGRATE_MAT(in, diff)                         \
    do {                                                \
        arma::mat in##D_NEW = diff;                     \
        in = integrate(in##D_NEW, in##D, in, int_step); \
        in##D = in##D_NEW;                              \
    } while (0)

/**
 * \brief Integration of scalar state variable.
 * Modified Euler Midpoint method
 * Example first order lag:
 *   phid_new=(phic-phi)/tphi;
 *   phi=integrate(phid_new,phid,phi,int_step);
 *   phid=phid_new;
 */
double integrate(const double &dydx_new, const double &dydx, const double &y,
                 const double &int_step);

arma::mat integrate(arma::mat &DYDX_NEW, arma::mat &DYDX, arma::mat &Y,
                    const double int_step);

#endif  // __INTEGRATE_UTIL_HH__
