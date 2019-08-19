#include "integrate.hh"

///////////////////////////////////////////////////////////////////////////////
// Integration of scalar state variable
// Modified Euler Midpoint method
// Example first order lag:
//    phid_new=(phic-phi)/tphi;
//    phi=integrate(phid_new,phid,phi,int_step);
//    phid=phid_new;
// 010628 Created by Peter H Zipfel
// 050202 Simplified and renamed to Modified Euler method, PZi
///////////////////////////////////////////////////////////////////////////////
double integrate(const double &dydx_new, const double &dydx, const double &y,
                 const double &int_step)
{
    // return y+int_step*(dydx+0.5*int_step*dydx_new);
    return y + (dydx_new + dydx) * int_step / 2.;
    // return y +
    //        (dydx + 4. * ((dydx_new + dydx) / 2.) + dydx_new) * int_step / 6.;
    // return y + (dydx * int_step);
    // return y + ((3.0 / 2.0) * dydx_new * int_step) -
    //        ((1.0 / 2.0) * dydx * int_step);
}

///////////////////////////////////////////////////////////////////////////////
// Integration of Matrix MAT(r,c)
//
// 030424 Created by Peter H Zipfel
// 170120 Modified to use armadillo library matrix type
///////////////////////////////////////////////////////////////////////////////
arma::mat integrate(arma::mat &DYDX_NEW, arma::mat &DYDX, arma::mat &Y,
                    const double int_step)
{
    int nrow = Y.n_rows;
    int nrow1 = DYDX_NEW.n_rows;
    int nrow2 = DYDX.n_rows;
    int ncol = Y.n_cols;
    int ncol1 = DYDX_NEW.n_cols;
    int ncol2 = DYDX.n_cols;

    assert((nrow == nrow1 && nrow == nrow2) &&
           " *** Error: incompatible row-dimensions in 'integrate()' *** ");
    assert((ncol == ncol1 && ncol == ncol2) &&
           " *** Error: incompatible column-dimensions in 'integrate()' *** ");

    arma::mat RESULT(nrow, ncol);
    for (int r = 0; r < nrow; r++)
        for (int c = 0; c < ncol; c++)
            RESULT(r, c) = integrate(DYDX_NEW(r, c), DYDX(r, c), Y(r, c), int_step);

    return RESULT;
}
