
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "atmosphere_nasa2002.hh"


cad::Atmosphere_nasa2002::Atmosphere_nasa2002()
{
    snprintf(name, sizeof(name), "Atmosphere US 1976, NASA Marshell");
    altitude = 0;

    assert(update_values() != 1);

    vwind = 0;
    dwind = 0;
}

cad::Atmosphere_nasa2002::~Atmosphere_nasa2002()
{
}

void cad::Atmosphere_nasa2002::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;

    assert(update_values() != 1);

    vwind = 0;
    dwind = 0;
}

int cad::Atmosphere_nasa2002::update_values()
{
    double z = altitude / 1000.;
    double *d = &density;
    double *p = &pressure;
    double *t = &tempk;
    double *s = &vsound;

    /* altitude independent variable (km) */
    double zs[49] = {
        0., 11.019, 20.063, 32.162, 47.35, 51.413, 71.802, 86., 91., 94.,
        97., 100., 103., 106., 108., 110., 112., 115., 120., 125.,
        130., 135., 140., 145., 150., 155., 160., 165., 170., 180.,
        190., 210., 230., 265., 300., 350., 400., 450., 500., 550.,
        600., 650., 700., 750., 800., 850., 900., 950., 1000.
    };

    double tms[49] = { 288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65,
                       186.95, 186.87, 187.74, 190.40, 195.08, 202.23, 212.89,
                       223.29, 240.00, 264.00, 300.00, 360.00, 417.23, 469.27,
                       516.59, 559.63, 598.78, 634.39, 666.80, 696.29, 723.13,
                       747.57, 790.07, 825.31, 878.84, 915.78, 955.20, 976.01,
                       990.06, 995.83, 998.22, 999.24, 999.67, 999.85, 999.93,
                       999.97, 999.99, 999.99, 1000., 1000., 1000., 1000. };

    double wms[49] = {
        28.9644, 28.9644, 28.9644, 28.9644, 28.9644, 28.9644, 28.9644,
        28.9522, 28.8890, 28.7830, 28.6200, 28.3950, 28.1040, 27.7650,
        27.5210, 27.2680, 27.0200, 26.6800, 26.2050, 25.8030, 25.4360,
        25.0870, 24.7490, 24.4220, 24.1030, 23.7920, 23.4880, 23.1920,
        22.9020, 22.3420, 21.8090, 20.8250, 19.9520, 18.6880, 17.7260,
        16.7350, 15.9840, 15.2470, 14.3300, 13.0920, 11.5050, 9.7180,
        7.9980, 6.5790, 5.5430, 4.8490, 4.4040, 4.1220, 3.9400
    };

    double ps[49] = { 1013.25, 226.32, 54.7487, 8.68014, 1.10905,
                      0.66938, 0.039564, 3.7338e-03, 1.5381e-03, 9.0560e-04,
                      5.3571e-04, 3.2011e-04, 1.9742e-04, 1.2454e-04, 9.3188e-05,
                      7.1042e-05, 5.5547e-05, 4.0096e-05, 2.5382e-05, 1.7354e-05,
                      1.2505e-05, 9.3568e-06, 7.2028e-06, 5.6691e-06, 4.5422e-06,
                      3.6930e-06, 3.0395e-06, 2.5278e-06, 2.1210e-06, 1.5271e-06,
                      1.1266e-06, 6.4756e-07, 3.9276e-07, 1.7874e-07, 8.7704e-08,
                      3.4498e-08, 1.4518e-08, 6.4468e-09, 3.0236e-09, 1.5137e-09,
                      8.2130e-10, 4.8865e-10, 3.1908e-10, 2.2599e-10, 1.7036e-10,
                      1.3415e-10, 1.0873e-10, 8.9816e-11, 7.5138e-11 };

    double ro = 6356.766; /* radius of Earth - km */
    double go = 9.80665;  /* nominal gravitational acceleration - m/s^2  */
    double wmo = 28.9644; /* molecular weight - ND */
    double rs = 8314.32;  /* universal gas constant (m^2/s^2) (gram/mole) / (degree Kelvin) */

    double alp0; /*  */
    double alp1; /*  */
    double alp2; /*  */
    double alpa; /*  */
    double alpb; /*  */
    double g;    /*  */
    double ht;   /*  */
    double wm;   /*  */
    double wma;  /*  */
    double wmb;  /*  */
    double xi;   /*  */
    double z0;   /*  */
    double z1;   /*  */
    double z2;   /*  */
    double zl;   /* z-lower (altitude below z in table) */
    double zu;   /* z-upper (altitude above z in table) */

    int i, j; /*  */

    /* check to see if input altitude is in range, return 1 if not */
    if (z < 0. || z > 1000.) {
        *t = 0.;
        *p = 0.;
        *d = 0.;
        *s = 0.;
        return 1;
    }

    /* bisection search for i such that zs[i] <= z < zs[i+1] */
    {
        int upper = 48;
        int test;
        i = 0;
        while (upper - i > 1) {
            test = (i + upper) >> 1;
            if (z > zs[test])
                i = test;
            else
                upper = test;
        }
    }

    if (i < 7) {
        zl = ro * zs[i] / (ro + zs[i]);
        zu = ro * zs[i + 1] / (ro + zs[i + 1]);
        wm = wmo;
        ht = (ro * z) / (ro + z);
        g = (tms[i + 1] - tms[i]) / (zu - zl);

        if (g < 0. || g > 0.) {
            *p = ps[i] * pow((tms[i] / (tms[i] + g * (ht - zl))), ((go * wmo) / (rs * g * 0.001))) *
                 100.;
        } else {
            *p = ps[i] *
                 exp(-(go * wmo * (ht * 1000. - zl * 1000.)) / (rs * tms[i])) *
                 100.;
        }
        *t = tms[i] + g * (ht - zl);

    } else {
        if (i == 7) {
            *t = 186.8673;
        }

        if (i >= 8 && i < 15) {
            *t = 263.1905 - 76.3232 * sqrt(1. - pow((z - 91.) / 19.9429, 2.));
        }

        if (i >= 15 && i < 18) {
            *t = 240. + 12. * (z - 110.);
        }

        if (i >= 18) {
            xi = (z - 120.) * (ro + 120.) / (ro + z);
            *t = 1000. - 640. * exp(-0.01875 * xi);
        }

        j = i;

        if (i == 47)
            j = i - 1;

        z0 = zs[j];
        z1 = zs[j + 1];
        z2 = zs[j + 2];
        wma = wms[j] * (z - z1) * (z - z2) / ((z0 - z1) * (z0 - z2)) +
              wms[j + 1] * (z - z0) * (z - z2) / ((z1 - z0) * (z1 - z2)) +
              wms[j + 2] * (z - z0) * (z - z1) / ((z2 - z0) * (z2 - z1));
        alp0 = log(ps[j]);
        alp1 = log(ps[j + 1]);
        alp2 = log(ps[j + 2]);
        alpa = alp0 * (z - z1) * (z - z2) / ((z0 - z1) * (z0 - z2)) +
               alp1 * (z - z0) * (z - z2) / ((z1 - z0) * (z1 - z2)) +
               alp2 * (z - z0) * (z - z1) / ((z2 - z0) * (z2 - z1));
        alpb = alpa;
        wmb = wma;

        if (i != 7 && i != 47) {
            j = j - 1;
            z0 = zs[j];
            z1 = zs[j + 1];
            z2 = zs[j + 2];
            alp0 = log(ps[j]);
            alp1 = log(ps[j + 1]);
            alp2 = log(ps[j + 2]);
            alpb = alp0 * (z - z1) * (z - z2) / ((z0 - z1) * (z0 - z2)) +
                   alp1 * (z - z0) * (z - z2) / ((z1 - z0) * (z1 - z2)) +
                   alp2 * (z - z0) * (z - z1) / ((z2 - z0) * (z2 - z1));
            wmb = wms[j] * (z - z1) * (z - z2) / ((z0 - z1) * (z0 - z2)) +
                  wms[j + 1] * (z - z0) * (z - z2) / ((z1 - z0) * (z1 - z2)) +
                  wms[j + 2] * (z - z0) * (z - z1) / ((z2 - z0) * (z2 - z1));
        }

        *p = 100. * exp((alpa + alpb) / 2.);
        wm = (wma + wmb) / 2.;
    }

    *d = (wm * *p) / (rs * *t);
    *s = sqrt(1.4 * *p / *d);

    return 0; /* normal return, altitude in range */
}
