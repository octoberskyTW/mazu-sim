
#include <cmath>
#include <cstring>
#include "cadac_constants.hh"
#include "env/atmosphere76.hh"

cad::Atmosphere76::Atmosphere76()
{
    snprintf(name, sizeof(name), "Atmosphere US 1976, public domain");

    altitude = 0;

    update_values();
    vsound = sqrt(1.4 * RGAS * tempk);

    vwind = 0;
    dwind = 0;
}

cad::Atmosphere76::~Atmosphere76()
{
}

void cad::Atmosphere76::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;

    update_values();
    vsound = sqrt(1.4 * RGAS * tempk);

    vwind = 0;
    dwind = 0;
}

void cad::Atmosphere76::update_values()
{
    double rearth(6369.0);   // radius of the earth - km
    double gmr(34.163195);   // gas constant
    double rhosl(1.22500);   // sea level density - kg/m^3
    double pressl(101325.);  // sea level pressure - Pa
    double tempksl(288.15);  // sea level temperature - dK

    double htab[8] = { 0.0, 11.0, 20.0, 32.0,
                       47.0, 51.0, 71.0, 84.852 };  // altitude
    double ttab[8] = { 288.15, 216.65, 216.65, 228.65,
                       270.65, 270.65, 214.65, 186.946 };  // temperature
    double ptab[8] = { 1.0, 2.233611e-1, 5.403295e-2,
                       8.5666784e-3, 1.0945601e-3, 6.6063531e-4,
                       3.9046834e-5, 3.68501e-6 };  // pressure
    double gtab[8] = { -6.5, 0.0, 1.0, 2.8,
                       0.0, -2.8, -2.0, 0.0 };  // temperature gradient

    double delta(0);

    // convert geometric (m) to geopotential altitude (km)
    double alt = altitude / 1000;
    double h = alt * rearth / (alt + rearth);

    // binary search determines altitude table entry i below actual altitude
    int i(0);  // offset of first value in table
    int j(7);  // offset of last value in table
    for (;;) {
        int k = (i + j) / 2;  // integer division
        if (h < htab[k])
            j = k;
        else
            i = k;
        if (j <= (i + 1))
            break;
    }
    // within stratosphere
    if (alt < 84.852) {
        // normalized temperature 'theta' from table look-up and gradient
        // interpolation
        double tgrad = gtab[i];
        double tbase = ttab[i];
        double deltah = h - htab[i];
        double tlocal = tbase + tgrad * deltah;
        double theta = tlocal / ttab[0];

        // normalized pressure from hydrostatic equations
        if (tgrad == 0)
            delta = ptab[i] * exp(-gmr * deltah / tbase);
        else
            delta = ptab[i] * pow((tbase / tlocal), (gmr / tgrad));

        // normalized density
        double sigma = delta / theta;

        // output
        density = rhosl * sigma;
        pressure = pressl * delta;
        tempk = tempksl * theta;
    } else {
        // beyond stratosphere
        density = 0;
        pressure = 0;
        tempk = 186.946;
    }
}
