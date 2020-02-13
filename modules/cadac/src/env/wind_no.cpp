#include "env/wind_no.hh"

#include <cstring>

cad::Wind_No::Wind_No()
    : Wind(1, 0)
{
    snprintf(name, sizeof(name), "No Wind");

    altitude = 0;

    vwind = 0;
    psiwdx = 0;
}

cad::Wind_No::~Wind_No()
{
}

void cad::Wind_No::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;
}
