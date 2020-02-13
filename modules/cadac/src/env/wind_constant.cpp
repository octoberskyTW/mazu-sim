#include "env/wind_constant.hh"

#include <cstring>

cad::Wind_Constant::Wind_Constant(double dvba, double dir, double twind_In, double vertical_wind)
    : Wind(twind_In, vertical_wind)
{
    snprintf(name, sizeof(name), "Constant Wind");

    altitude = 0;

    vwind = dvba;
    psiwdx = dir;
}

cad::Wind_Constant::~Wind_Constant()
{
}

void cad::Wind_Constant::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;
}
