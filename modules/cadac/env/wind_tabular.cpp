#include "wind_tabular.hh"

#include <cstring>

cad::Wind_Tabular::Wind_Tabular(char *filepath, double twind, double vertical_wind)
    : Wind(twind, vertical_wind), weathertable(filepath)
{
    snprintf(name, sizeof(name), "Tabular Wind");

    altitude = 0;

    vwind = 0;
    psiwdx = 0;
}

cad::Wind_Tabular::~Wind_Tabular()
{
}

void cad::Wind_Tabular::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;

    vwind = weathertable.look_up("speed", altitude, 0);
    psiwdx = weathertable.look_up("direction", altitude, 0);
}
