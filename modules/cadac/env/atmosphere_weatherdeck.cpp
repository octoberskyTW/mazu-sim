#include "atmosphere_weatherdeck.hh"
#include "datadeck.hh"

#include "cadac_constants.hh"

#include <cmath>
#include <cstring>

cad::Atmosphere_weatherdeck::Atmosphere_weatherdeck(char *filepath)
    : weathertable(filepath)
{
    snprintf(name, sizeof(name), "Atmosphere Weather Deck");

    altitude = 0;

    update_values();
}

cad::Atmosphere_weatherdeck::~Atmosphere_weatherdeck()
{
}

void cad::Atmosphere_weatherdeck::set_altitude(double altitude_in_meter)
{
    altitude = altitude_in_meter;

    update_values();
}

void cad::Atmosphere_weatherdeck::update_values()
{
    density = weathertable.look_up("density", altitude, 0);
    pressure = weathertable.look_up("pressure", altitude, 0);
    tempk = weathertable.look_up("temperature", altitude, 0) + 273.16;

    vsound = sqrt(1.4 * RGAS * tempk);
}
