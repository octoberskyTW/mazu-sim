#ifndef __ATMOSPHERE_HH__
#define __ATMOSPHERE_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Atmosphere model interface definition)
*******************************************************************************/
#include "aux.hh"

namespace cad
{
class Atmosphere
{
    TRICK_INTERFACE(cad__Atmosphere);

public:
    char name[256];

    Atmosphere() {}

    virtual ~Atmosphere() {}

    virtual void set_altitude(double altitude_in_meter) = 0;

    virtual double get_temperature_in_kelvin() { return tempk; }
    virtual double get_density() { return density; }
    virtual double get_pressure() { return pressure; }
    virtual double get_speed_of_sound() { return vsound; }

    virtual double get_speed_of_wind() { return vwind; }
    virtual double get_direction_of_wind() { return dwind; }

protected:
    double altitude;

    double tempk;    /* *o (K)          Atmospheric temperature */
    double density;  /* *o (kg/m3)      Atmospheric Density */
    double pressure; /* *o (pa)         Atmospheric pressure */
    double vsound;

    double vwind;
    double dwind;
};
}  // namespace cad

#endif  // __ATMOSPHERE_HH__
