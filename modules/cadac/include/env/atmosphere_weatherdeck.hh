#ifndef __ATMOSPHERE_WEATHERDECK_HH__
#define __ATMOSPHERE_WEATHERDECK_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (tabular atmosphere from WEATHER_DECK)
LIBRARY DEPENDENCY:
      ((../../src/env/atmosphere_weatherdeck.cpp))
*******************************************************************************/

#include "env/atmosphere.hh"
#include "datadeck.hh"


namespace cad
{
class Atmosphere_weatherdeck : public Atmosphere
{
public:
    explicit Atmosphere_weatherdeck(char *filepath);

    virtual ~Atmosphere_weatherdeck();

    virtual void set_altitude(double altitude_in_meter);

private:
    Datadeck weathertable;

    void update_values();
};
}  // namespace cad

#endif  // __ATMOSPHERE_WEATHERDECK_HH__
