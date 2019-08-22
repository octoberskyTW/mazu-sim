#ifndef __WIND_TABULAR_HH__
#define __WIND_TABULAR_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (tabular wind model)
LIBRARY DEPENDENCY:
      ((../../src/env/wind_tabular.cpp))
*******************************************************************************/

#include "datadeck.hh"
#include "wind.hh"

namespace cad
{
class Wind_Tabular : public Wind
{
public:
    Wind_Tabular(char *filepath, double twind_In, double vertical_wind);

    virtual ~Wind_Tabular();

    virtual void set_altitude(double altitude_in_meter);

private:
    Datadeck weathertable;
};
}  // namespace cad

#endif  // __WIND_TABULAR_HH__
