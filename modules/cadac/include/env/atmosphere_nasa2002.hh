#ifndef __ATMOSPHERE_NASA2002_HH__
#define __ATMOSPHERE_NASA2002_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (US 1976 Standard Atmosphere (NASA Marshall))
LIBRARY DEPENDENCY:
      ((../../src/env/atmosphere_nasa2002.cpp))
*******************************************************************************/
#include "env/atmosphere.hh"

namespace cad
{
class Atmosphere_nasa2002 : public Atmosphere
{
public:
    Atmosphere_nasa2002();

    virtual ~Atmosphere_nasa2002();

    virtual void set_altitude(double altitude_in_meter);

private:
    int update_values();
};
}  // namespace cad

#endif  // __ATMOSPHERE_NASA2002_HH__
