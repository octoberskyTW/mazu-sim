#ifndef __SCHMITT_TRIGGER_HH__
#define __SCHMITT_TRIGGER_HH__

#include "aux.hh"

class Schmitt_Trigger
{
    TRICK_INTERFACE(Schmitt_Trigger);

public:
    Schmitt_Trigger(double dead_zone_In, double hysteresis_In);
    Schmitt_Trigger(const Schmitt_Trigger &other);

    Schmitt_Trigger &operator=(const Schmitt_Trigger &other);

    int trigger(double in);

    void clear();

private:
    double dead_zone;  /* *o  (--)    Dead zone of Schmitt trigger */
    double hysteresis; /* *o  (--)    Hysteresis of Schmitt trigger */

    double saved_value; /* *o  (--)    Saved Value */
};

#endif  // __SCHMITT_TRIGGER_HH__