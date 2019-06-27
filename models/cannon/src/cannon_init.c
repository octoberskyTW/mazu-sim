/******************************* TRICK HEADER ****************************
PURPOSE: (Set the initial data values)
LIBRARY DEPENDENCIES:
    (
    )
*************************************************************************/

/* Model Include files */
#include <math.h>
#include "cannon.h"

/* initialization job */
int cannon_init(CANNON* C) {
    C->vel0[0] = C->init_speed * cos(C->init_angle);
    C->vel0[1] = C->init_speed * sin(C->init_angle);
    C->vel[0] = C->vel0[0];
    C->vel[1] = C->vel0[1];
    C->impactTime = 0.0;
    C->impact = 0.0;
    return 0;
}
