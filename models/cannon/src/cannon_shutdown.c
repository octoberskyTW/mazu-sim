/************************************************************************
PURPOSE: (Print the final cannon ball state.)
*************************************************************************/
#include "cannon.h"
#include "trick/exec_proto.h"
#include <stdio.h>

int cannon_shutdown(CANNON* C) {
    double t = exec_get_sim_time();
    printf("========================================\n");
    printf("      Cannon Ball State at Shutdown     \n");
    printf("t = %g, pos = [%g, %g], vel = [%g, %g]\n", t, C->pos[0], C->pos[1], C->vel[0], C->vel[1]);
    printf("========================================\n");
    C->impactTime = 0.0;
    C->impact = 0.0;
    return 0;
}
