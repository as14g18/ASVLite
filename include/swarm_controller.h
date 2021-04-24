#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H

#include "geometry.h"

struct Swarm_controller
{
  // Inputs
  // ------
  struct Dimensions asv_position; // Current position of the ASV in the X-Y 
                             // coordinate space. struct Point is for 3D 
                             // coordinate space, ignore z and set it to 0.0m.
  struct Dimensions old_way_point; // Desired position.
  struct Dimensions new_way_point; // Desired position.
};


void swarm_controller_set_current_state(struct Swarm_controller* controller,
                                      struct Dimensions position);

/**
 * Function to set the destination point for the ASV.
 * @param controller for which the way-point is to be set.
 * @param way_point desired destination point for the ASV.
 */
void swarm_controller_set_old_way_point(struct Swarm_controller* controller,
                                  struct Dimensions way_point);

void swarm_controller_set_new_way_point(struct Swarm_controller* controller);

#endif
