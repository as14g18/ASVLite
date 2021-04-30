#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H

#include "geometry.h"
#include "simulation.h"

struct Swarm_controller
{
  // Inputs
  // ------
  struct Dimensions asv_position; // Current position of the ASV in the X-Y 
                             // coordinate space. struct Point is for 3D 
                             // coordinate space, ignore z and set it to 0.0m.
  struct Dimensions old_way_point;
  struct Dimensions new_way_point;
  double buffer_speed;
  double calculated_speed;

  struct Simulation* node; // Contains data for other ASVs

  int latency_counter; // Counter for latency time steps
  int latency; // Actual latency
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

void swarm_controller_set_asv_states(struct Swarm_controller* controller, struct Simulation* node);

void swarm_controller_set_latency(struct Swarm_controller* controller, int latency);

void swarm_controller_moderate_speed(struct Swarm_controller* controller);

void swarm_controller_set_new_way_point(struct Swarm_controller* controller);

#endif
