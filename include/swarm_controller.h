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
  struct Dimensions asv_attitude;
  struct Dimensions first_waypoint;
  struct Dimensions old_way_point;
  struct Dimensions new_way_point;
  int current_waypoint_index;
  double buffer_speed;

  // data for other ASVs
  bool is_previous_null;
  int previous_waypoint_index;
  struct Dimensions previous_cog_position;
  struct Dimensions previous_waypoint;
  bool is_next_null;
  int next_waypoint_index;
  struct Dimensions next_cog_position;
  struct Dimensions next_waypoint;

  int latency_counter; // Counter for latency time steps
  int latency; // Actual latency
};

void swarm_controller_init(struct Swarm_controller* controller);

void swarm_controller_set_current_state(struct Swarm_controller* controller,
                                      struct Dimensions position,
                                      struct Dimensions attitude);

/**
 * Function to set the destination point for the ASV.
 * @param controller for which the way-point is to be set.
 * @param way_point desired destination point for the ASV.
 */
void swarm_controller_set_old_way_point(struct Swarm_controller* controller,
                                  struct Dimensions way_point);

void swarm_controller_set_first_waypoint(struct Swarm_controller* controller,
									  struct Dimensions first_waypoint, int current_waypoint_index);

void swarm_controller_set_asv_states(struct Swarm_controller* controller, struct Simulation* node);

int swarm_controller_set_latency(struct Swarm_controller* controller, int latency);

double swarm_controller_moderate_speed(struct Swarm_controller* controller);

void swarm_controller_set_new_way_point(struct Swarm_controller* controller);

#endif
