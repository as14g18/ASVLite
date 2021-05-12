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
  struct Dimensions asv_attitude; // Current roll, pitch and yaw angles of
                                // ASV. The angles are measured in radians.
  struct Dimensions old_way_point; // The original waypoint specified in the config file
  int current_waypoint_index;
  int latency; // Latency of controller

  // Outputs
  // ------
  struct Dimensions new_way_point; // The newly calculated waypoint

  // Data for other ASVs
    // -----------------
  bool is_previous_null;
  int previous_waypoint_index;
  struct Dimensions previous_cog_position;
  struct Dimensions previous_waypoint;
  bool is_next_null;
  int next_waypoint_index;
  struct Dimensions next_cog_position;
  struct Dimensions next_waypoint;

  // Intermediate variables
    // --------------------
  int latency_counter; // Counter for latency time steps
};

/**
 * Function to initalise the intermediate variables of the controller
 */
void swarm_controller_init(struct Swarm_controller* controller);

/**
 * Function to set the current position and attitude of the ASV.
 * @param controller for which the inputs are to be set.
 * @param position is the current position of the ASV. 
 * @param attitude is the current attitude of the ASV. Angles are in radians.
 */
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

/**
 * Function to set the first waypoint of the ASV
 */
void swarm_controller_set_first_waypoint(struct Swarm_controller* controller,
									  struct Dimensions first_waypoint, int current_waypoint_index);

/**
 * Sets the values of neighbouring ASVs as input to the controller
 */
void swarm_controller_set_asv_states(struct Swarm_controller* controller, struct Simulation* node);

/**
 * Sets the latency of the controller
 */
int swarm_controller_set_latency(struct Swarm_controller* controller, int latency);

/**
 * Speed regulation layer of the swarm controller
 */
double swarm_controller_moderate_speed(struct Swarm_controller* controller);

/**
 * Distance regulation layer of the sawrm controller
 */
void swarm_controller_set_new_way_point(struct Swarm_controller* controller);

#endif
