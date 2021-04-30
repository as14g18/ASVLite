#include "swarm_controller.h"
#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void swarm_controller_set_current_state(struct Swarm_controller* controller,
                                      struct Dimensions position)
{
  controller->asv_position.x = position.x;
  controller->asv_position.y = position.y;
  controller->asv_position.z = position.z;
}

void swarm_controller_set_old_way_point(struct Swarm_controller* controller,
                                  struct Dimensions way_point)
{
  controller->old_way_point.x = way_point.x;
  controller->old_way_point.y = way_point.y;
  controller->old_way_point.z = way_point.z;
}

void swarm_controller_set_asv_states(struct Swarm_controller* controller,
								  struct Simulation* node)
{
	controller->node = node;
}

void swarm_controller_set_latency(struct Swarm_controller* controller, int latency)
{
	controller->latency = latency;
}

void swarm_controller_increment_latency_counter(struct Swarm_controller* controller)
{
	controller->latency_counter = controller->latency_counter + 1;
	if (controller->latency_counter == controller->latency) {
		controller->latency_counter = 0;
		controller->updated_way_point.x = controller->new_way_point.x;
		controller->updated_way_point.y = controller->new_way_point.y;
		controller->updated_way_point.z = controller->new_way_point.z;
	}
}

double calculate_distance(struct Dimensions d1, struct Dimensions d2)
{
	return sqrt(pow(d1.x - d2.x, 2) + pow(d1.y - d2.y, 2) + pow(d1.z - d2.z, 2));
}

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
	// struct Simulation asv1 = controller->fnode;
	// struct Simulation asv2 = controller->fnode->next;

	// double target_distance = calculate_distance(
	// 	asv1->waypoints->points[asv1->current_waypoint_index],
	// 	asv2->waypoints->points[asv2->current_waypoint_index]
	// )

	double corridor_distance = 2;
	double difference = controller->asv_position.x - controller->old_way_point.x;
	double waypoint_x;
	if (controller->asv_position.x > controller->old_way_point.x + corridor_distance ||
		controller->asv_position.x < controller->old_way_point.x - corridor_distance) {
		if (abs(difference) < 5) {
			if (difference < 0) {
				difference = -5;
			} else {
				difference = 5;
			}
		}
		else if (abs(difference) < 10) {
			if (difference < 0) {
				difference = -10;
			} else {
				difference = 10;
			}
		}
		else if (abs(difference) < 15) {
			if (difference < 0) {
				difference = -15;
			} else {
				difference = 15;
			}
		}
		else if (abs(difference) < 20) {
			if (difference < 0) {
				difference = -20;
			} else {
				difference = 20;
			}
		}
		else {
			if (difference < 0) {
				difference = -25;
			} else {
				difference = 25;
			}
		}

		waypoint_x = controller->old_way_point.x - difference;
	}

	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = controller->old_way_point.y;
	controller->new_way_point.z = controller->old_way_point.z;
}
