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

void swarm_controller_moderate_speed(struct Swarm_controller* controller)
{
	double y_sum = 0;
	int counter = 0;
	for (struct Simulation* curnode = controller->node->previous; curnode != NULL; curnode = curnode->previous) {
		y_sum += curnode->asv->cog_position.y;
		counter++;
	}

	for (struct Simulation* curnode = controller->node->next; curnode != NULL; curnode = curnode->next) {
		y_sum += curnode->asv->cog_position.y;
		counter++;
	}

	double average_y = y_sum / counter;
	double diff = controller->node->asv->cog_position.y / average_y;
	double speed = 100;

	if (diff > 1) {
		if (diff < 1.25) {
			speed = 75;
		}
		else if (diff < 1.5) {
			speed = 50;
		}
		else if (diff < 1.75) {
			speed = 25;
		}
		else {
			speed = 0;
		}
	}

	controller->calculated_speed = speed;

	controller->latency_counter = controller->latency_counter + 1;
	if (controller->latency_counter == controller->latency) {
		controller->latency_counter = 0;
		controller->buffer_speed = controller->calculated_speed;
	}
}

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
	double corridor_distance = 5;
	double difference = controller->asv_position.x - controller->old_way_point.x;
	double multiplier = difference < 0 ? -1 : 1;
	double waypoint_x = controller->old_way_point.x;
	if (abs(difference) > corridor_distance) {
		if (abs(difference) < 10) {
			difference = 10 * multiplier;
		}
		else if (abs(difference) < 15) {
			difference = 15 * multiplier;
		}
		else if (abs(difference) < 20) {
			difference = 20 * multiplier;
		}
		else if (abs(difference) < 25) {
			difference = 25 * multiplier;
		}
		else {
			difference = 30 * multiplier;
		}

		waypoint_x = controller->old_way_point.x - difference;
		
		// if (controller->old_way_point.x == 1020  && controller->asv_position.y < 2000)
		// printf("old: %f | new: %f | x: %f | y: %f\n", controller->old_way_point.x, waypoint_x, controller->asv_position.x, controller->asv_position.y);
	}

	double waypoint_y = controller->asv_position.y + (controller->buffer_speed / 100) * (controller->old_way_point.y - controller->asv_position.y);

	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = controller->old_way_point.y;
	controller->new_way_point.z = controller->old_way_point.z;
}
