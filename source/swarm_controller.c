#include "swarm_controller.h"
#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void swarm_controller_init(struct Swarm_controller* controller)
{
  controller->buffer_speed = 100;
  controller->latency_counter = 0;
}

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
	double counter = 0;
	for (struct Simulation* curnode = controller->node->previous; curnode != NULL; curnode = curnode->previous) {
		y_sum += curnode->asv->cog_position.y;
		counter++;
	}

	for (struct Simulation* curnode = controller->node->next; curnode != NULL; curnode = curnode->next) {
		y_sum += curnode->asv->cog_position.y;
		counter++;
	}

	double average_y = y_sum / counter;
	double diff = controller->asv_position.y - average_y;
	// if (diff > 2 && (controller->old_way_point.x == 1010 || controller->old_way_point.x == 2010))
	// printf("diff: %f | y: %f\n", diff, controller->asv_position.y);
	double speed = diff > 2 ? 0 : 100;

	// if (controller->latency_counter > 1)
	// printf("lc: %d\n", controller->latency_counter);
	controller->latency_counter = controller->latency_counter + 1;
	if (controller->latency_counter == controller->latency) {
		controller->latency_counter = 0;
		controller->buffer_speed = speed;
		// if (speed == 0 && (controller->old_way_point.x == 1010 || controller->old_way_point.x == 2010)) printf("BRUH %f\n", controller->asv_position.y);
	}
}

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
	double corridor_distance = 1;
	double difference = controller->asv_position.x - controller->old_way_point.x;
	double multiplier = difference < 0 ? -1 : 1;
	double waypoint_x = controller->old_way_point.x;
	if (abs(difference) > corridor_distance) {
		difference = 1000 * multiplier;
		if (controller->buffer_speed != 0)
			controller->buffer_speed = 2;

		waypoint_x = controller->old_way_point.x - difference;
		
		// if (controller->old_way_point.x == 1005  && controller->asv_position.y < 3000)
		// printf("old: %f | new: %f | x: %f | y: %f\n", controller->old_way_point.x, waypoint_x, controller->asv_position.x, controller->asv_position.y);
	}

	double waypoint_y = controller->asv_position.y + (controller->buffer_speed / 100) * (controller->old_way_point.y - controller->asv_position.y) + 1;
	// if (waypoint_y < 2000) printf("BRUH");
	// if (controller->old_way_point.x == 1005)
	// printf("ay: %f | b: %f | owy: %f | nwy: %f\n", controller->asv_position.y, controller->buffer_speed, controller->old_way_point.y, waypoint_y);
	// controller->new_way_point.x = waypoint_x;
	// controller->new_way_point.y = waypoint_y;
	controller->new_way_point.x = controller->old_way_point.x;
	controller->new_way_point.y = controller->old_way_point.y;
	controller->new_way_point.z = controller->old_way_point.z;
}
