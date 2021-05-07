#include "swarm_controller.h"
#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

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

void swarm_controller_set_origin_position(struct Swarm_controller* controller,
									  struct Dimensions origin_position)
{
	controller->origin_position.x = origin_position.x
	controller->origin_position.y = origin_position.y
	controller->origin_position.z = origin_position.z
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

double calculate_distance(struct Dimensions d1, struct Dimensions d2)
{
	return sqrt(pow(d1.x - d2.x, 2) + pow(d1.y - d2.y, 2) + pow(d1.z - d2.z, 2));
}

bool equal_dimensions(struct Dimensions d1, struct Dimension d2)
{
	return d1.x == d2.x && d1.y == d2.y && d1.z == d2.z
}

void swarm_controller_moderate_speed(struct Swarm_controller* controller)
{
	// double y_sum = 0;
	// double counter = 0;
	// for (struct Simulation* curnode = controller->node->previous; curnode != NULL; curnode = curnode->previous) {
	// 	y_sum += curnode->asv->cog_position.y;
	// 	counter++;
	// }

	// for (struct Simulation* curnode = controller->node->next; curnode != NULL; curnode = curnode->next) {
	// 	y_sum += curnode->asv->cog_position.y;
	// 	counter++;
	// }

	// double average_y = y_sum / counter;
	// double diff = controller->asv_position.y - average_y;
	// // if (diff > 2 && (controller->old_way_point.x == 1010 || controller->old_way_point.x == 2010))
	// // printf("diff: %f | y: %f\n", diff, controller->asv_position.y);
	// double speed = diff > 2 ? 0 : 100;

	// // if (controller->latency_counter > 1)
	// // printf("lc: %d\n", controller->latency_counter);
	// controller->latency_counter = controller->latency_counter + 1;
	// if (controller->latency_counter == controller->latency) {
	// 	controller->latency_counter = 0;
	// 	controller->buffer_speed = speed;
	// 	// if (speed == 0 && (controller->old_way_point.x == 1010 || controller->old_way_point.x == 2010)) printf("BRUH %f\n", controller->asv_position.y);
	// }
}

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
	// Find first node in the linked list
	struct Simulation first_node;
	for (struct Simulation* curnode = controller->node->previous; curnode != NULL; curnode = curnode->previous) {
		if (controller->node->previous == NULL) {
			first_node = curnode;
		}
	}

	// Find the two closest ASVs to the current ASV
	double lowest_distance = DBL_MAX;
	struct Simulation closest_node;
	int lowest_index = 0;
	int current_index = 0;
	for (struct Simulation* curnode = first_node; curnode != NULL; curnode = curnode->next) {
		dist = calculate_distance(curnode->asv->cog_position, controller->asv_position);
		if (!equal_dimensions(curnode->asv->origin_position, controller->origin_position) && dist < lowest_distance) {
			lowest_distance = dist;
			closest_node = curnode;
			lowest_index = current_index;
		}

		current_index++;
	}

	// double second_lowest_distance = DBL_MAX;
	// struct Simulation second_closest_node;
	// int current_index2 = 0;
	// for (struct Simulation* curnode = first_node; curnode != NULL; curnode = curnode->next) {
	// 	if (current_index2 != lowest_index) {
	// 		dist = calculate_distance(curnode->asv->cog_position, controller->asv_position);
	// 		if (!equal_dimensions(curnode->asv->origin_position, controller->origin_position) && dist < second_lowest_distance) {
	// 			second_lowest_distance = dist;
	// 			second_closest_node = curnode;
	// 		}
	// 	}

	// 	current_index2++;
	// }

	if (closest_node->asv->)

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
