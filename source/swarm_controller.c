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

void swarm_controller_set_first_waypoint(struct Swarm_controller* controller,
									  struct Dimensions first_waypoint)
{
	controller->first_waypoint.x = first_waypoint.x;
	controller->first_waypoint.y = first_waypoint.y;
	controller->first_waypoint.z = first_waypoint.z;
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
	return sqrt(pow(d1.x - d2.x, 2) + pow(d1.y - d2.y, 2));
}

double calculate_angle(struct Dimensions v, struct Dimensions a, struct Dimensions b)
{
	return atan2(b.y - v.y, b.x - v.x) - atan2(a.y - v.y, a.x - v.x);
}

bool equal_dimensions(struct Dimensions d1, struct Dimensions d2)
{
	return d1.x == d2.x && d1.y == d2.y;
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


	// Find the closest ASV to the current ASV
	double lowest_distance = DBL_MAX;
	struct Dimensions closest_cog_position;
	int lowest_index = 0;
	int current_index = 0;
	for (struct Simulation* curnode = controller->node; curnode != NULL; curnode = curnode->previous) {
		if (curnode->previous == NULL) {
			for (struct Simulation* curnode2 = curnode; curnode2 != NULL; curnode2 = curnode2->next) {
				double dist = calculate_distance(controller->asv_position, curnode2->asv->cog_position);
				if (dist > 0 && dist < lowest_distance) {
					lowest_distance = dist;
					closest_cog_position.x = curnode2->asv->cog_position.x;
					closest_cog_position.y = curnode2->asv->cog_position.y;
					closest_cog_position.z = curnode2->asv->cog_position.z;
					lowest_index = current_index;
				}

				current_index++;
			}
		}
	}

	// printf("%d | %d\n", current_index, bruh);

	double waypoint_x = controller->old_way_point.x;
	double waypoint_y = controller->old_way_point.y;
	// if (controller->old_way_point.x == 3000 || controller->old_way_point.y == 2000) {
	double distance_threshold = 500;
	if (calculate_distance(controller->asv_position, closest_cog_position) < distance_threshold) {
		double rad_offset = 0.523598776;
		double angle = calculate_angle(controller->asv_position, closest_cog_position, controller->old_way_point);
		if (angle > 0) rad_offset *= -1;

		double radius = calculate_distance(controller->asv_position, controller->old_way_point);
		struct Dimensions offset_waypoint;
		offset_waypoint.x = controller->asv_position.x;
		offset_waypoint.y = controller->old_way_point.y + 2000;
		offset_waypoint.z = controller->asv_position.z;
		double offset_angle = (calculate_angle(controller->asv_position, controller->old_way_point, offset_waypoint));
		waypoint_x = radius * sin(offset_angle + rad_offset) + controller->asv_position.x;
		waypoint_y = radius * cos(offset_angle + rad_offset) + controller->asv_position.y;

		// printf("a: %f | rf: %f | radius: %f | wx: %f | wy: %f| c: %f\n", angle, rad_offset, radius, waypoint_x, waypoint_y, calculate_distance(controller->asv_position, closest_cog_position));
	}

	// if (controller->old_way_point.x == 1005  && controller->asv_position.y < 3000)
	// printf("old: %f | new: %f | x: %f | y: %f\n", controller->old_way_point.x, waypoint_x, controller->asv_position.x, controller->asv_position.y);

	// double waypoint_y = controllerontroller->asv_position.y + (controller->buffer_speed / 100) * (controller->old_way_point.y - controller->asv_position.y) + 1;
	// if (waypoint_y < 2000) printf("BRUH");
	// if (controller->old_way_point.x == 1005)
	// printf("ay: %f | b: %f | owy: %f | nwy: %f\n", controller->asv_position.y, controller->buffer_speed, controller->old_way_point.y, waypoint_y);
	// controller->new_way_point.x = waypoint_x;
	// controller->new_way_point.y = waypoint_y;
	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = waypoint_y;
	controller->new_way_point.z = controller->old_way_point.z;
	// printf("x: %f | y: %f", waypoint_x, waypoint_y);
}
