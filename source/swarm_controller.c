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
                                      struct Dimensions position,
                                      struct Dimensions attitude)
{
  controller->asv_position.x = position.x;
  controller->asv_position.y = position.y;
  controller->asv_position.z = position.z;
  controller->asv_attitude.x = attitude.x;
  controller->asv_attitude.y = attitude.y;
  controller->asv_attitude.z = attitude.z;
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

double calculate_slope(struct Dimensions d1, struct Dimensions d2)
{
	if (d2.x == d1.x) return DBL_MAX;
	return (d2.y - d1.y) / (d2.x - d1.x);
}

struct Dimensions rotate(struct Dimensions d, double angle)
{
	struct Dimensions w;
	w.x = d.x * cos(angle) + d.y * sin(angle);
	w.y = -d.x * sin(angle) + d.y * cos(angle);
	w.z = 0;

    return w;
}

double swarm_controller_moderate_speed(struct Swarm_controller* controller)
{	
	double max_distance = DBL_MAX;
	int lowest_waypoint_index = 1;
	if (controller->node->previous != NULL) {
		int cur_index = controller->node->previous->current_waypoint_index;
		double dist = calculate_distance(
			controller->node->previous->asv->cog_position,
			controller->node->previous->waypoints->points[cur_index]
		);

		if (dist < max_distance) max_distance = dist;
		if (cur_index < lowest_waypoint_index) lowest_waypoint_index = cur_index;
	}

	if (controller->node->next != NULL) {
		int cur_index = controller->node->next->current_waypoint_index;
		double dist = calculate_distance(
			controller->node->next->asv->cog_position,
			controller->node->next->waypoints->points[cur_index]
		);

		if (dist < max_distance) max_distance = dist;
		if (cur_index < lowest_waypoint_index) lowest_waypoint_index = cur_index;
	}

	double speed = 1;
	double cur_distance = calculate_distance(controller->asv_position, controller->old_way_point);
	if ((controller->node->waypoints->points[0].x == 3000 && controller->node->current_waypoint_index == 1) || controller->node->current_waypoint_index > lowest_waypoint_index || cur_distance < max_distance) {
		speed = 0;
	}
	// if (controller->node->waypoints->points[0].x == 3000) printf("%f\n", speed);
	return speed;
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

	double waypoint_x = controller->old_way_point.x;
	double waypoint_y = controller->old_way_point.y;
	double distance_threshold = 500;
	if (calculate_distance(controller->asv_position, closest_cog_position) < distance_threshold) {
		double rad_offset = 0.1323598776;
		// double angle = calculate_angle(controller->asv_position, closest_cog_position, controller->old_way_point);
		// if (angle > 0) rad_offset *= -1;

		// double radius = calculate_distance(controller->asv_position, controller->old_way_point);
		// struct Dimensions offset_waypoint;
		// offset_waypoint.x = controller->asv_position.x;
		// offset_waypoint.y = controller->old_way_point.y + 2000;
		// offset_waypoint.z = controller->asv_position.z;
		// double offset_angle = (calculate_angle(controller->asv_position, controller->old_way_point, offset_waypoint));
		// waypoint_x = radius * sin(rad_offset) + controller->asv_position.x;
		// waypoint_y = radius * cos(rad_offset) + controller->asv_position.y;

		struct Dimensions a1 = rotate(controller->old_way_point, rad_offset);
		struct Dimensions a2 = rotate(controller->old_way_point, rad_offset * -1);

		if (controller->node->waypoints->points[0].x == 2500)
		printf("c: %f | a1x: %f | a1y: %f | a2x: %f | a2y: %f\n", calculate_distance(controller->asv_position, closest_cog_position), a1.x, a1.y, a2.x, a2.y);

		if (calculate_distance(a1, closest_cog_position) > calculate_distance(a2, closest_cog_position)) {
			waypoint_x = a1.x;
			waypoint_y = a1.y;
		} else {
			waypoint_x = a2.x;
			waypoint_y = a2.y;
		}
	}

	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = waypoint_y;
	// controller->new_way_point.x = controller->old_way_point.x;
	// controller->new_way_point.y = controller->old_way_point.y;
	controller->new_way_point.z = controller->old_way_point.z;
}
