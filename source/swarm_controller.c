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
									  struct Dimensions first_waypoint, int current_waypoint_index)
{
	controller->first_waypoint.x = first_waypoint.x;
	controller->first_waypoint.y = first_waypoint.y;
	controller->first_waypoint.z = first_waypoint.z;
	controller->current_waypoint_index = current_waypoint_index;
}

void swarm_controller_set_asv_states(struct Swarm_controller* controller,
								  struct Simulation* node)
{
	controller->is_previous_null = node->previous == NULL;
	if (!controller->is_previous_null) {
		controller->previous_waypoint_index = node->previous->current_waypoint_index;
		controller->previous_cog_position.x = node->previous->asv->cog_position.x;
		controller->previous_cog_position.y = node->previous->asv->cog_position.y;
		controller->previous_cog_position.z = node->previous->asv->cog_position.z;
		controller->previous_waypoint.x = node->previous->waypoints->points[controller->previous_waypoint_index].x;
		controller->previous_waypoint.y = node->previous->waypoints->points[controller->previous_waypoint_index].y;
		controller->previous_waypoint.z = node->previous->waypoints->points[controller->previous_waypoint_index].z;
	}

	controller->is_next_null = node->next == NULL;
	if (!controller->is_next_null) {
		controller->next_waypoint_index = node->next->current_waypoint_index;
		controller->next_cog_position.x = node->next->asv->cog_position.x;
		controller->next_cog_position.y = node->next->asv->cog_position.y;
		controller->next_cog_position.z = node->next->asv->cog_position.z;
		controller->next_waypoint.x = node->next->waypoints->points[controller->next_waypoint_index].x;
		controller->next_waypoint.y = node->next->waypoints->points[controller->next_waypoint_index].y;
		controller->next_waypoint.z = node->next->waypoints->points[controller->next_waypoint_index].z;
	}
}

int swarm_controller_set_latency(struct Swarm_controller* controller, int latency)
{
	controller->latency = latency;

	return controller->latency_counter;
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
	double total_distance = 0;
	double count = 0;
	int lowest_waypoint_index = 1;
	if (!controller->is_previous_null) {
		int cur_index = controller->previous_waypoint_index;
		double dist = calculate_distance(
			controller->previous_cog_position,
			controller->previous_waypoint
		);

		total_distance += dist;
		count++;
		if (cur_index < lowest_waypoint_index) lowest_waypoint_index = cur_index;
	}

	if (!controller->is_next_null) {
		int cur_index = controller->next_waypoint_index;
		double dist = calculate_distance(
			controller->next_cog_position,
			controller->next_waypoint
		);

		total_distance += dist;
		count++;
		if (cur_index < lowest_waypoint_index) lowest_waypoint_index = cur_index;
	}

	double speed = 1;
	double average_distance = total_distance / count;
	double cur_distance = calculate_distance(controller->asv_position, controller->old_way_point);
	if (controller->current_waypoint_index > lowest_waypoint_index || cur_distance < average_distance) {
		speed = 0;
	}

	return speed;
}

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
	double waypoint_x = controller->old_way_point.x;
	double waypoint_y = controller->old_way_point.y;

	if (!controller->is_next_null) {
		double distance = calculate_distance(controller->asv_position, controller->next_cog_position);
		if (distance < 500) {
			waypoint_x = controller->asv_position.x;
			waypoint_y = controller->asv_position.y + 5000;
		}
	}

	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = waypoint_y;
	controller->new_way_point.z = controller->old_way_point.z;

	controller->latency_counter--;
	if (controller->latency_counter < 0) {
		controller->latency_counter = controller->latency;
	}
}
