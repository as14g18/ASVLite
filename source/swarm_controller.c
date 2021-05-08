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

double swarm_controller_moderate_speed(struct Swarm_controller* controller)
{	
	struct Dimensions prevpoint;
	struct Dimensions nextpoint;
	if (controller->node->previous != NULL) {
		prevpoint.x = controller->node->previous->asv->cog_position.x;
		prevpoint.y = controller->node->previous->asv->cog_position.y;
		prevpoint.z = controller->node->previous->asv->cog_position.z;
	} else {
		prevpoint.x = controller->node->next->next->asv->cog_position.x;
		prevpoint.y = controller->node->next->next->asv->cog_position.y;
		prevpoint.z = controller->node->next->next->asv->cog_position.z;
	}

	if (controller->node->next != NULL) {
		nextpoint.x = controller->node->next->asv->cog_position.x;
		nextpoint.y = controller->node->next->asv->cog_position.y;
		nextpoint.z = controller->node->next->asv->cog_position.z;
	} else {
		nextpoint.x = controller->node->previous->previous->asv->cog_position.x;
		nextpoint.y = controller->node->previous->previous->asv->cog_position.y;
		nextpoint.z = controller->node->previous->previous->asv->cog_position.z;
	}

	double asv_slope = calculate_slope(prevpoint, nextpoint);
	double angle = M_PI - fabs(atan(asv_slope) - (M_PI / 2));
	double neg_angle = M_PI - angle;

	if (asv_slope > 0) {
		if (angle > 90) {
			angle *= -1;
		} else {
			neg_angle *= -1;
		}
	} else {
		if (angle < 90) {
			angle *= -1;
		} else {
			neg_angle *= -1;
		}
	}

	if (angle < 0) {
		double temp = neg_angle;
		neg_angle = angle;
		angle = temp;
	}

	double speed = 0.1;
	if (controller->asv_attitude.z > neg_angle && controller->asv_attitude.z < angle) {
		speed = 1;
	}

	if (controller->node->waypoints->points[0].x == 1000) printf("%f | %f | %f | %f\n", angle, neg_angle, controller->asv_attitude.z, speed);


	// double average_distance = total_distance / count;
	// double current_distance = calculate_distance(
	// 	controller->asv_position,
	// 	controller->node->waypoints->points[1]
	// );

	// double speed_diff = average_distance - current_distance;
	// double speed;
	// if (speed_diff > 100) {
	// 	speed = 0.1;
	// } else if (speed_diff > 80) {
	// 	speed = 0.15;
	// } else if (speed_diff > 60) {
	// 	speed = 0.2;
	// } else if (speed_diff > 40) {
	// 	speed = 0.25;
	// } else if (speed_diff > 20) {
	// 	speed = 0.3;
	// } else {
	// 	speed = 1;
	// }

	// double speed = current_distance / average_distance;
	// if (controller->node->waypoints->points[0].x == 1000)
	// 	printf("%f\n", speed);
	// 	printf("%f\n", controller->asv_attitude.z);
	// printf("s: %f | ad: %f | cd: %f | td: %f | c: %f | %f | %f\n", speed, average_distance, current_distance, total_distance, count, controller->asv_position.x, controller->asv_position.y);

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
		double rad_offset = 0.123598776;
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

	if (controller->buffer_speed = 0) {
		waypoint_x = controller->asv_position.x;
		waypoint_y = controller->asv_position.y;
	}

	// controller->new_way_point.x = waypoint_x;
	// controller->new_way_point.y = waypoint_y;
	controller->new_way_point.x = waypoint_x;
	controller->new_way_point.y = waypoint_y;
	controller->new_way_point.z = controller->old_way_point.z;
}
