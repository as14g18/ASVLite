#include "swarm_controller.h"
#include "constants.h"

#include <stdio.h>
#include <stdlib.h>

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

void swarm_controller_set_new_way_point(struct Swarm_controller* controller)
{
  controller->new_way_point.x = controller->old_way_point.x;
  controller->new_way_point.y = controller->old_way_point.y;
  controller->new_way_point.z = controller->old_way_point.z;
}
