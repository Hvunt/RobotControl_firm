
#ifndef LPA_H
#define LPA_H

// #include "main.h"

// #include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "esp_heap_caps.h"
// #include "esp_log.h"


#include "node.h"

#include "priority_queue.h"
#include "list.h"
// #include "motor_defs.h"
#include "slave_mcu.h"

typedef struct l list_t;

int lpa_init(/*node_t *map,*/ int x_MAX, int y_MAX);
int lpa_compute_path(/*node_t *map,*/ PQ_list_t *queue, list_t *path, int goalX, int goalY);
void lpa_get_current_coords(char *data);
void lpa_free(PQ_list_t *queue /*, list_t *path*/);

void print_map(node_t *map, node_t *current_node, node_t *goal_node);
// void make_obstacles(void);

enum
{
    LPA_INIT_POINT_ERROR = -10,
    LPA_MOVING_ERROR,
    LPA_PATH_ERROR,
    LPA_INIT_POINT_IS_OBSTACLE, // point is a obstacle or non exist
    LPA_PATH_CANT_BE_FOUND,
    LPA_POINT_CANT_BE_REACHED = -1,

    LPA_OK = 1,
};


/*  snail
    * - - -
    *|2|1|8|
    *|3|X|7|
    *|4|5|6|
    * - - - 
*/

enum
{
    LPA_DIRECTION_SNAIL_1 = 0x01,
    LPA_DIRECTION_SNAIL_2,
    LPA_DIRECTION_SNAIL_3,
    LPA_DIRECTION_SNAIL_4,
    LPA_DIRECTION_SNAIL_5,
    LPA_DIRECTION_SNAIL_6,
    LPA_DIRECTION_SNAIL_7,
    LPA_DIRECTION_SNAIL_8,
};

#endif // LPA_H