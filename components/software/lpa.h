
#ifndef LPA_H
#define LPA_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

#include "node.h"
#include "priority_queue.h"
#include "list.h"

typedef struct l list_t;

int lpa_init(/*node_t *map,*/ int x_MAX, int y_MAX);
int lpa_compute_path(/*node_t *map,*/ PQ_list_t *queue, list_t *path, int goalX, int goalY);
void lpa_get_current_coords(char *data);
void lpa_free(PQ_list_t *queue, list_t *path);

void print_map(node_t *map, node_t *current_node, node_t *goal_node);
// void make_obstacles(void);

enum{
    LPA_INIT_POINT_ERROR = -10,
    LPA_INIT_POINT_IS_OBSTACLE, // point is a obstacle or non exist
    LPA_PATH_CANT_BE_FOUND = -1,

    LPA_OK = 1,
};

#endif // LPA_H