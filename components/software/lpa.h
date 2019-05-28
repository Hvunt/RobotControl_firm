
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

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

#include "node.h"
#include "priority_queue.h"
#include "list.h"

typedef struct l list_t;

int lpa_init(node_t *map, int x_MAX, int y_MAX/*, int x_START, int y_START*/);
// int lpa_compute_path(node_t *goal_node);
int lpa_compute_path(node_t *map, int goalX, int goalY);
// void lpa_move(uint8_t x_GOAL, uint8_t y_GOAL);
// void lpa_move_task(void * parameters);
void lpa_get_current_coords(char *data);
void lpa_free(void);

void print_map(node_t *map, node_t *goal_node);
// void write_results();
// void make_obstacles(void);

enum{
    LPA_INIT_POINT_ERROR = -2,
    LPA_INIT_POINT_IS_OBSTACLE, // point is a obstacle or non exist
    LPA_INIT_OK = 1, // lpa is initiated
};

#endif // LPA_H