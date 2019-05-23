
#ifndef LPA_H
#define LPA_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "node.h"
#include "priority_queue.h"
#include "list.h"

typedef struct l list_t;

int lpa_init(int x_MAX, int y_MAX, int x_START, int y_START/*, int x_GOAL, int y_GOAL*/);
int lpa_compute_path(void);
void get_predecessors(list_t **pred_list, node_t *current_node);
void get_successors(list_t **suc_list, node_t *current_node);
node_t *get_min_pred(node_t *current_node);
void update_node(node_t *node);
void make_path(void);

void map_init(node_t *map_, int x_MAX, int y_MAX);
node_t * get_node_coord(int x, int y);
// // node_t *get_node_values(int g_rhs, bool is_g);
float get_cost(node_t *from, node_t *to);
void calc_key(float *key, node_t *current_node, node_t *goal_node);

void print_map(node_t *node);
void write_results();
void make_obstacles(void);

enum{
    LPA_INIT_POINT_ERROR = -2,
    LPA_INIT_POINT_IS_OBSTACLE, // point is a obstacle or non exist
    LPA_INIT_OK = 1, // lpa is initiated
};

#endif // LPA_H