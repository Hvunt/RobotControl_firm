
#ifndef NODE_H
#define NODE_H

#include <stdbool.h>

typedef struct node
{
    float rhs, g;
    int x, y;
    bool isObstacle;
    bool isVisited;
    bool isPath;
} node_t;

int Node_isConsistent(node_t *node);
float Node_getHeuristic(node_t *current_node, node_t *goal_node);

#endif // NODE_H