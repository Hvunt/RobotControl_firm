#ifndef LIST_H
#define LIST_H

#include "node.h"

typedef struct l
{
    node_t *nodes;
    struct l *next;
    // int length, size;
} list_t;

void list_add(list_t **list, node_t *node);

#endif