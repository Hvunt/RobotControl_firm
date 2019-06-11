#ifndef LIST_PQ_H
#define LIST_PQ_H

#include <stdlib.h>
#include "node.h"
// typedef struct l
// {
//     node_t *nodes;
//     struct l *next;
//     // int length, size;
// } list_t;

struct l
{
    node_t *nodes;
    struct l *next;
    // int length, size;
};

typedef struct l list_t;

void list_add(list_t **list, node_t *node);
int list_get_length(list_t *list);
void list_free(list_t **list);

#endif