#ifndef PRIORITY_QUEUE
#define PRIORITY_QUEUE

#include "node.h"

typedef struct list
{
    node_t *data;
    float priority[2];
    struct list *next;
} PQ_list_t;

void PQ_push(PQ_list_t **list, float *priority, node_t *data);
node_t *PQ_pop(PQ_list_t **list);
float *PQ_getTopKey(PQ_list_t *list);

PQ_list_t *PQ_sort(PQ_list_t *list);
PQ_list_t *PQ_getMiddle(PQ_list_t *head);
PQ_list_t *PQ_mergeLists(PQ_list_t *l, PQ_list_t *r);
void PQ_remove(PQ_list_t **list, node_t *node);
int PQ_contains(PQ_list_t *list, node_t *node);
long PQ_SizeOfQueue(PQ_list_t *list);
void PQ_free(PQ_list_t **list);
#endif // PRIORITY_QUEUE