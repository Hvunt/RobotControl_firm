
#include "list.h"

void list_add(list_t **list, node_t *node){
    list_t *new_node = (list_t *) malloc(sizeof(list_t));
    new_node->nodes = node;
    new_node->next = *list;
    *list = new_node;
}