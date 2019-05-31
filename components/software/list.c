
#include "list.h"

void list_add(list_t **list, node_t *node)
{
    list_t *new_node = (list_t *)malloc(sizeof(list_t));
    new_node->nodes = node;
    new_node->next = *list;
    *list = new_node;
}

int list_get_length(list_t *list)
{
    int length = 0;
    list_t *prev = list;
    while (prev != NULL)
    {
        length++;
        prev = prev->next;
    }
    return length;
}

void list_free(list_t **list)
{
    if (*list != NULL)
    {
        list_t *prev = *list;
        while (*list)
        {
            *list = (*list)->next;
            free(prev);
            prev = *list;
        }
    }
}