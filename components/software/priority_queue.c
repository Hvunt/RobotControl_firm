#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "priority_queue.h"

void PQ_push(PQ_list_t **list, float *pr, node_t *data)
{

    PQ_list_t *new_node = (PQ_list_t *)malloc(sizeof(PQ_list_t));
    new_node->priority[0] = pr[0];
    new_node->priority[1] = pr[1];
    new_node->data = data;
    new_node->next = *list;
    *list = new_node;

    *list = PQ_sort(*list);
}

node_t *PQ_pop(PQ_list_t **list)
{
    if ((*list) != NULL)
    {
        node_t *node = (*list)->data;
        PQ_list_t *temp = (*list);
        (*list) = (*list)->next;
        free(temp);
        // printf("node X: %d, Y: %d \n", node->x, node->y);
        // printf("key[0]: %f, key[1]: %f \n", temp->priority[0], temp->priority[1]);
        // free(temp);
        return node;
    }
    return NULL;
}

float *PQ_getTopKey(PQ_list_t *list)
{
    if (list != NULL)
        return list->priority;
    else
        return NULL;
}

int PQ_contains(PQ_list_t *list, node_t *node)
{
    if (list != NULL)
    {
        PQ_list_t *temp = list;
        while (temp->next != NULL)
        {
            if (temp->data->x == node->x && temp->data->y == node->y)
                return 1;
            temp = temp->next;
        }
    }
    return 0;
}

void PQ_remove(PQ_list_t **list, node_t *node)
{

    if (list != NULL)
    {
        PQ_list_t *temp = *list, *prev = NULL;
        while (temp->data != node && temp->next != NULL)
        {
            prev = temp;
            temp = temp->next;
        }
        if (temp->data == node)
        {
            if (prev)
                prev->next = temp->next;
            else
                *list = temp->next;
        }
        free(temp);
    }
}

// Sorting linked list by merge sorting
PQ_list_t *PQ_sort(PQ_list_t *list)
{
    PQ_list_t *list_l = list, *list_r = NULL;

    if ((list_l == NULL) || (list_l->next == NULL))
        return list_l;

    list_r = PQ_getMiddle(list_l);

    return PQ_mergeLists(PQ_sort(list_l), PQ_sort(list_r));
}

PQ_list_t *PQ_getMiddle(PQ_list_t *head)
{
    PQ_list_t *fast = head, *slow = head, *prev = NULL;

    while (fast != NULL && fast->next != NULL)
    {
        fast = fast->next->next;
        prev = slow;
        slow = slow->next;
    }

    if (prev != NULL)
    {
        prev->next = NULL;
    }
    return slow;
}

PQ_list_t *PQ_mergeLists(PQ_list_t *l, PQ_list_t *r)
{
    PQ_list_t dummy_head = {NULL, {INFINITY, INFINITY}, NULL};
    PQ_list_t *tail = &dummy_head;
    while ((l != NULL) && (r != NULL))
    {
        PQ_list_t **min;
        if (l->priority[0] == r->priority[0])
        {
            if (l->priority[1] < r->priority[1])
            {
                min = &l;
            }
            else
            {
                min = &r;
            }
        }
        else if (l->priority[0] < r->priority[0])
        {
            min = &l;
        }
        else
        {
            min = &r;
        }
        // PQ_list_t **min = (l->priority[0] < r->priority[0]) ? &l : &r;
        PQ_list_t *next = (*min)->next;
        tail = tail->next = *min;
        *min = next;
    }
    tail->next = l ? l : r;
    return dummy_head.next;
}

long PQ_SizeOfQueue(PQ_list_t *list)
{
    int size = 0;
    PQ_list_t *temp = list;
    while (temp != NULL)
    {
        size += sizeof(PQ_list_t) + sizeof(node_t);
        temp = temp->next;
    }
    return size;
}
