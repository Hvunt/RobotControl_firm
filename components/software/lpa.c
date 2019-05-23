
#include "lpa.h"

node_t *current_node/*, *goal_node*/;

PQ_list_t *queue;
node_t *map;
list_t *path;

static int x_MAX, y_MAX;

int lpa_init(int _x_MAX, int _y_MAX, int x_START, int y_START/*, int x_GOAL, int y_GOAL*/)
{
    x_MAX = _x_MAX;
    y_MAX = _y_MAX;

    // сделать проверку на максимально возможный размер карты
    map = (node_t *)calloc(x_MAX * y_MAX, sizeof(node_t)); 

    map_init(map, x_MAX, y_MAX);
    printf("###########\n");

    start_node = get_node_coord(x_START, y_START);
    if (start_node == NULL)
        return LPA_INIT_POINT_ERROR;
    start_node->rhs = 0;
    // start_node->isObstacle = false; //!!!!!!!!!!! ONLY FOR TEST!!!!!!!!!

    // goal_node = get_node_coord(x_GOAL, y_GOAL);
    // if (goal_node == NULL)
    //     return LPA_INIT_POINT_ERROR;
    // goal_node->isObstacle = false; //!!!!!!!!!!! ONLY FOR TEST!!!!!!!!!

    if (start_node->isObstacle/* || goal_node->isObstacle*/)
        return LPA_INIT_POINT_IS_OBSTACLE;

    // float key_[2];
    // calc_key(key_, start_node, goal_node);
    // PQ_push(&queue, key_, start_node);
    return LPA_INIT_OK;
}

int lpa_compute_path(node_t *goal_node)
{
    // ftime(&start_time);
    float key_[2];
    calc_key(key_, start_node, goal_node);
    PQ_push(&queue, key_, start_node);
    float *top_key = PQ_getTopKey(queue), goal_key[2];
    calc_key(goal_key, goal_node, goal_node);

    while ((top_key[0] < goal_key[0] && top_key[1] < goal_key[1]) || (goal_node->rhs != goal_node->g))
    {
        node_t *node = PQ_pop(&queue);
        node->isVisited = true;

        if (node->g > node->rhs)
        {
            node->g = node->rhs;
            list_t *successors = NULL;
            get_successors(&successors, node);
            if (successors != NULL)
            {
                while (successors != NULL)
                {
                    update_node(successors->nodes);
                    successors = successors->next;
                }
            }
            free(successors);
        }
        else
        {
            node->g = INFINITY;
            // update_node(node);
            list_t *successors = NULL;
            get_successors(&successors, node);
            if (successors != NULL)
            {
                while (successors != NULL)
                {
                    update_node(successors->nodes);
                    successors = successors->next;
                }
            }
            free(successors);
        }

        top_key = PQ_getTopKey(queue);
        if (top_key == NULL)
        {
            return -1;
        }

        calc_key(goal_key, goal_node, goal_node);
    }
    // ftime(&end_time);
    make_path();
    print_map(goal_node);
    return 1;
}

void update_node(node_t *node)
{
    if (node != start_node)
    {
        node->rhs = INFINITY;
        list_t *predecessors = NULL;
        get_predecessors(&predecessors, node);
        if (predecessors != NULL || predecessors->nodes != NULL)
        {
            while (predecessors != NULL)
            {
                float min = fmin(node->rhs, predecessors->nodes->g + get_cost(predecessors->nodes, node));
                node->rhs = min;
                predecessors = predecessors->next;
            }
        }
        free(predecessors);
    }
    if (PQ_contains(queue, node))
    {
        PQ_remove(&queue, node);
    }
    if (node->g != node->rhs)
    {
        float key[2];
        calc_key(key, node, goal_node);
        PQ_push(&queue, key, node);
    }
}

node_t *get_node_coord(int x, int y)
{
    for (int i = 0; i <= x_MAX * y_MAX; i++)
        if (map[i].x == x && map[i].y == y)
            return &map[i];
    return NULL;
}

//It is equivalent of get_successors
void get_predecessors(list_t **pred_list, node_t *from_node)
{
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x != 0 || y != 0)
            {
                node_t *node = get_node_coord(from_node->x + x, from_node->y + y);
                if (node != NULL && !node->isObstacle)
                {
                    list_add(pred_list, node);
                }
            }
        }
    }
}

void get_successors(list_t **suc_list, node_t *from_node)
{
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x != 0 || y != 0)
            {
                node_t *node = get_node_coord(from_node->x + x, from_node->y + y);
                if (node != NULL && !node->isObstacle)
                {
                    list_add(suc_list, node);
                }
            }
        }
    }
}

//get min node by rhs
node_t *get_min_pred(node_t *from_node)
{
    node_t *min_node = from_node;
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            node_t *temp = get_node_coord(from_node->x + x, from_node->y + y);
            if (temp != NULL && !temp->isObstacle && min_node->rhs > temp->rhs)
            {
                min_node = temp;
            }
        }
    }
    return min_node;
}

void make_path(void)
{
    node_t *node = get_min_pred(goal_node);
    node_t *prev_node = goal_node;
    while (node != start_node)
    {
        list_add(&path, node);
        node->isPath = true;
        prev_node = node;
        node = get_min_pred(prev_node);
    }
}

//UTILS
void map_init(node_t *map_, int x_MAX, int y_MAX)
{
    int i = 0;
    for (int x = 0; x < x_MAX; x++)
    {
        for (int y = 0; y < y_MAX; y++)
        {
            node_t node;
            node.rhs = INFINITY;
            node.g = INFINITY;
            node.x = x;
            node.y = y;
            // int random = rand() % 100; // isObstacle calculating. Only during development
            // node.isObstacle = (random > 10) ? false : true;
            node.isObstacle = false;
            node.isVisited = false;
            node.isPath = false;
            *(map + i) = node;
            i++;
        }
    }

    //only for debug.
    // make_obstacles();
}

float get_cost(node_t *from, node_t *to)
{
    if (from->x == to->x || from->y == to->y)
        return 1;
    else
        return sqrt(2);
}

void calc_key(float *key, node_t *from_node, node_t *goal_node)
{
    key[0] = fmin(from_node->g, from_node->rhs) + Node_getHeuristic(from_node, goal_node);
    key[1] = fmin(from_node->rhs, from_node->g);
}

void print_map(node_t *node)
{
    printf("==========================>Y\n");
    printf("||");
    for (int i = 0; i < x_MAX * y_MAX; i++)
    {
        if (map[i].isObstacle)
            printf("#");
        // else if (map[i].x == node->x && map[i].y == node->y)
        //     printf("O");
        else if (map[i].x == goal_node->x && map[i].y == goal_node->y)
            printf("X");
        else if (map[i].x == start_node->x && map[i].y == start_node->y)
            printf("S");
        else if (map[i].isPath)
            printf("O");
        else
            printf(" ");

        if (map[i].y == y_MAX - 1)
            printf("\n||");
    }
    printf("\n\\/\n");
    printf("X\n");
    printf("###########\n");
}

// void write_results()
// {
//     time_t t = time(NULL);
//     struct tm current_time = *localtime(&t);
//     char file_name[30] = {};
//     strftime(file_name, 31, "result_%Y-%m-%d_%H-%M-%S.txt", &current_time);

//     FILE *f = fopen(file_name, "wt");
//     if (f == NULL)
//     {
//         printf("Error opening file\n");
//         getchar();
//         exit(1);
//     }
//     fprintf(f, "size of the map: %dx%d,\n start coordinates X: %d, Y: %d\ngoal coordinates X:%d, Y:%d\n", x_MAX, y_MAX, start_node->x, start_node->y, goal_node->x, goal_node->y);
//     elapsed_time = (int)(1000 * (end_time.time - start_time.time) + (end_time.millitm - start_time.millitm));
//     elapsed_time /= 1000;
//     fprintf(f, "elapsed_time: %f sec.", elapsed_time);
//     fprintf(f, "==========================>Y\n");
//     fprintf(f, "||");
//     for (int i = 0; i < x_MAX * y_MAX; i++)
//     {
//         if (map[i].isObstacle)
//             fprintf(f, "#");
//         else if (map[i].x == goal_node->x && map[i].y == goal_node->y)
//             fprintf(f, "X");
//         else if (map[i].x == start_node->x && map[i].y == start_node->y)
//             fprintf(f, "S");
//         else if (map[i].isPath)
//             fprintf(f, "O");
//         else if (map[i].isVisited)
//             fprintf(f, "-");
//         else
//             fprintf(f, " ");

//         if (map[i].y == y_MAX - 1)
//             fprintf(f, "\n||");
//     }
//     fprintf(f, "\n\\/\n");
//     fprintf(f, "X\n");
//     fprintf(f, "###########\n");

//     fclose(f);
// }

//make obstacles like a island
void make_obstacles(void)
{
    for (int i = 0; i < x_MAX * y_MAX; i += rand() % 30)
    {
        if (map[i].isObstacle)
        {
            for (int x = -2; x <= 2; x++)
            {
                for (int y = -2; y <= 2; y++)
                {
                    if (x != 0 || y != 0)
                    {
                        node_t *node = get_node_coord(map[i].x + x, map[i].y + y);
                        if (node != NULL)
                            node->isObstacle = true;
                    }
                }
            }
        }
    }
}