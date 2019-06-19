
#include "lpa.h"

static const char *TAG = "LPA";

node_t map[25 * 25];
static int x_current = 0, y_current = 0;
static int8_t snail_current = 1;

static int x_MAX, y_MAX;

static int lpa_move(list_t **path);
static void get_predecessors(node_t *map, list_t **pred_list, node_t *current_node);
static void get_successors(node_t *map, list_t **suc_list, node_t *current_node);
static node_t *get_min_pred(node_t *map, node_t *current_node);
static void update_node(node_t *map, PQ_list_t **queue, node_t *current_node, node_t *node, node_t *goal_node);
static void make_path(node_t *map, list_t **path, node_t *current_node, node_t *goal);

static void map_init(node_t *map, int x_MAX, int y_MAX);
static node_t *get_node_coord(node_t *map, int x, int y);
static float get_cost(node_t *from, node_t *to);
static void calc_key(float *key, node_t *current_node, node_t *goal_node);
// static void get_direction(int8_t x, int8_t y, uint8_t *direction, uint8_t *value);
static void get_direction(uint16_t goal_x, uint16_t goal_y, uint16_t current_x, uint16_t current_y, uint8_t *direction, uint8_t *value);

int lpa_init(/*node_t *map,*/ int _x_MAX, int _y_MAX)
{
    x_MAX = _x_MAX;
    y_MAX = _y_MAX;

    map_init(map, x_MAX, y_MAX);

    // current_node->isObstacle = false; //!!!!!!!!!!! ONLY FOR TEST!!!!!!!!!

    // goal_node->isObstacle = false; //!!!!!!!!!!! ONLY FOR TEST!!!!!!!!!

    // if (current_node->isObstacle /* || goal_node->isObstacle*/)
    //     return LPA_INIT_POINT_IS_OBSTACLE;

    return LPA_OK;
}

int lpa_compute_path(PQ_list_t *queue, list_t *path, int goalX, int goalY)
{
    node_t *current_node = get_node_coord(map, x_current, y_current);

    if (current_node == NULL)
        return LPA_INIT_POINT_ERROR;
    current_node->rhs = 0;

    node_t *goal_node = get_node_coord(map, goalX, goalY);
    if (goal_node == NULL)
        return -1;
    if (current_node->isObstacle || goal_node->isObstacle)
        return LPA_INIT_POINT_IS_OBSTACLE;

    float key_[2];
    calc_key(key_, current_node, goal_node);

    PQ_push(&queue, key_, current_node);

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
            get_successors(map, &successors, node);
            if (successors != NULL)
            {
                while (successors != NULL)
                {
                    update_node(map, &queue, current_node, successors->nodes, goal_node);
                    free(successors);
                    successors = successors->next;
                }
            }
        }
        else
        {
            node->g = INFINITY;
            list_t *successors = NULL;
            get_successors(map, &successors, node);
            if (successors != NULL)
            {
                while (successors != NULL)
                {
                    update_node(map, &queue, current_node, successors->nodes, goal_node);
                    free(successors);
                    successors = successors->next;
                }
            }
        }
        top_key = PQ_getTopKey(queue);
        if (top_key == NULL)
        {
            return LPA_PATH_CANT_BE_FOUND;
        }

        calc_key(goal_key, goal_node, goal_node);
    }
    make_path(map, &path, current_node, goal_node);
    print_map(map, current_node, goal_node);
    lpa_free(queue);
    if (lpa_move(&path) != LPA_OK)
        return LPA_MOVING_ERROR;

    return LPA_OK;
}

static int lpa_move(list_t **path)
{
    if (*path == NULL)
        return LPA_PATH_ERROR;

    
    uint8_t sm_ret_code = 0;
    while (*path != NULL)
    {
        uint8_t buffer[I2C_STM32_PACKET_LENGTH] = {0};
        buffer[0] = COMM_INIT;
        buffer[1] = COMM_MOVE;

        get_direction((*path)->nodes->x, (*path)->nodes->y, x_current, y_current, &buffer[2], &buffer[4]);
        ESP_LOGI(TAG, "buffer 1:%d 2:%d", buffer[2], buffer[4]);
        if (buffer[2] == MOVE_LEFT || buffer[2] == MOVE_RIGHT || buffer[2] == MOVE_REVERSE)
        {
            sm_ret_code = SM_send_command(buffer);
            if (sm_ret_code != SM_OK)
                ESP_LOGE(TAG, "Connection with slave MCU with error: %d", sm_ret_code);
            // if (SM_send_command(buffer) != SM_OK)
            // ESP_LOGE(TAG, "Connection with slave MCU can't be reached");
            // vTaskDelay(4000 / portTICK_RATE_MS);
            get_direction((*path)->nodes->x, (*path)->nodes->y, x_current, y_current, &buffer[2], &buffer[4]);
            ESP_LOGI(TAG, "buffer 1:%d 2:%d", buffer[2], buffer[4]);
        }
        sm_ret_code = SM_send_command(buffer);
        if (sm_ret_code != SM_OK)
            ESP_LOGE(TAG, "Connection with slave MCU with error: %d", sm_ret_code);
        // SM_send_command(buffer);
        // if (SM_send_command(buffer) != SM_OK)
        //     ESP_LOGE(TAG, "Connection with slave MCU can't be reached");
        x_current = (*path)->nodes->x;
        y_current = (*path)->nodes->y;
        ESP_LOGI(TAG, "temp x:%d y:%d", x_current, y_current);
        free(*path);
        *path = (*path)->next;
        // vTaskDelay(4000 / portTICK_RATE_MS);
    }
    return LPA_OK;
}

void lpa_get_current_coords(char *data)
{
    sprintf(data, "%d:%d", x_current, y_current);
}

void lpa_free(PQ_list_t *queue /*, list_t *path*/)
{
    PQ_free(&queue);
    // list_free(&path);
}

static void update_node(node_t *map, PQ_list_t **queue, node_t *current_node, node_t *node, node_t *goal_node)
{
    if (node != current_node)
    {
        node->rhs = INFINITY;
        list_t *predecessors = NULL;
        get_predecessors(map, &predecessors, node);
        if (predecessors != NULL || predecessors->nodes != NULL)
        {
            while (predecessors != NULL)
            {
                float min = fmin(node->rhs, predecessors->nodes->g + get_cost(predecessors->nodes, node));
                node->rhs = min;
                free(predecessors);
                predecessors = predecessors->next;
            }
        }
    }
    if (PQ_contains(*queue, node))
    {
        PQ_remove(queue, node);
    }
    if (node->g != node->rhs)
    {
        float key[2];
        calc_key(key, node, goal_node);
        PQ_push(queue, key, node);
    }
}

static node_t *get_node_coord(node_t *map, int x, int y)
{
    for (int i = 0; i < x_MAX * y_MAX; i++)
        if (map[i].x == x && map[i].y == y)
            return &map[i];
    return NULL;
}

//It is equivalent of get_successors
static void get_predecessors(node_t *map, list_t **pred_list, node_t *from_node)
{
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x != 0 || y != 0)
            {
                node_t *node = get_node_coord(map, from_node->x + x, from_node->y + y);
                if (node != NULL && !node->isObstacle)
                {
                    list_add(pred_list, node);
                }
            }
        }
    }
}

static void get_successors(node_t *map, list_t **suc_list, node_t *from_node)
{
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x != 0 || y != 0)
            {
                node_t *node = get_node_coord(map, from_node->x + x, from_node->y + y);
                if (node != NULL && !node->isObstacle)
                {
                    list_add(suc_list, node);
                }
            }
        }
    }
}

//get min node by rhs
static node_t *get_min_pred(node_t *map, node_t *from_node)
{
    node_t *min_node = from_node;
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            node_t *temp = get_node_coord(map, from_node->x + x, from_node->y + y);
            if (temp != NULL && !temp->isObstacle && min_node->rhs > temp->rhs)
            {
                min_node = temp;
            }
        }
    }
    return min_node;
}

static void make_path(node_t *map, list_t **path, node_t *current_node, node_t *goal)
{
    list_add(path, goal);
    node_t *node = get_min_pred(map, goal);

    node_t *prev_node = goal;
    while (node != current_node)
    {
        list_add(path, node);
        node->isPath = true;
        prev_node = node;
        node = get_min_pred(map, prev_node);
    }
}

static void get_direction(uint16_t goal_x, uint16_t goal_y, uint16_t current_x, uint16_t current_y, uint8_t *direction, uint8_t *value)
{
    int8_t x = goal_x - current_x,
           y = goal_y - current_y;
    uint8_t snail_next = 0;
    if (x == 1 && y == 1)
        snail_next = LPA_DIRECTION_SNAIL_8;

    else if (x == 0 && y == 1)
        snail_next = LPA_DIRECTION_SNAIL_7;

    else if (x == 1 && y == -1)
        snail_next = LPA_DIRECTION_SNAIL_6;

    else if (x == -1 && y == 0)
        snail_next = LPA_DIRECTION_SNAIL_5;

    else if (x == -1 && y == -1)
        snail_next = LPA_DIRECTION_SNAIL_4;

    else if (x == 0 && y == -1)
        snail_next = LPA_DIRECTION_SNAIL_3;

    else if (x == -1 && y == 1)
        snail_next = LPA_DIRECTION_SNAIL_2;

    else if (x == 1 && y == 0)
        snail_next = LPA_DIRECTION_SNAIL_1;

    ESP_LOGI(TAG, "snail_next: %d", snail_next);
    ESP_LOGI(TAG, "snail_current: %d", snail_current);
    int8_t temp = snail_next - snail_current;
    switch (temp)
    {
    case 0:
        *direction = MOVE_FORWARD;
        *value = 50;
        break;
    case 4:
    case -4:
        *direction = MOVE_RIGHT;
        *value = 180;
        break;
    case 7:
    case 1:
        *direction = MOVE_RIGHT;
        *value = 45;
        break;
    case -7:
    case -1:
        *direction = MOVE_LEFT;
        *value = 45;
        break;
    case -6:
    case 2:
        *direction = MOVE_LEFT;
        *value = 90;
        break;
    case 6:
    case -2:
        *direction = MOVE_RIGHT;
        *value = 90;
        break;
    case 5:
    case 3:
        *direction = MOVE_RIGHT;
        *value = 135;
        break;
    case -5:
    case -3:
        *direction = MOVE_LEFT;
        *value = 135;
        break;
    }

    snail_current = snail_next;
}

//UTILS
static void map_init(node_t *map, int x_MAX, int y_MAX)
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

static float get_cost(node_t *from, node_t *to)
{
    if (from->x == to->x || from->y == to->y)
        return 1;
    else
        return sqrt(2);
}

static void calc_key(float *key, node_t *from_node, node_t *goal_node)
{
    key[0] = fmin(from_node->g, from_node->rhs) + Node_getHeuristic(from_node, goal_node);
    key[1] = fmin(from_node->rhs, from_node->g);
}

void print_map(node_t *map, node_t *current_node, node_t *goal_node)
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
        else if (map[i].x == current_node->x && map[i].y == current_node->y)
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

//make obstacles like a island
// void make_obstacles(void)
// {
//     for (int i = 0; i < x_MAX * y_MAX; i += rand() % 30)
//     {
//         if (map[i].isObstacle)
//         {
//             for (int x = -2; x <= 2; x++)
//             {
//                 for (int y = -2; y <= 2; y++)
//                 {
//                     if (x != 0 || y != 0)
//                     {
//                         node_t *node = get_node_coord(map[i].x + x, map[i].y + y);
//                         if (node != NULL)
//                             node->isObstacle = true;
//                     }
//                 }
//             }
//         }
//     }
// }