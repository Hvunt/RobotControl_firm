
// #include <stdint.h>

#define JSON_TOKEN_NAME_RESPONSE "response"

#define JSON_TOKEN_NAME_ACTION "action"
#define JSON_TOKEN_NAME_ACTION_set_settings "set_settings"
#define JSON_TOKEN_NAME_ACTION_get_settings "get_settings"
#define JSON_TOKEN_NAME_ACTION_sensors "sensors_data"
#define JSON_TOKEN_NAME_ACTION_move "move"

#define JSON_TOKEN_NAME_DATA "data"

#define JSON_TOKEN_NAME_robot_type "robot_type"
#define JSON_TOKEN_NAME_wheels_type "wheels_type"
#define JSON_TOKEN_NAME_motors_ports "motors_ports"
#define JSON_TOKEN_NAME_encoder_type "encoder_type"
#define JSON_TOKEN_NAME_servos_ports "servos_ports"
#define JSON_TOKEN_NAME_coordinates "coordinates"
#define JSON_TOKEN_NAME_algorithm "algorithm"

struct json_defs
{
    char action[15];
    uint8_t robot_type;
    uint8_t wheels_type;
    uint8_t motors_ports;
    uint8_t encoder_type;
    uint8_t servos_ports;
    char coordinates[15];
    char algorithm[15];
};
