
#include "slave_mcu.h"

static slave_mcu_t slave_mcu_settings;

static esp_err_t i2c_master_write_slave(uint8_t address, uint8_t *data_wr, size_t size);
static esp_err_t i2c_master_read_slave(uint8_t address, uint8_t *data_rd, size_t size);

/**
 * @brief i2c sender function
 */
static esp_err_t i2c_master_write_slave(uint8_t address, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave(uint8_t address, uint8_t *data_rd, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, size, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void SM_init()
{
    //default initialization
    slave_mcu_settings.robot_type = 0;
    slave_mcu_settings.encoder_type = 3;
    slave_mcu_settings.dc_ports = 0x30 | (TERMINAL_ONE | TERMINAL_TWO | TERMINAL_THREE | TERMINAL_FOUR);
}

uint8_t SM_get_status(void)
{
    esp_err_t ret;
    uint8_t slave_response[I2C_STM32_PACKET_LENGTH] = {0};
    slave_response[0] = SM_ERROR;

    uint8_t slave_command[I2C_STM32_PACKET_LENGTH] = {0};
    slave_command[0] = COMM_INIT;
    slave_command[1] = COMM_GET_STATUS;

    ret = i2c_master_write_slave(28, slave_command, I2C_STM32_PACKET_LENGTH);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SM_sending_task", "w %s", esp_err_to_name(ret));
        slave_response[0] = SM_CONNECTION_ERROR;
    }
    vTaskDelay(2 / portTICK_RATE_MS);
    ret = i2c_master_read_slave(28, slave_response, I2C_STM32_PACKET_LENGTH);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SM_sending_task", "r %s", esp_err_to_name(ret));
        slave_response[0] = SM_CONNECTION_ERROR;
    }

    return slave_response[0];
}

uint8_t SM_send_command(uint8_t *data)
{
    esp_err_t ret;
    ret = i2c_master_write_slave(28, data, I2C_STM32_PACKET_LENGTH);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SM_sending_task", "%s", esp_err_to_name(ret));
        return SM_CONNECTION_ERROR;
    }
    
    uint8_t error_counter = 0, status = SM_get_status();
    while (error_counter < 5 || status != SM_OK)
    {
        status = SM_get_status();
        error_counter++;
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    
    return status;
}