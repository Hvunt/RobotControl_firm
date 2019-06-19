
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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 2000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave(uint8_t address, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 2000 / portTICK_RATE_MS);
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

// void SM_sending_task(void *params)
uint8_t SM_send_command(uint8_t *data)
{
    esp_err_t ret;
    // ret = i2c_master_write_slave(28, data, I2C_STM32_PACKET_LENGTH);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE("SM_sending_task", "%s", esp_err_to_name(ret));
    //     return SM_CONNECTION_ERROR;
    // }

    uint8_t slave_response = SM_ERROR;
    uint8_t slave_command[I2C_STM32_PACKET_LENGTH] = {0};
    slave_command[0] = COMM_INIT;
    slave_command[1] = COMM_GET_STATUS;
    uint16_t error_counter = 0;
    while (error_counter < 500)
    {
        i2c_master_write_slave(28, slave_command, I2C_STM32_PACKET_LENGTH);
        ret = i2c_master_read_slave(28, &slave_response, 1);
        // ESP_LOGE("SM_sending_task", "%d", slave_response);
        if (ret != ESP_OK)
        {
            ESP_LOGE("SM_sending_task", "%s", esp_err_to_name(ret));
            return SM_CONNECTION_ERROR;
        }
        if (slave_response == SM_OK)
            return slave_response;
        error_counter++;
        vTaskDelay(5 / portTICK_RATE_MS);
    }

    // for (uint16_t i = 0; i < 500; i++)
    // {
    //     ret = i2c_master_read_slave(28, &slave_response, 1);
    //     ESP_LOGE("SM_sending_task", "%d", slave_response);
    //     if (ret == ESP_OK)
    //         return slave_response;
    //     vTaskDelay(5 / portTICK_RATE_MS);
    // }
    return SM_ERROR;
}