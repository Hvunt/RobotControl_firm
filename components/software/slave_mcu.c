
#include "slave_mcu.h"

static slave_mcu_t slave_mcu_settings;

static esp_err_t i2c_master_write_slave(uint8_t address,uint8_t *data_wr, size_t size);

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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
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
void SM_send_command(uint8_t *data)
{
    esp_err_t ret = i2c_master_write_slave(28, data, sizeof(data));
    if (ret != ESP_OK)
        ESP_LOGE("SM_sending_task", "%s", esp_err_to_name(ret));

    // vTaskDelete(NULL);
    // uint8_t i = 0;
    // while (1)
    // {
    //     uint8_t buffer[10];
    //     buffer[0] = COMM_SET_SERVOS_POS;
    //     buffer[1] = 1;
    //     buffer[2] = i;
    //     esp_err_t ret = i2c_master_write_test(I2C_MASTER_NUM, buffer, sizeof(buffer));
    //     if (ret != ESP_OK)
    //         ESP_LOGE("SM_sending_task", "%s", esp_err_to_name(ret));
    //     // ESP_ERROR_CHECK(ret);
    //     i++;
    //     if (i == 180)
    //         i = 0;
    //     vTaskDelay(20);
    // }
}