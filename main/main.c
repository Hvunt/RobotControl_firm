/* Esptouch example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "main.h"

#define PORT 80

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAG = "RC";

const int IPV4_GOTIP_BIT = BIT3;
const int IPV6_GOTIP_BIT = BIT4;

wifi_config_t *wifi_config;

void smartconfig_task(void *parm);
void tcp_server_task(void *pvParameters);
void find_path_task(void *params);

void i2c_master_init(void);
// void i2c_send(char *data);
// static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void sc_callback(smartconfig_status_t status, void *pdata)
{
    switch (status)
    {
    case SC_STATUS_WAIT:
        ESP_LOGI(TAG, "SC_STATUS_WAIT");
        break;
    case SC_STATUS_FIND_CHANNEL:
        ESP_LOGI(TAG, "SC_STATUS_FINDING_CHANNEL");
        break;
    case SC_STATUS_GETTING_SSID_PSWD:
        ESP_LOGI(TAG, "SC_STATUS_GETTING_SSID_PSWD");
        break;
    case SC_STATUS_LINK:
        ESP_LOGI(TAG, "SC_STATUS_LINK");
        wifi_config = pdata;
        ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", wifi_config->sta.password);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SC_STATUS_LINK_OVER:
        ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
        if (pdata != NULL)
        {
            uint8_t phone_ip[4] = {0};
            memcpy(phone_ip, (uint8_t *)pdata, 4);
            ESP_LOGI(TAG, "Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
        }
        xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
        break;
    default:
        break;
    }
}

void smartconfig_task(void *parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    ESP_ERROR_CHECK(esp_smartconfig_start(sc_callback));
    while (1)
    {
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 3, NULL);
            SL_setState(SL_WAIT_FOR_CONNECTION_TO_DEVICE);
            vTaskDelete(NULL);
        }
    }
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1)
    {
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket binded");

        err = listen(listen_sock, 1);
        if (err != 0)
        {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");
        SL_setState(SL_NORMAL_MODE);
        while (1)
        {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0)
            {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else
            {
                // Get the sender's ip address as string
                if (sourceAddr.sin6_family == PF_INET)
                {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }
                else if (sourceAddr.sin6_family == PF_INET6)
                {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                // i2c_send(rx_buffer);
                int err = send(sock, rx_buffer, len, 0);
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            vTaskDelay(100);
            close(listen_sock);
            vTaskDelay(100);
            close(sock);
            vTaskDelay(100);
            SL_setState(SL_WAIT_FOR_CONNECTION_TO_DEVICE);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief i2c sender function
 */
// static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( I2C_STM32_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

/**
 * @brief i2c master initialization
 */
void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

// static void i2c_send(char *data)
// {
//     int ret;
//     uint8_t data_buffer[2];
//     if (strcmp(data,MOTORS_FORWARD) == 0){
//         data_buffer[0] = COMM_MOVE;
//         data_buffer[1] = MOVE_FORWARD;
//     } else if (strcmp(data,MOTORS_REVERSE) == 0)
//     {
//         data_buffer[0] = COMM_MOVE;
//         data_buffer[1] = MOVE_REVERSE;
//     } else if (strcmp(data,MOTORS_LEFT) == 0)
//     {
//         data_buffer[0] = COMM_MOVE;
//         data_buffer[1] = MOVE_LEFT;
//     } else if (strcmp(data,MOTORS_RIGHT) == 0)
//     {
//         data_buffer[0] = COMM_MOVE;
//         data_buffer[1] = MOVE_RIGHT;
//     } else if (strcmp(data,MOTORS_STOP) == 0)
//     {
//         data_buffer[0] = COMM_MOVE;
//         data_buffer[1] = MOVE_STOP;
//     }

//     ret = i2c_master_write_slave(I2C_MASTER_NUM, data_buffer, sizeof(data_buffer));
//     if (ret == ESP_ERR_TIMEOUT)
//         printf("i2c timeout\n");
// }

void show_angles_task(void *params)
{
    vTaskDelay(2000);
    while (1)
    {
        ESP_LOGI(TAG, "Yaw %f", EC_getYaw());
        ESP_LOGI(TAG, "Pitch %f", EC_getPitch());
        ESP_LOGI(TAG, "Roll %f", EC_getRoll());
        vTaskDelay(100);
    }
}

void find_path_task(void *params)
{
    ESP_LOGI(TAG, "path founded? %d", lpa_compute_path());
    vTaskDelete(NULL);
}

void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    SL_init();
    SL_setState(SL_INIT);
    xTaskCreate(SL_task, "SL_task", 2048, NULL, 3, NULL);

    initialise_wifi();
    i2c_master_init();
    ESP_LOGI(TAG, "lpa init code: %d", lpa_init(50, 50, 0, 0, 40, 20));

<<<<<<< HEAD
    xTaskCreate(EC_ecTask, "EC_ecTask", 8192, NULL, 7, NULL);
=======
    xTaskCreate(find_path_task,"find_path", 51200, NULL, 6, NULL);
    // xTaskCreate(EC_ecTask, "EC_ecTask", 8192, NULL, 6, NULL);
>>>>>>> 318fe891ed4a944eb6aacec9d19e5333afcfcc39
    // xTaskCreate(show_angles_task, "show_angles_task", 2048, NULL, 6, NULL);
    wait_for_ip();
}
