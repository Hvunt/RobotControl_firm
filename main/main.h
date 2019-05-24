

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "esp_heap_trace.h"

#include "jsmn.h"
#include "jWrite.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/i2c.h"

#include "status_led.h"
#include "e_compass.h"
#include "lpa.h"
#include "slave_mcu.h"
#include "json_defs.h"

#include "configs.h"

void smartconfig_task(void *parm);
void tcp_server_task(void *pvParameters);
void find_path_task(void *params);

void i2c_master_init(void);
// void i2c_send(char *data);
void show_angles_task(void *params);

#ifdef __cplusplus
}
#endif