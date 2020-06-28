#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define PIN GPIO_NUM_25

char *TAG = "CONNECTION";

xSemaphoreHandle connectionSemaphore;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			ESP_LOGI(TAG,"connecting...\n");
			break;

		case SYSTEM_EVENT_STA_CONNECTED:
			ESP_LOGI(TAG, "connected\n");
			break;
		
		case SYSTEM_EVENT_STA_GOT_IP:
    		ESP_LOGI(TAG,"got ip\n");
    		xSemaphoreGive(connectionSemaphore);
    		break;
		
		case SYSTEM_EVENT_STA_DISCONNECTED:
    		ESP_LOGI(TAG,"disconnected\n");
    		break;

 		default:
		  break;
	}

    return ESP_OK; 
}

void wifiInit()
{
	ESP_ERROR_CHECK(nvs_flash_init());
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	wifi_config_t wifi_config = 
		{
			.sta = {
              .ssid = CONFIG_WIFI_SSID,
              .password = CONFIG_WIFI_PASSWORD
              }
		};
    
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
	ESP_ERROR_CHECK(esp_wifi_start());
}

void OnConnected(void *para)
{
  while (true)
  {
    if (xSemaphoreTake(connectionSemaphore, 10000 / portTICK_RATE_MS))
    {
      ESP_LOGI(TAG, "connected");
      
      
      xSemaphoreTake(connectionSemaphore, portMAX_DELAY);
    }
    else
    {
      ESP_LOGE(TAG, "Failed to connect. Retry in");
      for (int i = 0; i < 5; i++)
      {
        ESP_LOGE(TAG, "...%d", i);
        vTaskDelay(1000 / portTICK_RATE_MS);
      }
      esp_restart();
    }
  }
}



// see https://www.learnesp32.com/3_blinkey for details
void blinky(void *params)
{
	gpio_pad_select_gpio(PIN);
	gpio_set_direction(PIN, GPIO_MODE_OUTPUT);
	int isOn = 0;
	while (true)
	{
		isOn = !isOn;
		gpio_set_level(PIN, isOn);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
void app_main(void)
{
  printf("Hello world!\n");
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  connectionSemaphore = xSemaphoreCreateBinary();
  wifiInit();
  xTaskCreate(&OnConnected, "handel comms", 1024 * 3, NULL, 5, NULL);
  
  xTaskCreate(&blinky, "blink led", 2048, NULL, 2, NULL);
}
