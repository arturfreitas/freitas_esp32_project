#include <stdio.h>
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include "driver/i2c.h"
#include "ssd1366.h"
#include "font8x8_basic.h"

#define SDA_PIN GPIO_NUM_4
#define SCL_PIN GPIO_NUM_15
#define RST_PIN GPIO_NUM_16

#define LED_BUILTIN 25

#define tag "SSD1306"

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

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	gpio_set_direction(RST_PIN,GPIO_MODE_OUTPUT);
	gpio_set_level(RST_PIN, 0);
	vTaskDelay( pdMS_TO_TICKS( 100 ) );
	gpio_set_level(RST_PIN, 1);
	

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "OLED configured successfully");
	} else {
		ESP_LOGE(tag, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_text(const void *arg_text) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = 0;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
	}

	vTaskDelete(NULL);
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
	  ESP_LOGE(TAG, "teste");
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
	gpio_pad_select_gpio(LED_BUILTIN);
	gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
	int isOn = 0;
	while (true)
	{
		isOn = !isOn;
		gpio_set_level(LED_BUILTIN, isOn);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}



void app_main(void)
{ 
  i2c_master_init();
  ssd1306_init();

  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  connectionSemaphore = xSemaphoreCreateBinary();
  wifiInit();
  xTaskCreate(&OnConnected, "handel comms", 1024 * 3, NULL, 5, NULL);
  
  xTaskCreate(&blinky, "blink led", 2048, NULL, 2, NULL);

  xTaskCreate(&task_ssd1306_display_text, "ssd1306_display_text",  2048,
		(void *)"Hello world!\nMulitine is OK!\nAnother line", 6, NULL);
}
