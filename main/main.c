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
//#include "ssd1366.h"
//#include "font8x8_basic.h"
#include "u8g2_esp32_hal.h"

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

void task_test_SSD1306i2c(void *ignore) {
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = SDA_PIN;
	u8g2_esp32_hal.scl  = SCL_PIN ;
	u8g2_esp32_hal.reset = RST_PIN;
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	u8g2_t u8g2; // a structure which will contain all the data for one display
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
	ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	u8g2_DrawFrame(&u8g2, 0,26,100,6);

	ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
	ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");

	vTaskDelete(NULL);
}

void wifiInit()
{
	
	 //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
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

// void i2c_master_init()
// {
// 	i2c_config_t i2c_config = {
// 		.mode = I2C_MODE_MASTER,
// 		.sda_io_num = SDA_PIN,
// 		.scl_io_num = SCL_PIN,
// 		.sda_pullup_en = GPIO_PULLUP_ENABLE,
// 		.scl_pullup_en = GPIO_PULLUP_ENABLE,
// 		.master.clk_speed = 1000000
// 	};
// 	i2c_param_config(I2C_NUM_0, &i2c_config);
// 	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
// }


// void task_ssd1306_display_text(const void *arg_text) {
// 	char *text = (char*)arg_text;
// 	uint8_t text_len = strlen(text);

// 	i2c_cmd_handle_t cmd;

// 	uint8_t cur_page = 0;

// 	cmd = i2c_cmd_link_create();
// 	i2c_master_start(cmd);
	//i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

// 	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
// 	i2c_master_write_byte(cmd, 0x00, true); // reset column
// 	i2c_master_write_byte(cmd, 0x10, true);
// 	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

// 	i2c_master_stop(cmd);
// 	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
// 	i2c_cmd_link_delete(cmd);

// 	for (uint8_t i = 0; i < text_len; i++) {
// 		if (text[i] == '\n') {
// 			cmd = i2c_cmd_link_create();
// 			i2c_master_start(cmd);
// 			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

// 			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
// 			i2c_master_write_byte(cmd, 0x00, true); // reset column
// 			i2c_master_write_byte(cmd, 0x10, true);
// 			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

// 			i2c_master_stop(cmd);
// 			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
// 			i2c_cmd_link_delete(cmd);
// 		} else {
// 			cmd = i2c_cmd_link_create();
// 			i2c_master_start(cmd);
// 			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

// 			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
// 			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

// 			i2c_master_stop(cmd);
// 			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
// 			i2c_cmd_link_delete(cmd);
// 		}
// 	}

// 	vTaskDelete(NULL);
// }

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
  //i2c_master_init();
  //ssd1306_init();

  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  connectionSemaphore = xSemaphoreCreateBinary();
  wifiInit();
  xTaskCreate(&OnConnected, "handel comms", 1024 * 3, NULL, 5, NULL);
  
  xTaskCreate(&blinky, "blink led", 2048, NULL, 2, NULL);
  
  xTaskCreate(&task_test_SSD1306i2c, "ssd1306_display_text", 1024 * 3,
		NULL, 6, NULL);

//   xTaskCreate(&task_ssd1306_display_text, "ssd1306_display_text",  2048,
// 		(void *)"Hello world!\nMulitine is OK!\nAnother line", 6, NULL);
}
