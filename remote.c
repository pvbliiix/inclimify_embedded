#include <espressif/esp_common.h>
#include <esp8266.h>
#include <esp/uart.h>
#include <string.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

// MQTT data

#define MQTT_HOST ("192.168.20.15")
#define MQTT_PORT 1883

#define MQTT_USER "appti2019"
#define MQTT_PASS "appti2019"

// times below are written in milliseconds (ms)

#define SLOW_LED_TIME 500
#define FAST_LED_TIME 50

#define FAST_LED_TICKS 10
#define SLOW_LED_TICKS 3

#define TOPIC_LEN 32

#define BUF_SIZE 100

#define RETRY_INTERVAL 1000
#define YIELD_DURATION 5000
#define COMMAND_TIMEOUT 10000

#define MAX_RETRIES 5

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

// LED pin

uint8_t static const led_gpio = D4;

inline void swap(char *x, char *y) {
	char t = *x; *x = *y; *y = t;
}

char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]);

	return buffer;
}

char* numtostr(int value, char* buffer)
{

	int n = abs(value);

	int i = 0;
	while (n)
	{
		int r = n % 10;

		if (r >= 10)
			buffer[i++] = 65 + (r - 10);
		else
			buffer[i++] = 48 + r;

		n = n / 10;
	}

	if (i == 0)
		buffer[i++] = '0';

	if (value < 0)
		buffer[i++] = '-';

	buffer[i] = '\0';

	return reverse(buffer, 0, i - 1);
}
void topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    char cmd[(int)message->payloadlen];

    memcpy(cmd, ((char *)(message->payload)), (int)message->payloadlen);

    cmd[(int)message->payloadlen] = '\0';

    printf("COMMAND: %s\r\n", cmd);

    if(!strcmp(cmd, "LED FAST"))
    {
      printf("FAST LED BLINK\r\n");
      for(i = 0; i < FAST_LED_TICKS; i++)
      {
        gpio_toggle(led_gpio);
        vTaskDelay(FAST_LED_TIME / portTICK_PERIOD_MS);
        gpio_toggle(led_gpio);
        vTaskDelay(FAST_LED_TIME / portTICK_PERIOD_MS);
      }
    }
    else if(!strcmp(cmd, "LED SLOW"))
    {
      printf("SLOW LED BLINK\r\n");
      for(i = 0; i < SLOW_LED_TICKS; i++)
      {
        gpio_toggle(led_gpio);
        vTaskDelay(SLOW_LED_TIME / portTICK_PERIOD_MS);
        gpio_toggle(led_gpio);
        vTaskDelay(SLOW_LED_TIME / portTICK_PERIOD_MS);
      }
    }
    else if(!strcmp(cmd, "POWER ON"))
    {
      printf("LED POWER ON\r\n");
      gpio_write(led_gpio, false);
    }
    else if(!strcmp(cmd, "POWER OFF"))
    {
      printf("LED POWER OFF\r\n");
      gpio_write(led_gpio, true);
    }
    else
      printf("INVALID COMMAND: \"%s\"\r\n", cmd);

}

void remoteControl(void* pvParameters)
{
  int ret = 0;
  int retries = 0;
  struct mqtt_network network;
  mqtt_client_t client = mqtt_client_default;
  uint8_t mqtt_buf[BUF_SIZE];
  uint8_t mqtt_readbuf[BUF_SIZE];
  char topic[TOPIC_LEN];
  char device_id[8];
  mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

  mqtt_network_new( &network );

  while(1)
  {
      printf("%s: started\n\r", __func__);
      printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
             MQTT_HOST);
      while( (ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT)) ){
          printf("cant connect to network error: %d\n\r", ret);
          retries++;
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
          if(retries >= MAX_RETRIES)
            break;
      }

      mqtt_client_new(&client, &network, COMMAND_TIMEOUT, mqtt_buf, BUF_SIZE,
                    mqtt_readbuf, BUF_SIZE);

      data.willFlag       = 0;
      data.MQTTVersion    = 3;
      data.clientID.cstring   = "remote";
      data.username.cstring   = MQTT_USER;
      data.password.cstring   = MQTT_PASS;
      data.keepAliveInterval  = 10;
      data.cleansession   = 0;
      printf("Send MQTT connect ... ");
      retries = 0;

      while((ret = mqtt_connect(&client, &data))){
          printf("error: %d\n\r", ret);
          mqtt_network_disconnect(&network);
          retries++;
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
          if(retries >= MAX_RETRIES)
            break;
      }

      numtostr(sdk_system_get_chip_id(), device_id);
      printf("ID: %s\r\n", device_id);

      strcpy(topic, "device/");
      strcat(topic, device_id);
      strcat(topic, "/config");

      mqtt_subscribe(&client, topic, MQTT_QOS1, topic_received);

      while(1)
      {
        ret = mqtt_yield(&client, YIELD_DURATION);
        if (ret == MQTT_DISCONNECTED)
          break;
      }

      printf("Connection dropped, request restart\n\r");
      mqtt_network_disconnect(&network);
      vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
  }

}
void user_init(void)
{
    uart_set_baud(0, 115200);

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = "xDDD",
    };

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    sdk_wifi_station_connect();

    gpio_enable(led_gpio, GPIO_OUTPUT);
    gpio_write(led_gpio, true);

    xTaskCreate(&remoteControl, "remoteControl", 512, NULL, 2, NULL);
}
