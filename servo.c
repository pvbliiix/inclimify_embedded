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
#include <pwm.h>

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

uint8_t static const servo1_pin = D3;
uint8_t static const servo2_pin = D4;

#define MQTT_HOST ("") // INSERT IP HERE
#define MQTT_PORT 1883

#define MQTT_USER "" // INSERT CREDENTIALS HERE
#define MQTT_PASS ""

#define TOPIC_LEN 32

#define BUF_SIZE 100

#define RETRY_INTERVAL 1000
#define YIELD_DURATION 5000
#define COMMAND_TIMEOUT 10000

#define MAX_RETRIES 5

void topic_received(mqtt_message_data_t *md)
{
    if(md != NULL)
    {
      mqtt_message_t *message = md->message;
    char cmd[(int)message->payloadlen];

    memcpy(cmd, ((char *)(message->payload)), (int)message->payloadlen);

    cmd[(int)message->payloadlen] = '\0';

    printf("COMMAND: %s\r\n", cmd);

    uint8_t pin[1];
    char servo[8];
    int degrees = 0;

    sscanf(cmd, "%s %i", servo, &degrees);

    if(!strcmp(servo, "SERVO1"))
    {
      pin[0] = 0;

      pwm_init(1, pin, false);

      pwm_set_freq(50);
      pwm_start();

      if(degrees <= 180 && degrees >= 0)
        pwm_set_duty(3300+(35*degrees));
    }

    else if(!strcmp(servo, "SERVO2"))
    {
      pin[0] = 2;

      pwm_init(1, pin, false);

      pwm_set_freq(50);
      pwm_start();

      if(degrees <= 180 && degrees >= 0)
        pwm_set_duty(3300+(35*degrees));
    }
    else
      printf("INVALID COMMAND: \"%s\"\r\n", cmd);
    }

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
      data.clientID.cstring   = "servo";
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

      printf("ID: %s\r\n", device_id);

      strcpy(topic, "device/servo/config");

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
        .password = WIFI_PASS,
    };

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    sdk_wifi_station_connect();

    xTaskCreate(&remoteControl, "remoteControl", 512, NULL, 2, NULL);
}
