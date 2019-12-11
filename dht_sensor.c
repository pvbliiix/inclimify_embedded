#include <stdio.h>
#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <dht/dht.h>
#include "esp8266.h"

#include <unistd.h>
#include <string.h>
#include <stdbool.h>

#include "ssid_config.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <httpd/httpd.h>

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15

#define WEB_SERVER "192.168.20.15"
#define WEB_PORT "8000"

#define REQ_SIZE 256
#define RECV_SIZE 128

#define MAX_VOLTAGE 4.2
#define MIN_VOLTAGE 3.2

#define R1 350.0
#define R2 100.0
#define VREF 1.0
#define RES 1023.0

// time written in microseconds (us)

#define DEEPSLEEP_DURATION 55000000
#define RETRY_INTERVAL 1000

uint8_t const dht_gpio = D2;
uint8_t const led_gpio = D4;

const dht_sensor_type_t sensor_type = DHT_TYPE_DHT22;

typedef struct {
  int id;
  float humidity;
  float temperature;
  int batteryLevel;
} sensorData;

void wifiSetup()
{
    struct sdk_station_config config = {
      .ssid = WIFI_SSID,
      .password = WIFI_PASS,
    };

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
}

int getBatteryLevel()
{
    float sourceVoltage = ((VREF*(sdk_system_adc_read()/RES))*(R1+R2))/R2;

    printf("sourceVoltage: %.2f, ADC: %i, ID: %i", sourceVoltage, sdk_system_adc_read(), sdk_system_get_chip_id());

    int batteryLevel = (sourceVoltage - MIN_VOLTAGE) * 100;

    if(batteryLevel > 100)
      batteryLevel = 100;
    else if(batteryLevel < 0)
      batteryLevel = 0;

    return batteryLevel;
}

sensorData dhtMeasurementTask()
{
    sensorData sensor;

    sensor.id = sdk_system_get_chip_id();
    sensor.batteryLevel = getBatteryLevel();

    if (!dht_read_float_data(sensor_type, dht_gpio, &sensor.humidity, &sensor.temperature)) {
        sensor.humidity = 0;
        sensor.temperature = 0;
      }

    return sensor;
}

void sendSensorData(void *pvParameters)
{
    sensorData sensor = dhtMeasurementTask();

    if(!sensor.humidity && !sensor.temperature)
    {
      printf("Error while reading data from device\r\n");
    }
    else
    {
      printf("HTTP get task starting... temp: %.1f hum: %.1f battery: %i\r\n", sensor.temperature, sensor.humidity, sensor.batteryLevel);

      const struct addrinfo hints = {
          .ai_family = AF_UNSPEC,
          .ai_socktype = SOCK_STREAM,
        };

      struct addrinfo *res;

      printf("Running DNS lookup for %s...\r\n", WEB_SERVER);

      while (getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res) != 0 || res == NULL) {
          printf("DNS lookup failed\r\n");
          if(res)
            freeaddrinfo(res);
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
      }

      struct sockaddr *sa = res->ai_addr;

      if (sa->sa_family == AF_INET) {
          printf("DNS lookup succeeded. IP=%s\r\n", inet_ntoa(((struct sockaddr_in *)sa)->sin_addr));
      }

      int soc = socket(res->ai_family, res->ai_socktype, 0);
      while(soc < 0) {
          printf("... Failed to allocate socket.\r\n");
          freeaddrinfo(res);
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
      }

      printf("... allocated socket\r\n");

      while(connect(soc, res->ai_addr, res->ai_addrlen) != 0) {
          freeaddrinfo(res);
          printf("... socket connect failed.\r\n");
          printf("connect: %s\r\n", strerror(errno));
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
      }

      printf("... connected\r\n");
      freeaddrinfo(res);

      char req[REQ_SIZE], req_data[REQ_SIZE];

      snprintf(req_data, REQ_SIZE, "{\"device_id\":\"%i\",\"temperature\":\"%.1f\",\"humidity\":\"%.1f\","
      "\"battery_level\":\"%i\"}", sensor.id, sensor.temperature,
      sensor.humidity, sensor.batteryLevel);

      snprintf(req, REQ_SIZE, "POST /history/ HTTP/1.1\r\nHost: "WEB_SERVER"\r\n"
      "Content-Type: application/json\r\nContent-Length: %i\r\nConnection: close\r\n\r\n",
      strlen(req_data));

      strcat(req, req_data);

      while (write(soc, req, strlen(req)) < 0) {
          printf("... socket send failed\r\n");
          vTaskDelay(RETRY_INTERVAL / portTICK_PERIOD_MS);
        }
        printf("... socket send success\r\n");

      char recv_buf[RECV_SIZE];
      int recv;
      do {
        memset(recv_buf, 0, RECV_SIZE);
        recv = read(soc, recv_buf, RECV_SIZE-1);
        if(recv > 0) {
          printf("%s", recv_buf);
          }
      } while(recv > 0);

      close(soc);
    }
    sdk_system_deep_sleep(DEEPSLEEP_DURATION);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    gpio_enable(dht_gpio, 4);

    wifiSetup();

    xTaskCreate(sendSensorData, "sendSensorData", 1024, NULL, 2, NULL);
}
