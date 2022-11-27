/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

// #define BLYNK_DEBUG        // Optional, this enables more detailed prints

/*Comment #include "config.h"*/
#include "config.h"

/*Change your parameter*/
#define BLYNK_TEMPLATE_ID BLYNK_TEMPLATE_ID_OWNER
#define BLYNK_DEVICE_NAME BLYNK_DEVICE_NAME_OWNER
#define BLYNK_AUTH_TOKEN BLYNK_AUTH_TOKEN_OWNER

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include "RodaiSensor.h"

#include "esp_log.h"

#define SOIL_SENSOR_1 V5
#define SOIL_SENSOR_2 V6
#define SOIL_SENSOR_3 V7
#define SOIL_SENSOR_4 V8

#define AMBIENT_TEMPERATURE_SENSOR V20
#define AMBIENT_HUMIDITY_SENSOR V21
#define LIGHT_SENSOR V25

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

/*Change your parameter*/
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;

#define RS485_PORT UART_NUM_1
#define RS485_TX GPIO_NUM_23
#define RS485_RX GPIO_NUM_22
#define RS485_DIR GPIO_NUM_4
#define RS485_CTS UART_PIN_NO_CHANGE
#define RS485_BAUDRATE 9600

BlynkTimer timer;

RodaiSensor rodaiSensor = RodaiSensor(RS485_PORT, RS485_TX, RS485_RX, RS485_DIR, RS485_CTS, RS485_BAUDRATE);

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void myTimerEvent()
{

    // You can send any value at any time.
    // Please don't send more that 10 values per second.

    int soil1 = rodaiSensor.getRodaiSoilWaterContentValue(16);
    ESP_LOGI("MAIN", "soil1= %f", (float)soil1 / 1000.0f);
    Blynk.virtualWrite(SOIL_SENSOR_1, (float)soil1 / 1000.0f);

    int soil2 = rodaiSensor.getRodaiSoilWaterContentValue(17);
    ESP_LOGI("MAIN", "soil2= %f", (float)soil2 / 1000.0f);
    Blynk.virtualWrite(SOIL_SENSOR_2, (float)soil2 / 1000.0f);

    int soil3 = rodaiSensor.getRodaiSoilWaterContentValue(18);
    ESP_LOGI("MAIN", "soil3= %f", (float)soil3 / 1000.0f);
    Blynk.virtualWrite(SOIL_SENSOR_3, (float)soil3 / 1000.0f);

    int soil4 = rodaiSensor.getRodaiSoilWaterContentValue(19);
    ESP_LOGI("MAIN", "soil4= %f", (float)soil4 / 1000.0f);
    Blynk.virtualWrite(SOIL_SENSOR_4, (float)soil4 / 1000.0f);

    Blynk.virtualWrite(AMBIENT_TEMPERATURE_SENSOR, 35.64f);
    Blynk.virtualWrite(AMBIENT_HUMIDITY_SENSOR, 75.62f);
    Blynk.virtualWrite(LIGHT_SENSOR, 4324);
}

void setup()
{
    // Debug console
    Serial.begin(115200);

    Blynk.begin(auth, ssid, pass);

    // Setup a function to be called every second
    timer.setInterval(5000L, myTimerEvent);
}

void loop()
{
    Blynk.run();
    timer.run(); // Initiates BlynkTimer
}