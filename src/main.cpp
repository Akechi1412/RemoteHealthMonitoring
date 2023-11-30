#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <WiFi.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include "WiFiManager.h"

#define   MAX30102_USE_FIFO
#define   MAX30102_FINGER_ON        30000   // If red signal is lower than this, it indicates your finger is not on the sensor
#define   MAX30102_RATE_SIZE        4       // Increase this for more averaging. 4 is good.
#define   DS18B20_GPIO              25      // GPIO where the DS18B20 is connected to
#define   DS18B20_DELAY             3000    // DS18B20 time delay 
#define   SCREEN_WIDTH              128
#define   SCREEN_HEIGHT             64
#define   OLED_MOSI                 23
#define   OLED_CLK                  18
#define   OLED_DC                   16
#define   OLED_CS                   5
#define   OLED_RESET                17
#define   AD8232_ADC                33
#define   AD8232_L1                 27
#define   AD8232_L2                 14
#define   ECG_SAMPLING_RATE         5
#define   ECG_BLOCK_LENGTH          200
#define   NORMAL_HUMAN_TEMP         35.5
#define   AWS_SUBSCRIBE_TOPIC1      "max30102/sub"
#define   AWS_SUBSCRIBE_TOPIC2      "ds18b20/sub"
#define   AWS_SUBSCRIBE_TOPIC3      "ad8232/sub"
#define   AWS_PUBLISH_TOPIC1        "max30102/pub"
#define   AWS_PUBLISH_TOPIC2        "ds18b20/pub"
#define   AWS_PUBLISH_TOPIC3        "ad8232/pub"

enum dataType_t { MAX30102, DS18B20, AD8232 };

typedef struct {
  int heartRate;
  float spo2;
} max30102Data_t;

typedef struct {
  max30102Data_t max30102Data;
  float temperature;
  uint16_t ecgBlock[ECG_BLOCK_LENGTH]; 
  dataType_t dataType;
} data_t;

// Global variable
QueueHandle_t queueHandle;
const int queueElementSize = 10;
float ambianceTemp = 0;
Adafruit_SSD1306 ssd1306(SCREEN_WIDTH, SCREEN_HEIGHT,
    OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Prototype
void readMAX30102SensorTask(void *pvParameters);
void readDS18B20SensorTask(void *pvParameters);
void readAD8232SensorTask(void *pvParamters);
void vReceiverTask(void *pvParameters);
void messageHandler(char* topic, byte* payload, unsigned int length);
void connectAWS();
void publishMAX30102Data(int heartRate, float spo2);
void publishDS18B20Data(float temperature);
void publishAD82832Data(uint16_t ecgBlock[], size_t size);

void setup()
{
  Serial.begin(115200);

  if(!ssd1306.begin(SSD1306_SWITCHCAPVCC))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  ssd1306.clearDisplay();
  ssd1306.display();
  delay(1000);
  ssd1306.clearDisplay();
  ssd1306.setTextSize(2);
  ssd1306.setTextColor(SSD1306_WHITE);
  ssd1306.setCursor(3, 3);
  ssd1306.printf("  Remote\n  Health\nMonitoring");
  ssd1306.display();
  delay(4000);
  ssd1306.clearDisplay();
  ssd1306.setCursor(3, 14);
  ssd1306.print("Connecting...");
  ssd1306.display();

  connectAWS();

  ssd1306.clearDisplay();

  xTaskCreatePinnedToCore(
    readMAX30102SensorTask
    , "Read MAX30102 Sensor Task"
    , 2048
    , NULL
    , 1
    , NULL
    , 1
  );

  xTaskCreatePinnedToCore(
    readDS18B20SensorTask
    , "Read DS18B20 Sensor Task"
    , 2048
    , NULL
    , 1
    , NULL
    , 1
  );

  xTaskCreatePinnedToCore(
    readAD8232SensorTask
    , "Read AD8232 Sensor Task"
    , 10240
    , NULL
    , 3
    , NULL
    , 1
  );

  xTaskCreatePinnedToCore(
    vReceiverTask
    , "Receiver Task"
    , 10240
    , NULL
    , 2
    , NULL
    , 0
  );

  queueHandle = xQueueCreate(queueElementSize, sizeof(data_t));

  // Check if the queue was successfully created
  if(queueHandle == NULL){
    Serial.println("Queue could not be created. Halt.");
    while(1) 
    {
      delay(1000); // Halt at this point as is not possible to continue
    }
  }
}

void loop()
{

}

void readMAX30102SensorTask(void *pvParameters)
{
  const int       SPO2_SAMPLING        = 100;  // Calculate SpO2 by this sampling interval
  const double    SPO2_FILTER_FACTOR   = 0.7;  // Filter factor for estimated SpO2
  const double    LPF_RATE             = 0.95; // Low pass filter for IR/red LED value to eliminate AC component
  MAX30105 particleSensor;

  data_t    data;                              // Sensor data to send to queue
  double    averageRed                 = 0;    // Average red level by low pass filter
  double    averageIr                  = 0;    // Average IR level by low pass filter
  double    sumRedRms                  = 0;    // Square sum of alternate component of red level
  double    sumIrRms                   = 0;    // Square sum of alternate component of IR level
  int       i                          = 0;    // For count 
  float     estimatedSpo2              = 0;    // Initial value of estimated SpO2
  byte      beatRateList[MAX30102_RATE_SIZE];  // Array of heart rates
  byte      rateSpot                   = 0;
  long      lastBeat                   = 0;    // Time at which the last beat occurred
  float     beatsPerMinute             = 0;    // Beats per minute
  int       averageBeat                = 0;    // Average beat
  uint32_t  lastMillis                 = 0;

  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }

  // Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F;    // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;       // Options: 1, 2, 4, 8, 16, 32
  byte ledMode       = 2;       // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate     = 200;     // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth     = 411;     // Options: 69, 118, 215, 411
  int adcRange       = 16384;   // Options: 2048, 4096, 8192, 16384
  
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();

  while(1)
  {
    uint32_t ir, red, green;
    uint32_t irRecent = particleSensor.getIR();
    double dRed, dIr;
    double spo2 = 0; // Raw SpO2 before low pass filtered

    //Calculate BPM independent of Maxim Algorithm. 
    if (checkForBeat(irRecent) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        beatRateList[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= MAX30102_RATE_SIZE; // Wrap variable

        // Take average of readings
        averageBeat = 0;
        for (byte x = 0 ; x < MAX30102_RATE_SIZE ; x++)
          averageBeat += beatRateList[x];
        averageBeat /= MAX30102_RATE_SIZE;
      }
    }

#ifdef MAX30102_USE_FIFO
    particleSensor.check(); // Check the sensor, read up to 3 samples

    while (particleSensor.available()) // Do we have new data 
    {
#ifdef MAX30105
    red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
    ir  = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR();  // Why getFIFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir  = particleSensor.getFIFORed(); // Why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif
      i++;
      dRed = (double)red;
      dIr  = (double)ir;
      averageRed = averageRed * LPF_RATE + dRed * (1.0 - LPF_RATE);
      averageIr = averageIr * LPF_RATE + dIr * (1.0 - LPF_RATE);
      sumRedRms += (dRed - averageRed) * (dRed - averageRed);
      sumIrRms += (dIr - averageIr) * (dIr - averageIr);

      if ((i % SPO2_SAMPLING) == 0) {
        ambianceTemp = particleSensor.readTemperature();

        double R = (sqrt(sumRedRms) / averageRed) / (sqrt(sumIrRms) / averageIr);
        // Serial.println(R);
        spo2 = -23.3 * (R - 0.4) + 100; // http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf -- I don't see this directly in the App Note... look here https://github.com/espressif/arduino-esp32/issues/4561
        estimatedSpo2 = SPO2_FILTER_FACTOR * estimatedSpo2 + (1.0 - SPO2_FILTER_FACTOR) * spo2; // Low pass filter
        
        if (ir < MAX30102_FINGER_ON) // No finger on the sensor
        {
          // Serial.println("No finger detected");
          break;
        }
        else if(ir > MAX30102_FINGER_ON)
        {
          data.max30102Data.heartRate = averageBeat;
          data.max30102Data.spo2 = estimatedSpo2;
          data.dataType = MAX30102;
          BaseType_t returnValue = xQueueSend(queueHandle, (void*)&data, 0);
          if(returnValue == errQUEUE_FULL)
          {
            Serial.println("The `readMAX30102SensorTask` was unable to send data into the Queue");
          } 
        }

        sumRedRms = 0.0; 
        sumIrRms = 0.0; 
        i = 0;

        break;
      }

      particleSensor.nextSample(); // We're finished with this sample so move to next sample
    }
#endif
  }
}

void readDS18B20SensorTask(void *pvParameters)
{
  data_t data;

  // Setup a oneWire instance to communicate with any OneWire devices
  OneWire oneWire(DS18B20_GPIO);

  // Pass our oneWire reference to Dallas Temperature sensor 
  DallasTemperature ds18b20(&oneWire);

  ds18b20.begin();

 while(1)
  {
    // Serial.printf("Ambiance Temp: %f\n", ambianceTemp);
    ds18b20.requestTemperatures(); 
    float temperatureC = ds18b20.getTempCByIndex(0) + 0.5;
    if (temperatureC != DEVICE_DISCONNECTED_C) 
    {
      data.temperature = temperatureC;
      data.dataType = DS18B20;
      BaseType_t returnValue = xQueueSend(queueHandle, (void*)&data, portMAX_DELAY);
      if(returnValue == errQUEUE_FULL)
      {
        Serial.println("The `readDS18B20SensorTask` was unable to send data into the Queue");
      } 
    }
    vTaskDelay(DS18B20_DELAY / portTICK_PERIOD_MS);
  }
}

void readAD8232SensorTask(void *pvParamters)
{
  portTickType    xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  data_t data;
  int i = 0, j = 0;

  pinMode(AD8232_L2, INPUT); // Setup for leads off detection LO +
  pinMode(AD8232_L1, INPUT); // Setup for leads off detection LO -

  while(1)
  {
    if((digitalRead(AD8232_L2) == 1) || (digitalRead(AD8232_L1) == 1))
    {
      // Serial.println("AD8232: No detected!");
      j++;
      if (j >= ECG_BLOCK_LENGTH)
      {
        j = 0;
        i = 0;
      }
    }
    else
    {
      data.ecgBlock[i] = analogRead(AD8232_ADC);
      i++;
    }

    if (i == ECG_BLOCK_LENGTH)
    {
      i = 0;
      j = 0;
      data.dataType = AD8232;
      BaseType_t returnValue = xQueueSend(queueHandle, (void*)&data, portMAX_DELAY);
      if(returnValue == errQUEUE_FULL)
      {
        Serial.println("The `readAD8232SensorTask` was unable to send data into the Queue");
      } 
    }

    xTaskDelayUntil(&xLastWakeTime, ECG_SAMPLING_RATE / portTICK_PERIOD_MS);
  }
}

void vReceiverTask(void *pvParameters)
{
  data_t data;
  uint32_t lastMillis = 0;
  int i = 0;
  int heartRate = 0;
  float spo2 = 0;
  float temperature = 0;
  bool  ecgDetected = false;

  while(1)
  {
    if (queueHandle != NULL)
    {
      BaseType_t returnValue = xQueueReceive(queueHandle, &data, portMAX_DELAY);
      if(returnValue == pdFALSE)
      {
        Serial.println("The `vReceiverTask` was unable to receive data from the Queue");
        continue;
      }
      switch (data.dataType)
      {
      case MAX30102:
        heartRate = data.max30102Data.heartRate;
        spo2 = data.max30102Data.spo2;
        publishMAX30102Data(heartRate, spo2);
        Serial.print("Heart Rate = ");
        Serial.print(heartRate);
        Serial.print(",Oxygen % = ");
        Serial.print(spo2);
        Serial.println("%");
        break;

      case DS18B20:
        if (ambianceTemp == 0)
          break;
        if (data.temperature > ambianceTemp || data.temperature >= NORMAL_HUMAN_TEMP)
        {
          if (data.temperature > temperature)
          {
            temperature = data.temperature;
            publishDS18B20Data(temperature);
            // Serial.printf("Temperature: %.1f ºC\n", temperature);
          }
        }
        break;

      case AD8232:
        ecgDetected = true;
        publishAD82832Data(data.ecgBlock, ECG_BLOCK_LENGTH);
        break;
        
      default:
        break;
      }
    }

    client.loop();

    if (millis() - lastMillis >= 2500)
    {
      if (i == 0)
      {
        ssd1306.clearDisplay();
        ssd1306.setCursor(18, 4);
        if (temperature != 0)
        {
          ssd1306.printf("T:%.1fC\n\n", temperature);
        }
        else
        {
          ssd1306.printf("T: --.-C\n\n");
        }
        if (ecgDetected)
        {
          ssd1306.print("  ECG: D");
          ecgDetected = false;
        }
        else
        {
          ssd1306.print("  ECG: ND");
        }
        ssd1306.display();
        i++;
      }
      else 
      {
        ssd1306.clearDisplay();
        ssd1306.setCursor(20, 4);
        if (heartRate != 0)
        {
          ssd1306.printf("BPM:%d\n\n", heartRate);
        }
        else
        {
          ssd1306.printf("BPM:--\n\n");
        }
        if (spo2 != 0)
        {
          ssd1306.printf("SpO2:%.1f%%", spo2);
        }
        else
        {
          ssd1306.printf("SpO2:--.-%%");
        }
        ssd1306.display();
        i = 0;
      }
      lastMillis = millis();
    }
  }
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void connectAWS()
{
  Serial.println("Connecting to Wi-Fi");

  WiFi.mode(WIFI_STA); 
  WiFiManager wm;
  // wm.resetSettings();
  bool res;
  res = wm.autoConnect("RemoteHealthMonitoringAP", "password");

  if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
  } 
  else {   
      Serial.println("Wifi connected");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_SUBSCRIBE_TOPIC1);
  client.subscribe(AWS_SUBSCRIBE_TOPIC2);
  client.subscribe(AWS_SUBSCRIBE_TOPIC3);
 
  Serial.println("AWS IoT Connected!");
}

void publishMAX30102Data(int heartRate, float spo2)
{
  StaticJsonDocument<200> doc;
  doc["heartRate"] = heartRate;
  doc["spo2"] = spo2;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
 
  client.publish(AWS_PUBLISH_TOPIC1, jsonBuffer);
}

void publishDS18B20Data(float temperature)
{
  StaticJsonDocument<200> doc;
  doc["temperature"] = temperature;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
 
  client.publish(AWS_PUBLISH_TOPIC2, jsonBuffer);
}

void publishAD82832Data(uint16_t ecgBlock[], size_t size)
{
  StaticJsonDocument<200> doc;
  JsonArray ecgArray = doc.createNestedArray("ecgBlock");
  
  for (size_t i = 0; i < size; i++) {
    ecgArray.add(ecgBlock[i]);
  }
  
  char jsonBuffer[10240];
  serializeJson(doc, jsonBuffer);
 
  client.publish(AWS_PUBLISH_TOPIC3, jsonBuffer);
}