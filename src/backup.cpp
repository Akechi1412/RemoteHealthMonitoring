// #include <Wire.h>
// #include "MAX30105.h"
// #include "heartRate.h"
// #include <OneWire.h>
// #include <DallasTemperature.h>

// #define   USE_FIFO
// #define   TIME_TO_BOOT    3000    // Wait for this time(msec) to output SpO2
// #define   SCALE           88.0    // Adjust to display heart beat and SpO2 in the same scale
// #define   SAMPLING        100     // 25 // 5 // If you want to see heart beat more precisely, set SAMPLING to 1
// #define   FINGER_ON       30000   // If red signal is lower than this, it indicates your finger is not on the sensor
// #define   RATE_SIZE       4       // Increase this for more averaging. 4 is good.
// #define   DS18B20_GPIO    15      // GPIO where the DS18B20 is connected to
// #define   DS18B20_DELAY   5000    // DS18B20 time delay

// // MAX30102 Sensor
// MAX30105 particleSensor;

// // Setup a oneWire instance to communicate with any OneWire devices
// OneWire oneWire(DS18B20_GPIO);

// // Pass our oneWire reference to Dallas Temperature sensor 
// DallasTemperature ds18b20(&oneWire);

// double    averageRed                 = 0;    // Average red level by low pass filter
// double    averageIr                  = 0;    // Average IR level by low pass filter
// double    sumRedRms                  = 0;    // Square sum of alternate component of red level
// double    sumIrRms                   = 0;    // Square sum of alternate component of IR level
// int       i                          = 0;    // For count 
// int       SpO2Sampling               = 100;  // Aalculate SpO2 by this sampling interval
// float     estimatedSpO2;                      // Initial value of estimated SpO2
// double    SpO2FilterFactor           = 0.7;  // Filter factor for estimated SpO2
// double    lpfRate                    = 0.95; // Low pass filter for IR/red LED value to eliminate AC component
// byte      beatRateList[RATE_SIZE];           // Array of heart rates
// byte      rateSpot                   = 0;
// long      lastBeat                   = 0;    // Time at which the last beat occurred
// float     beatsPerMinute             = 0;    // Beats per minute
// int       averageBeat                = 0;    // Average beat
// uint32_t  lastMillis                 = 0;

// void setup()
// {
//   Serial.begin(115200);
//   ds18b20.begin();
//   Serial.setDebugOutput(true);
//   Serial.println();
//   Serial.println("Running...");
//   delay(3000);

//   // Initialize sensor
//   while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
//   {
//     Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
//     while (1);
//   }

//   // Setup to sense a nice looking saw tooth on the plotter
//   byte ledBrightness = 0x7F;    // Options: 0=Off to 255=50mA
//   byte sampleAverage = 4;       // Options: 1, 2, 4, 8, 16, 32
//   byte ledMode       = 2;       // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
//   int sampleRate     = 200;     // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
//   int pulseWidth     = 411;     // Options: 69, 118, 215, 411
//   int adcRange       = 16384;   // Options: 2048, 4096, 8192, 16384
  
//   // Set up the wanted parameters
//   particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
//   particleSensor.enableDIETEMPRDY();
// }

// void loop()
// {
//   uint32_t ir, red, green;
//   uint32_t irRecent = particleSensor.getIR();
//   double dRed, dIr;
//   double SpO2 = 0; // Raw SpO2 before low pass filtered

//   // Read temperature sensor
//   if (millis() - lastMillis > DS18B20_DELAY) {
//     ds18b20.requestTemperatures(); 
//     float temperatureC = ds18b20.getTempCByIndex(0);
//     float temperatureF = ds18b20.getTempFByIndex(0);
//     Serial.print("Temperature: ");
//     Serial.print(temperatureC);
//     Serial.print("ºC,");
//     Serial.print(temperatureF);
//     Serial.println("ºF");
//     lastMillis = millis();
//   }

//   //Calculate BPM independent of Maxim Algorithm. 
//   if (checkForBeat(irRecent) == true)
//   {
//     //We sensed a beat!
//     long delta = millis() - lastBeat;
//     lastBeat = millis();

//     beatsPerMinute = 60 / (delta / 1000.0);

//     if (beatsPerMinute < 255 && beatsPerMinute > 20)
//     {
//       beatRateList[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
//       rateSpot %= RATE_SIZE; // Wrap variable

//       // Take average of readings
//       averageBeat = 0;
//       for (byte x = 0 ; x < RATE_SIZE ; x++)
//         averageBeat += beatRateList[x];
//       averageBeat /= RATE_SIZE;
//     }
//   }

// #ifdef USE_FIFO
//   particleSensor.check(); // Check the sensor, read up to 3 samples

//   while (particleSensor.available()) // Do we have new data 
//   {
// #ifdef MAX30105
//   red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
//   ir  = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
// #else
//   red = particleSensor.getFIFOIR();  // Why getFIFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
//   ir  = particleSensor.getFIFORed(); // Why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
// #endif
//     i++;
//     dRed = (double)red;
//     dIr  = (double)ir;
//     averageRed = averageRed * lpfRate + (double)red * (1.0 - lpfRate);
//     averageIr = averageIr * lpfRate + (double)ir * (1.0 - lpfRate);
//     sumRedRms += (dRed - averageRed) * (dRed - averageRed);
//     sumIrRms += (dIr - averageIr) * (dIr - averageIr);

//     if ((i % SAMPLING) == 0) // Slow down graph plotting speed for arduino Serial plotter by thin out
//     {
//       if ( millis() > TIME_TO_BOOT) 
//       {
//         float irForGraph = (2.0 * dIr - averageIr) / averageIr * SCALE;
//         float redForGraph = (2.0 * dRed - averageRed) / averageRed * SCALE;

//         // Truncation for Serial plotter's autoscaling
//         if ( irForGraph > 100.0) 
//           irForGraph = 100.0;
//         else if ( irForGraph < 80.0) 
//           irForGraph = 80.0;
//         if ( redForGraph > 100.0 ) 
//           redForGraph = 100.0;
//         else if ( redForGraph < 80.0 ) 
//           redForGraph = 80.0;

//         // Print out red and IR sensor reading to serial interface for monitoring...
//         Serial.print("Red: "); 
//         Serial.print(red); 
//         Serial.print(","); 
//         Serial.print("Infrared: "); 
//         Serial.print(ir); 
//         Serial.print(".    ");
        
//         if (ir < FINGER_ON) // No finger on the sensor
//         {
//            Serial.println("No finger detected");
//            break;
//         }
//         else if(ir > FINGER_ON)
//         {
//             Serial.print("Heart Rate = ");
//            Serial.print(averageBeat);
//            Serial.print(",Oxygen % = ");
//            Serial.print(estimatedSpO2);
//            Serial.println("%");
//         }
//       }
//     }

//     if ((i % SpO2Sampling) == 0) {
//       double R = (sqrt(sumRedRms) / averageRed) / (sqrt(sumIrRms) / averageIr);
//       // Serial.println(R);
//       SpO2 = -23.3 * (R - 0.4) + 100; // http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf -- I don't see this directly in the App Note... look here https://github.com/espressif/arduino-esp32/issues/4561
//       estimatedSpO2 = SpO2FilterFactor * estimatedSpO2 + (1.0 - SpO2FilterFactor) * SpO2; // Low pass filter
//       // Serial.print(SpO2);Serial.print(",");Serial.println(estimatedSpO2);
//       sumRedRms = 0.0; 
//       sumIrRms = 0.0; 
//       i = 0;
//       break;
//     }

//     particleSensor.nextSample(); // We're finished with this sample so move to next sample
//   }
// #endif
// }
