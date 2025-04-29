/************************************************************
 *                      INCLUDES
 ************************************************************/ 
#include <Arduino.h>
#include <Wire.h>
#include <Protocentral_FDC1004.h>

/************************************************************
 *                      PIN DEF
 ************************************************************/ 
#define I2C_SDA 21 // ESP32 SDA on GPIO21
#define I2C_SCL 22 // ESP32 SCL on GPIO22

/************************************************************
 *                      CONSTANTS
 ************************************************************/ 
#define UPPER_BOUND  0X4000             // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND) // min readout capacitance
#define CHANNEL 0
#define MEASURMENT 0
#define INTERVAL 10                     // 10ms for 100Hz

FDC1004 FDC;
/************************************************************
 *                      CODE
 ************************************************************/ 

/*
 * Initialize I2C and Serial Com
 */
void setup()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
}

/*
 * Main Loop
 */
void loop()
{
  // Variables
  char result[100];
  uint8_t capdacResetFlag = 0;
  int32_t capdac = 0;
  uint64_t lastMillis[] = {0, 0};
  uint64_t currentMillis = 0;

  int32_t averageCount = 0;
  float averageSum = 0.0;
  float minValue = 0.0;
  float maxValue = 0.0;


  // Init time variables
  lastMillis[0] = millis();
  lastMillis[1] = millis();

  while(1)
  {
    // Every system cicle
    currentMillis = millis();

    // Every interval of time
    if (currentMillis - lastMillis[0] >= INTERVAL)
    {
      // Update time
      lastMillis[0] += INTERVAL;

      // Read FDC measurement
      uint16_t value[2];
      if (!FDC.readMeasurement(MEASURMENT, value))
      {
        // Treat data
        uint16_t msb = (int16_t) value[0];
        uint16_t lsb = (int16_t) value[1];
        uint32_t byte = (uint32_t) (msb<<16 | lsb);
        byte = byte & 0xFFFFFF00;
        byte = byte>>8;

        // Convert to capacitance (My Method)
        float offset = ((float) (capdac))*3.125;
        float cap = ((float) ((float) byte / 0x00080000));
        float total = offset + cap;

        // Avarege and min max values
        averageSum += total;
        averageCount++;
        if(total > maxValue)
          maxValue = total;
        if(total < minValue || minValue == 0)
          minValue = total;

        // Old capacitance convert method
        int32_t capacitance = ((int32_t)457) * ((int32_t)msb); // in attofarads
        capacitance /= 1000;   // in femtofarads
        capacitance += ((int32_t)3028) * ((int32_t)capdac);
        
        // Serial Print
        // Serial.printf("offset: %7.3f pF || ", offset);
        // Serial.printf("capac:  %7.3f pF || ", cap);
        // Serial.printf("total:  %7.3f pF || \r\n", total);
        // Serial.println((((float)capacitance / 1000)), 4);
        
        // Adjust CAPDAC (if needed)
        if (msb > UPPER_BOUND)
        {
          if (capdac < FDC1004_CAPDAC_MAX)
            capdac++;
        }
        else if (msb < LOWER_BOUND)
        {
          if (capdac > 0)
            capdac--;
        }
      }

      // Trigger Measurement
      FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
      FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);
    }

    // Every 1 sec
    if(currentMillis - lastMillis[1] >= INTERVAL*100 && averageSum)
    {
      // Update time
      lastMillis[1] += INTERVAL*100;

      // Serial Print Average 
      Serial.printf("Average: %7.3f pF || ", averageSum/averageCount);
      Serial.printf("Min: %7.3f pF || ", minValue);
      Serial.printf("Max: %7.3f pF || ", maxValue);
      Serial.printf("Variation: %7.3f pF || ", maxValue - minValue);
      Serial.printf("Samples: %d\r\n", averageCount);

      // Reset Average Variables
      averageSum = 0;
      averageCount = 0;
      minValue = 0.0;
      maxValue = 0.0;

      // Reset capdac if it is max for 3 sec  
      if(capdac == FDC1004_CAPDAC_MAX)
        capdacResetFlag++;
      if(capdacResetFlag > 2)
      {
        capdac = 0;
        capdacResetFlag = 0;
      }
    }
  }
}
