#include <Arduino.h>
#include <Wire.h>
#include <Protocentral_FDC1004.h>

#define I2C_SDA 21 // ESP32 SDA on GPIO21
#define I2C_SCL 22 // ESP32 SCL on GPIO22

#define UPPER_BOUND  0X4000  // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 0
#define MEASURMENT 0

int capdac = 0;
char result[100];

FDC1004 FDC;
unsigned long lastMillis = 0;
const unsigned long interval = 10;  // 10ms for 100Hz

void setup()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= interval)
  {
    lastMillis += interval;

    uint16_t value[2];
    if (!FDC.readMeasurement(MEASURMENT, value))
    {
      int16_t msb = (int16_t) value[0];
      int32_t capacitance = ((int32_t)457) * ((int32_t)msb); // in attofarads
      capacitance /= 1000;   // in femtofarads
      capacitance += ((int32_t)3028) * ((int32_t)capdac);

      Serial.println((((float)capacitance / 1000)), 4);

      // Adjust CAPDAC if needed
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

    FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
    FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);
  }
}
