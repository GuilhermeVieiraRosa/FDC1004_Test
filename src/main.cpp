#include <Arduino.h>
#include <Wire.h>
#include <Protocentral_FDC1004.h>

#define I2C_SDA 21 // ESP32 SDA on GPIO21
#define I2C_SCL 22 // ESP32 SCL on GPIO22

#define UPPER_BOUND  0X4000  // max readout capacitance
#define LOWER_BOUND  0x2000  //(-1 * UPPER_BOUND)
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

    //Serial.printf("---------------------------------------------------------------\r\n", 0);
    uint16_t value[2];
    if (!FDC.readMeasurement(MEASURMENT, value))
    {
      uint16_t msb = (int16_t) value[0];
      uint16_t lsb = (int16_t) value[1];
      //Serial.printf("msb: 0x%04X\n", msb);
      //Serial.printf("lsb: 0x%04X\n", lsb);

      uint32_t byte = (uint32_t) (msb<<16 | lsb);
      //Serial.printf("byte: 0x%08X\n", read);
      byte = byte & 0xFFFFFF00;
      byte = byte>>8;
      //Serial.printf("byte: 0x%08X\n", byte);

      //Serial.printf("capdac: %d\n", capdac);
      float offset = ((float) (capdac))*3.125;
      Serial.printf("offset: %7.3f pF || ", offset);
      float cap = ((float) ((float) byte / 0x00080000));
      Serial.printf("capac:  %7.3f pF || ", cap);
      Serial.printf("total:  %7.3f pF || ", cap + offset);

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
