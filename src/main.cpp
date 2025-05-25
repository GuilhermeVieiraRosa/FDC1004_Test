/************************************************************
 *                      INCLUDES
 ************************************************************/ 
#include <Arduino.h>
#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include <math.h>

/************************************************************
 *                      PIN DEF
 ************************************************************/ 
#define I2C_SDA 40 // ESP32 SDA on GPIO21
#define I2C_SCL 41 // ESP32 SCL on GPIO22

/************************************************************
 *                      CONSTANTS
 ************************************************************/ 
#define UPPER_BOUND  0X4000             // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND) // min readout capacitance
#define CHANNEL 0
#define MEASURMENT 0
#define INTERVAL 10                     // 10ms for 100Hz

FDC1004 FDC;

#define TMP117_ADDR 0X48

/************************************************************
 *                      STRUCTS
 ************************************************************/ 

typedef struct {
  float average;
  int32_t averageCount;
  float averageSum;
  float minValue;
  float maxValue;
  float variance;
  float varianceSum;
  float deviation;

  float vector[600];
} Data;

/************************************************************
 *                      METHODS
 ************************************************************/ 

void main_adjustCapdac(uint16_t msb, int32_t* capdac);
void main_resetCapdac(int32_t* capdac, uint8_t* capdacResetFlag);
void main_dataInit(Data* data);
void main_dataCalc(Data* data, uint32_t index);
void main_dataPrint(Data* data);

/************************************************************
 *                      CODE
 ************************************************************/ 

/*
 * Initialize I2C and Serial Com
 */
void setup()
{
  USBSerial.begin(115200);
  while (!USBSerial) {
    delay(10);  // Aguarda a conexão com o host
  }
  delay(1000);
  USBSerial.println("TCC FDC1004 GUILHERME E ODAIR");

  Wire.begin(I2C_SDA, I2C_SCL);
}

/*
 * Main Loop
 */
void loop()
{
  // Variables
  uint8_t capdacResetFlag = 0;
  int32_t capdac = 0;
  uint32_t index = 0;
  uint64_t lastMillis[] = {0, 0};
  uint64_t currentMillis = 0;

  Data data;
  main_dataInit(&data);

  // Init time variables
  lastMillis[0] = millis();
  lastMillis[1] = millis();

  // System cicle
  while(1)
  {
    // Every system cicle
    currentMillis = millis();

    // Every 10 milisec
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

        // Store data
        data.vector[index++] = total;
        data.averageSum += total;
        data.averageCount++;

        if(total > data.maxValue)
          data.maxValue = total;
        if(total < data.minValue || data.minValue == 0)
          data.minValue = total;
        
        // Adjust CAPDAC (if needed)
        main_adjustCapdac(msb, &capdac);
      }

      // Trigger Measurement
      FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
      FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);
    }

    // Every 5000 milisec
    if(currentMillis - lastMillis[1] >= INTERVAL*500)
    {
      // Update time
      lastMillis[1] += INTERVAL*500;

      // If averageSum is not zero, print data
      if(data.averageSum)
      {
        // Data calc
        main_dataCalc(&data, index);

        // Serial Print Average 
        main_dataPrint(&data);
      }

      // Reset Average Variables
      index = 0;
      main_dataInit(&data);

      // If capdac is max for 3 5 sec cicles, Reset capdac
      main_resetCapdac(&capdac, &capdacResetFlag);
    }
  }
}

void main_adjustCapdac(uint16_t msb, int32_t* capdac)
{
  if (msb > UPPER_BOUND)
  {
    if (*capdac < FDC1004_CAPDAC_MAX)
      *capdac++;
  }
  else if (msb < LOWER_BOUND)
  {
    if (*capdac > 0)
      *capdac--;
  }
}

void main_resetCapdac(int32_t* capdac, uint8_t* capdacResetFlag)
{
  if(*capdac == FDC1004_CAPDAC_MAX)
    *capdacResetFlag++;
  if(*capdacResetFlag > 2)
  {
    *capdac = 0;
    *capdacResetFlag = 0;
  }
}

void main_dataInit(Data* data)
{
  data->average = 0.0;
  data->averageCount = 0;
  data->averageSum = 0.0;

  data->minValue = 0.0;
  data->maxValue = 0.0;

  data->variance = 0.0;
  data->varianceSum = 0.0;

  data->deviation = 0.0;
}

void main_dataCalc(Data* data, uint32_t index)
{
  uint32_t i = 0;

  // Average
  data->average = data->averageSum / data->averageCount;

  // Standard Deviation 
  for(i = 0; i < index; i++)
  {
    data->varianceSum += pow(data->vector[i] - data->average, 2);
  }
  data->variance = data->varianceSum / data->averageCount;
  data->deviation = sqrt(data->variance);
}

void main_dataPrint(Data* data)
{
  USBSerial.printf("Average: %7.3f pF || ", data->average);
  USBSerial.printf("Min: %7.3f pF || ", data->minValue);
  USBSerial.printf("Max: %7.3f pF || ", data->maxValue);
  USBSerial.printf("Span: %7.3f pF || ", data->maxValue - data->minValue);
  USBSerial.printf("Deviation: %7.3e pF || ",  data->deviation);
  USBSerial.printf("Samples: %d\r\n", data->averageCount);
}