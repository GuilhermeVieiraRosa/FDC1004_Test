/************************************************************
 *                      INCLUDES
 ************************************************************/ 
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Protocentral_FDC1004.h>
#include <PID_v1.h>

/************************************************************
 *                      PIN DEF
 ************************************************************/ 
#define I2C_SDA 40
#define I2C_SCL 41
#define HEATER_PIN 2

/************************************************************
 *                      CONSTANTS
 ************************************************************/
// Timers
#define INTERVAL_10MS   10                     // 10ms for 100Hz
#define INTERVAL_100MS  100
#define INTERVAL_5000MS 5000

// FDC
#define UPPER_BOUND  0X4000             // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND) // min readout capacitance
#define CHANNEL 0
#define MEASURMENT 0
FDC1004 FDC;

// TMP
#define TMP117_ADDR 0X48
#define TMP117_CONFIG 0x01

// PID
#define HEATER_EN
#define PWM_PIN HEATER_PIN  // GPIO2 for heater control
#define PWM_CHANNEL 0       // LEDC PWM channel
#define PWM_FREQ 1000       // PWM frequency in Hz
#define PWM_RESOLUTION 12   // PWM resolution
#define PWM_SET_POINT 25.0
// PID parameters
double Setpoint = 40.0; // Target temperature in °C
double Input = 0.0;     // Current temperature (updated at 64 Hz)
double Output = 0.0;    // PID output
// PID tuning parameters (adjust as needed)
double Kp = 40; // Proportional gain  40        84.4;
double Ki = 1;  // Integral gain      1         7.74;
double Kd = 0;  // Derivative gain    0.019     229.57;
// PID Object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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

  double temperature;
} Data;

/************************************************************
 *                      METHODS
 ************************************************************/ 

void main_adjustCapdac(uint16_t msb, int32_t* capdac);
void main_resetCapdac(int32_t* capdac, uint8_t* capdacResetFlag);
void main_dataInit(Data* data);
void main_dataCalc(Data* data, uint32_t index);
void main_dataPrint(Data* data);
void tmp117_writeRegister(uint8_t reg, uint16_t value);
uint16_t tmp117_readRegister(uint8_t reg);
float tmp117_readTemperature(void);
void tmp117_init(void);

/************************************************************
 *                      CODE
 ************************************************************/ 

/*
 * Initialize I2C and Serial Com
 */
void setup()
{
  // Serial
  USBSerial.begin(115200);
  while (!USBSerial) {
    delay(10);  // Aguarda a conexão com o host
  }
  delay(1000);
  USBSerial.println("TCC FDC1004 GUILHERME E ODAIR");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // PID
#ifdef HEATER_EN
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, (1 << PWM_RESOLUTION) - 1); // Match PWM range
  myPID.SetSampleTime(1000 / 10.0);                    // Match 10 Hz sample rate
#endif
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
  uint64_t lastMillis[] = {0, 0, 0};
  uint64_t currentMillis = 0;

  uint8_t dutyCicle = 0;

  Data data;
  main_dataInit(&data);

  // Init time variables
  lastMillis[0] = millis();
  lastMillis[1] = millis();
  lastMillis[2] = millis();

  // TMP117
  tmp117_init();

  // System cicle
  while(1)
  {
    // Every system cicle
    currentMillis = millis();

    // Every 10 milisec
    if (currentMillis - lastMillis[0] >= INTERVAL_10MS)
    {
      // Update time
      lastMillis[0] += INTERVAL_10MS;

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

    // Every 1000 milisec
    if(currentMillis - lastMillis[1] >= INTERVAL_100MS)
    {
      // Update time
      lastMillis[1] += INTERVAL_100MS;

      // Temperature Polling
      data.temperature = tmp117_readTemperature();
      Input = data.temperature;

      // PID
#ifndef PID_TEST
      // Compute the PID output
      if (myPID.Compute())
      {
        // Apply the PID output to the PWM pin
        ledcWrite(PWM_CHANNEL, (uint32_t)Output);
      }
#endif
    }
    
    // Every 5000 milisec
    if(currentMillis - lastMillis[2] >= INTERVAL_5000MS)
    {
      // Update time
      lastMillis[2] += INTERVAL_5000MS;

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

  data->temperature = 0.0;
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
  USBSerial.printf("Samples: %d || ", data->averageCount);
  USBSerial.printf("Temperature: %7.3f °C || ", data->temperature);
  USBSerial.printf("DutyCicle: %7.3f %%\r\n", 100 * Output / ((1 << PWM_RESOLUTION) - 1));
}

// Function to write a 16-bit value to a TMP117 register
void tmp117_writeRegister(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(reg);                 // Register address
  Wire.write((value >> 8) & 0xFF); // MSB
  Wire.write(value & 0xFF);        // LSB
  Wire.endTransmission();
}

// Function to read a 16-bit value from a TMP117 register
uint16_t tmp117_readRegister(uint8_t reg)
{
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(reg);             // Register address
  Wire.endTransmission(false); // Restart condition

  Wire.requestFrom(TMP117_ADDR, 2);                  // Request 2 bytes
  uint16_t value = (Wire.read() << 8) | Wire.read(); // MSB | LSB
  return value;
}

// Function to read the temperature in Celsius
float tmp117_readTemperature(void)
{
  uint16_t raw_temp = tmp117_readRegister(0);
  return raw_temp * 0.0078125; // TMP117 temperature resolution
}

// Function to init the temperature sensor tmp117
void tmp117_init(void)
{
  tmp117_writeRegister(TMP117_CONFIG, 0);
}