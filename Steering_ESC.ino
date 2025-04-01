#include "libraries/CircularBuffer/CircularBuffer.hpp"
#include "libraries/PID/PID_v1_impl.h"

#include "libraries/ServoInput/ServoInput_impl.h"

// degrees of neutral position
#define ENCODER_CALIBRATION_VALUE 44.82f
// experimental values
//#define CALIBRATED_ENCODER_MAX_RIGHT 29.3f
//#define CALIBRATED_ENCODER_MAX_LEFT -39.2f
// testing values
#define CALIBRATED_ENCODER_MAX_RIGHT 28.0f
#define CALIBRATED_ENCODER_MAX_LEFT -28.0f
#define CALIBRATED_ENCODER_AMPLITUDE (CALIBRATED_ENCODER_MAX_RIGHT - CALIBRATED_ENCODER_MAX_LEFT)

// ######################################## Constants ########################################

// PID Tuning
double consKp = 0.03, consKi = 0.001, consKd = 0.0010;
const double OUTPUT_LIMIT = 0.95;
const double OUTPUT_LIMIT_LOW = 0.05;

int loopPeriod = 12;

CircularBuffer<float, 2> setpoint_buffer;
CircularBuffer<float, 4> fb_buffer;
CircularBuffer<float, 2> out_buffer;

// Steering Setup
const int SteeringSignalPin = 3;   // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

const int PIN_ENC_CLK = 8; // Green Pin
const int PIN_ENC_DATA = 7; // White Pin
const int PIN_ENC_nCS = 6; // Yellow Pin
const int ENC_BIT_COUNT = 10;  // 10 Bit Mode

// ######################################## Encoder ########################################

void encoder_setup()
{
  //give some default values
  digitalWrite(PIN_ENC_CLK, HIGH);
  digitalWrite(PIN_ENC_nCS, HIGH);

  pinMode(PIN_ENC_DATA, INPUT);
  pinMode(PIN_ENC_CLK, OUTPUT);
  pinMode(PIN_ENC_nCS, OUTPUT);
}

//read in a byte of data from the digital input of the board.
unsigned long encoder_shiftIn(const int data_pin, const int clock_pin, const int cs_pin, const int bit_count)
{
  unsigned long data = 0;

  digitalWrite(cs_pin, LOW);
  for (int i = 0; i < bit_count; i++) {
    data <<= 1;
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    data |= digitalRead(data_pin);
  }
  digitalWrite(cs_pin, HIGH);
  return data;
}

float encoder_read()
{
  encoder_setup();
  // Read the same position data twice to check for errors
  unsigned long sample1 = encoder_shiftIn(PIN_ENC_DATA, PIN_ENC_CLK, PIN_ENC_nCS, ENC_BIT_COUNT);
  unsigned long sample2 = encoder_shiftIn(PIN_ENC_DATA, PIN_ENC_CLK, PIN_ENC_nCS, ENC_BIT_COUNT);
  delayMicroseconds(20);  // Clock must be high for 20 microseconds before a new sample can be taken

  if (sample1 == 0 || sample2 == 0) return -1.0;

  const unsigned long c_errorThreshold = 20;

  if (sample1 > sample2) {
    if (sample1 - sample2 > c_errorThreshold)
      return -1.0;
  }

  if (sample1 < sample2) {
    if (sample2 - sample1 > c_errorThreshold)
      return -1.0;
  }

  float v1 = ((sample1 & 0x0FFF) * 360UL) / 1024.0;
  float v2 = ((sample2 & 0x0FFF) * 360UL) / 1024.0;

  return (v1 + v2) / 2.0;
}

// ######################################## Main ########################################

double pid_Setpoint, pid_Input, pid_Output;
PID posPID(&pid_Input, &pid_Output, &pid_Setpoint, consKp, consKi, consKd, DIRECT);

bool rcInputActive = false;
bool encoderActive = false;

void setup()
{
  posPID.SetOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);
  posPID.SetSampleTime(loopPeriod);

  Serial.begin(115200);
  Serial.println("Hello!");

  delay(1);

  steering.attach();  // attaches the steering servo input interrupt

  delay(1000);
}

void loop()
{
  float encAngle360 = encoder_read();  // 0~360

  if (encAngle360 != -1.0)
  {
    Serial.print("rawEnc: ");
    Serial.print(encAngle360);
    Serial.print(" ");

    encAngle360 -= ENCODER_CALIBRATION_VALUE;

    Serial.print("calibEnc: ");
    Serial.print(encAngle360);
    Serial.print(" ");

    pid_Input = encAngle360;

    encoderActive = true;
  }
  else
  {
    encoderActive = false;
  }

  Serial.print("Encoder: ");
  Serial.print(encoderActive);
  Serial.print(" |");

  // Servo signal input

  if (steering.available())
    rcInputActive = true;

  Serial.print(" rcInputActive ");
  Serial.print(rcInputActive);

  const float setpointRange = CALIBRATED_ENCODER_AMPLITUDE;

  if (rcInputActive && encoderActive)
  {
      float rcAngle = steering.getAngle();
      Serial.print(" rcAngle: ");
      Serial.print(rcAngle);
      float rcSetpoint = (rcAngle * setpointRange / 180.0 - (setpointRange / 2));

      Serial.print(" rcSet: ");
      Serial.print(rcSetpoint);

      setpoint_buffer.push(rcSetpoint);

      float setpoint_avg = 0;
      for (unsigned char i = 0; i < setpoint_buffer.size(); i++) {
        setpoint_avg += setpoint_buffer[i];
      }
      setpoint_avg /= setpoint_buffer.size();

      pid_Setpoint = setpoint_avg;
  }
  else
  {
    pid_Setpoint = pid_Input;
  }

  posPID.SetMode(AUTOMATIC);
  posPID.Compute();

  if (!rcInputActive || !encoderActive)
    pid_Output = 0.0;

  out_buffer.push(pid_Output);

  float out_avg = 0;
  for (unsigned char i = 0; i < out_buffer.size(); i++)
  {
    out_avg += out_buffer[i];
  }
  out_avg /= out_buffer.size();

  //motor_setPWM(out_avg, SlowDecay);

  Serial.print(" pid_Setpoint: ");
  Serial.print(pid_Setpoint);
  Serial.print(" pid_Input: ");
  Serial.print(pid_Input);
  Serial.print(" pid_Output: ");
  Serial.println(pid_Output);
}
