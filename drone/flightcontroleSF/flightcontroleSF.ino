#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "web_radio-transmeter-index.h"

// Web server part
const char* ssid = "drone"; // Wi-Fi SSID
const char* password = "rayane2005"; // Wi-Fi password

WebServer server(80); // Create a web server on port 80

// Variables to store joystick values from the web page
int throttleValue = 1050;
int rollValue = 1050;
int pitchValue = 1050;
int yawValue = 1050;
bool roadMode = false;
bool emergencyStop = false;

// Flight control variables
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch, InputYaw;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// PID and Kalman filter variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

// PID gains
float PRateRoll = 0.6, PRatePitch = PRateRoll, PRateYaw = 2;
float IRateRoll = 3.5, IRatePitch = IRateRoll, IRateYaw = 12;
float DRateRoll = 0.03, DRatePitch = DRateRoll, DRateYaw = 0;

// Web server handlers
void handleRoot() {
  server.send(200, "text/html", MAIN_page);
}

void handleControl() {
  String channel = server.arg("channel");
  int value = server.arg("value").toInt();

  if (channel == "throttle") throttleValue = value;
  else if (channel == "roll") rollValue = value;
  else if (channel == "pitch") pitchValue = value;
  else if (channel == "yaw") yawValue = value;
  else if (channel == "roadMode") roadMode = (value == 1);
  else if (channel == "emergency") emergencyStop = (value == 1);

  server.send(200, "text/plain", "OK");
}

// Kalman filter function
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// Read gyro and accelerometer data
void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

// PID equation
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm, float* PIDReturn) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize web server
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.begin();
  Serial.println("HTTP server started");

  // Initialize I2C and MPU6050
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate gyro
  for (int i = 0; i < 2000; i++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Initialize motor pins
  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteResolution(12);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {
  // Handle incoming client requests
  server.handleClient();

  // If emergency stop is activated, cut off motor inputs
  if (emergencyStop) {
    MotorInput1 = 1000;
    MotorInput2 = 1000;
    MotorInput3 = 1000;
    MotorInput4 = 1000;
    analogWrite(1, MotorInput1);
    analogWrite(2, MotorInput2);
    analogWrite(3, MotorInput3);
    analogWrite(4, MotorInput4);
    return; // Skip the rest of the loop
  }

  // Read gyro data
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Kalman filter for roll and pitch
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, 0);
  KalmanAngleRoll = Kalman1DOutput[0];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, 0);
  KalmanAnglePitch = Kalman1DOutput[0];

  // Use joystick values from the web page
  DesiredRateRoll = 0.10 * (rollValue - 1500);
  DesiredRatePitch = 0.10 * (pitchValue - 1500);
  DesiredRateYaw = 0.15 * (yawValue - 1500);

  // PID control for roll, pitch, and yaw
  float PIDReturn[3];
  pid_equation(DesiredRateRoll - RateRoll, PRateRoll, IRateRoll, DRateRoll, ErrorRateRoll, 0, PIDReturn);
  InputRoll = PIDReturn[0];
  ErrorRateRoll = PIDReturn[1];
  pid_equation(DesiredRatePitch - RatePitch, PRatePitch, IRatePitch, DRatePitch, ErrorRatePitch, 0, PIDReturn);
  InputPitch = PIDReturn[0];
  ErrorRatePitch = PIDReturn[1];
  pid_equation(DesiredRateYaw - RateYaw, PRateYaw, IRateYaw, DRateYaw, ErrorRateYaw, 0, PIDReturn);
  InputYaw = PIDReturn[0];
  ErrorRateYaw = PIDReturn[1];

  // Calculate motor inputs
  MotorInput1 = 1.024 * (throttleValue - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (throttleValue - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (throttleValue + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (throttleValue + InputRoll - InputPitch + InputYaw);

  // Constrain motor inputs to safe limits
  MotorInput1 = constrain(MotorInput1, 1000, 2000);
  MotorInput2 = constrain(MotorInput2, 1000, 2000);
  MotorInput3 = constrain(MotorInput3, 1000, 2000);
  MotorInput4 = constrain(MotorInput4, 1000, 2000);

  // Apply motor inputs
  analogWrite(1, MotorInput1);
  analogWrite(2, MotorInput2);
  analogWrite(3, MotorInput3);
  analogWrite(4, MotorInput4);
}