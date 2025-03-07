#include <Wire.h>
#include <PulsePosition.h>
#include <WiFi.h>
#include <WebServer.h>
#include "web_radio-transmeter-index.h"

// Web server part
const char* ssid = "drone";       // Wi-Fi SSID
const char* password = "rayane2005"; // Wi-Fi password

WebServer server(80); // Create a web server on port 80

// MPU and receiver part
PulsePositionInput espReceiverInput(RISING); // Initialize PWM input for receiver

float ReceiverValue[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Array to store receiver channel values
int ChannelNumber = 0; // Number of available channels

float RollRate, RollPitch, RollYaw; // Roll, pitch, and yaw rates
float RateRoll, RatePitch, RateYaw; // Raw gyro rates
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // Calibration values for gyro rates
int RateCalibrationNumber; // Counter for calibration loop
float inputThrottle; // Throttle input from receiver or web interface

// Road Mode variables
bool roadMode = false; // Toggle between Drone Mode and Road Mode

// ESC and Servo pins
#define ESC1_PIN 12 // ESC1 (Left Front)
#define ESC2_PIN 13 // ESC2 (Right Front)
#define ESC3_PIN 14 // ESC3 (Left Rear)
#define ESC4_PIN 15 // ESC4 (Right Rear)
#define SERVO1_PIN 16 // Servo 1
#define SERVO2_PIN 17 // Servo 2
#define SERVO3_PIN 18 // Servo 3

PulsePositionOutput escOutput; // Initialize PWM output for ESCs

void gyro_signals() {
  Wire.beginTransmission(0x68); // Start communication with MPU6050
  Wire.write(0x1A); // Activate the register with the filters
  Wire.write(0x05); // Set low-pass filter to 5Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Activate sensitivity scale factor setting
  Wire.write(0x1B); // Register address to adjust the scale factor
  Wire.write(0x08); // Set sensitivity to 65.5 LSB/°/s (0x08 in hex)
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Import measurement values from the dedicated registers
  Wire.write(0x43); // Register address for gyro data (X, Y, Z)
  Wire.endTransmission();

  Wire.requestFrom(0x43, 6); // Request 6 bytes (2 bytes per axis)
  int16_t GyroX = Wire.read() << 8 | Wire.read(); // Combine high and low bytes for X-axis
  int16_t GyroY = Wire.read() << 8 | Wire.read(); // Combine high and low bytes for Y-axis
  int16_t GyroZ = Wire.read() << 8 | Wire.read(); // Combine high and low bytes for Z-axis

  RateRoll = (float)GyroX / 65.5; // Convert raw data to degrees/second
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void handleRoot() {
  server.send(200, "text/html", MAIN_page); // Serve the HTML page
}

void handleControl() {
  if (server.hasArg("channel") && server.hasArg("value")) {
    String channel = server.arg("channel"); // Get the control channel (throttle, roll, pitch, yaw)
    int value = server.arg("value").toInt(); // Get the control value

    if (channel == "throttle") {
      inputThrottle = value; // Update throttle value
    } else if (channel == "roll") {
      RollRate = value; // Update roll value
    } else if (channel == "pitch") {
      RollPitch = value; // Update pitch value
    } else if (channel == "yaw") {
      RollYaw = value; // Update yaw value
    } else if (channel == "roadMode") {
      roadMode = (value == 1); // Toggle Road Mode
    }

    server.send(200, "text/plain", "Command received"); // Send response to client
  }
}

void setup() {
  Serial.begin(57600); // Initialize serial communication at 57600 baud
  pinMode(13, OUTPUT); // Set digital pin 13 as an output
  digitalWrite(13, HIGH); // Turn on pin 13 (e.g., for an LED indicator)
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  Wire.begin(); // Initialize I2C communication
  delay(250); // Give MPU6050 time to start

  Wire.beginTransmission(0x68); // Activate power management
  Wire.write(0x6B); // Power management register address
  Wire.write(0x00); // Activate power mode (0x00 to wake up MPU6050)
  Wire.endTransmission();

  // Calibrate gyro by taking 2000 measurements
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals(); // Read gyro signals
    RateCalibrationRoll += RateRoll; // Accumulate roll calibration values
    RateCalibrationPitch += RatePitch; // Accumulate pitch calibration values
    RateCalibrationYaw += RateYaw; // Accumulate yaw calibration values
    delay(1); // Wait 1ms between measurements
  }

  // Calculate average calibration values
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Initialize ESC and Servo outputs
  escOutput.begin(ESC1_PIN); // Initialize PWM output for ESCs
  pinMode(SERVO1_PIN, OUTPUT); // Set Servo1 pin as output
  pinMode(SERVO2_PIN, OUTPUT); // Set Servo2 pin as output
  pinMode(SERVO3_PIN, OUTPUT); // Set Servo3 pin as output

  // Web server part
  WiFi.begin(ssid, password); // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot); // Handle root URL
  server.on("/control", handleControl); // Handle control commands
  server.begin(); // Start the web server
  Serial.println("HTTP server started");
}

void loop() {
  gyro_signals(); // Read gyro signals
  RateRoll -= RateCalibrationRoll; // Subtract calibration values
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Print gyro rates to Serial Monitor
  Serial.print("Roll Rate[°/s]=");
  Serial.print(RateRoll);
  Serial.print(" Pitch Rate[°/s]=");
  Serial.print(RatePitch);
  Serial.print(" Yaw Rate[°/s]=");
  Serial.print(RateYaw);
  delay(50);

  // Control ESCs and Servos
  if (roadMode) {
    // Road Mode: Tank-like controls
    int leftSpeed = inputThrottle + RollRate; // Left ESCs
    int rightSpeed = inputThrottle - RollRate; // Right ESCs

    escOutput.write(1, leftSpeed);  // ESC1 (Left Front)
    escOutput.write(2, rightSpeed); // ESC2 (Right Front)
    escOutput.write(3, leftSpeed);  // ESC3 (Left Rear)
    escOutput.write(4, rightSpeed); // ESC4 (Right Rear)
  } else {
    // Drone Mode: Normal controls
    escOutput.write(1, inputThrottle + RollRate + RollPitch + RollYaw);  // ESC1
    escOutput.write(2, inputThrottle - RollRate + RollPitch - RollYaw);  // ESC2
    escOutput.write(3, inputThrottle + RollRate - RollPitch - RollYaw);  // ESC3
    escOutput.write(4, inputThrottle - RollRate - RollPitch + RollYaw);  // ESC4
  }

  // Control Servos (example: neutral position)
  analogWrite(SERVO1_PIN, 1500); // Servo1 neutral position
  analogWrite(SERVO2_PIN, 1500); // Servo2 neutral position
  analogWrite(SERVO3_PIN, 1500); // Servo3 neutral position

  server.handleClient(); // Handle client requests
}