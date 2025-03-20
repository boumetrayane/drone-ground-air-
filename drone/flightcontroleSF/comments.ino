#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "web_radio-transmeter-index.h"

// Web server part
const char* ssid = "drone"; // Wi-Fi SSID
const char* password = "rayane2005"; // Wi-Fi password

WebServer server(80); // Create a web server on port 80

// Variables to store joystick values from the web page
int throttleValue = 1000;
int rollValue = 1000;
int pitchValue = 1000;
int yawValue = 1000;
bool roadMode = false;
bool emergencyStop = false;

// Flight control variables
float RollRate, RollPitch, RollYaw;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float RateCalibrationNumber;
float InputThrottle;

uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.6, PRatePitch = PRateRoll, PRateYaw = 2;
float IRateRoll = 3.5, IRatePitch = IRateRoll, IRateYaw = 12;
float DRateRoll = 0.03, DRatePitch = DRateRoll, DRateYaw = 0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2, PAnglePitch = PAngleRoll;
float IAngleRoll = 0, IAnglePitch = IAngleRoll;
float DAngleRoll = 0, DAnglePitch = DAngleRoll;

// Web server handlers
void handleRoot() {
  server.send(200, "text/html", MAIN_page);
  static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
<head>
  <title>Drone Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #2c3e50; /* Dark background */
      color: #ecf0f1; /* Light text */
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
      margin: 0;
      overflow: hidden;
    }

    h1 {
      font-size: 24px;
      margin-bottom: 20px;
      text-align: center;
    }

    .controls {
      display: flex;
      justify-content: space-around;
      width: 100%;
      max-width: 600px;
    }

    .joystick {
      width: 120px;
      height: 120px;
      background-color: #34495e; /* Darker background for joysticks */
      border-radius: 50%;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); /* Subtle shadow */
    }

    .joystick-inner {
      width: 60px;
      height: 60px;
      background-color: #e74c3c; /* Red inner circle */
      border-radius: 50%;
      position: absolute;
      cursor: pointer;
      transition: transform 0.1s ease; /* Smooth movement */
    }

    .button {
      padding: 10px 20px;
      background-color: #3498db; /* Blue button */
      border: none;
      border-radius: 5px;
      color: white;
      font-size: 16px;
      cursor: pointer;
      margin-top: 20px;
      transition: background-color 0.2s ease; /* Smooth hover effect */
    }

    .button:active {
      background-color: #2980b9; /* Darker blue when pressed */
    }

    .emergency-button {
      background-color: #e74c3c; /* Red for emergency button */
    }

    .emergency-button:active {
      background-color: #c0392b; /* Darker red when pressed */
    }

    .status {
      margin-top: 20px;
      font-size: 18px;
      text-align: center;
    }

    @media (max-width: 600px) {
      h1 {
        font-size: 20px;
      }

      .joystick {
        width: 100px;
        height: 100px;
      }

      .joystick-inner {
        width: 50px;
        height: 50px;
      }

      .button {
        padding: 8px 16px;
        font-size: 14px;
      }
    }
  </style>
</head>
<body>
  <h1>Drone Control</h1>
  <div class="controls">
    <!-- Left Joystick (Throttle and Roll) -->
    <div class="joystick" id="joystickLeft">
      <div class="joystick-inner"></div>
    </div>
    <!-- Right Joystick (Pitch and Yaw) -->
    <div class="joystick" id="joystickRight">
      <div class="joystick-inner"></div>
    </div>
  </div>
  <!-- Road Mode Button -->
  <button class="button" onclick="toggleRoadMode()">Road Mode</button>
  <!-- Emergency Stop Button -->
  <button class="button emergency-button" onclick="emergencyStop()">Emergency Stop</button>
  <!-- Status Display -->
  <div class="status">
    <p>Throttle: <span id="throttleValue">1500</span></p>
    <p>Roll: <span id="rollValue">1500</span></p>
    <p>Pitch: <span id="pitchValue">1500</span></p>
    <p>Yaw: <span id="yawValue">1500</span></p>
  </div>

  <script>
    let roadMode = false;

    // Toggle Road Mode
    function toggleRoadMode() {
      roadMode = !roadMode;
      fetch(`/control?channel=roadMode&value=${roadMode ? 1 : 0}`)
        .then(response => response.text())
        .then(data => console.log(data));
      alert(roadMode ? "Road Mode ON" : "Road Mode OFF");
    }

    // Emergency Stop
    function emergencyStop() {
      fetch(`/control?channel=emergency&value=1`)
        .then(response => response.text())
        .then(data => console.log(data));
      alert("Emergency Stop Activated");
    }

    // Send Control Commands to ESP32
    function sendCommand(channel, value) {
      fetch(`/control?channel=${channel}&value=${value}`)
        .then(response => response.text())
        .then(data => console.log(data));

      // Update status display
      if (channel === 'throttle') document.getElementById('throttleValue').textContent = value;
      if (channel === 'roll') document.getElementById('rollValue').textContent = value;
      if (channel === 'pitch') document.getElementById('pitchValue').textContent = value;
      if (channel === 'yaw') document.getElementById('yawValue').textContent = value;
    }

    // Left Joystick Event Listener (Throttle and Roll)
    const joystickLeft = document.getElementById('joystickLeft');
    joystickLeft.addEventListener('mousemove', (e) => {
      const rect = joystickLeft.getBoundingClientRect();
      const x = e.clientX - rect.left - 60; // X position relative to joystick center
      const y = e.clientY - rect.top - 60; // Y position relative to joystick center
      const throttle = Math.round((y / 60) * 500 + 1500); // Map Y to throttle (1000-2000)
      const roll = Math.round((x / 60) * 500 + 1500); // Map X to roll (1000-2000)
      sendCommand('throttle', throttle);
      sendCommand('roll', roll);
    });

    // Right Joystick Event Listener (Pitch and Yaw)
    const joystickRight = document.getElementById('joystickRight');
    joystickRight.addEventListener('mousemove', (e) => {
      const rect = joystickRight.getBoundingClientRect();
      const x = e.clientX - rect.left - 60; // X position relative to joystick center
      const y = e.clientY - rect.top - 60; // Y position relative to joystick center
      const pitch = Math.round((y / 60) * 500 + 1500); // Map Y to pitch (1000-2000)
      const yaw = Math.round((x / 60) * 500 + 1500); // Map X to yaw (1000-2000)
      sendCommand('pitch', pitch);
      sendCommand('yaw', yaw);
    });
  </script>
</body>
</html>
)rawliteral";
//Instatiation of different HTTP hander
 static esp_err_t index_handler(httpd_req_t *req){
   httpd_resp_set_type(req, "text/html");
   return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
 }
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
  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
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

void reset_pid() {
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
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
  analogWriteFrequency(17, 250);
  analogWriteFrequency(16, 250);
  analogWriteFrequency(4, 250);
  analogWriteFrequency(2, 250);
  analogWriteResolution(12);
  pinMode(17, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);

  // Initialize LoopTimer
  LoopTimer = micros(); // Add this line

}

void loop() {
  while (micros() - LoopTimer < 4000); // Wait for 4ms (250Hz)
  LoopTimer = micros();
  server.handleClient();

  // Handle incoming client requests
  server.handleClient();

  // If emergency stop is activated, cut off motor inputs
  if (emergencyStop) {
    MotorInput1 = 1000;
    MotorInput2 = 1000;
    MotorInput3 = 1000;
    MotorInput4 = 1000;
    analogWrite(17, MotorInput1);
    analogWrite(16, MotorInput2);
    analogWrite(4, MotorInput3);
    analogWrite(2, MotorInput4);
    return; // Skip the rest of the loop
  }

  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  DesiredAngleRoll = 0.10 * (rollValue - 1500);
  DesiredAnglePitch = 0.10 * (pitchValue - 1500);
  InputThrottle = throttleValue;
  DesiredRateYaw = 0.15 * (yawValue - 1500);
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  int ThrottleIdle = 1180;

  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  int ThrottleCutOff = 1000;

  if (ReceiverValue[2] < 1050) { //if the web server sends a signal under 1050ms throuth the 2nd channel (throttle) call rest_pid
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }
  analogWrite(17, MotorInput1);
  analogWrite(16, MotorInput2);
  analogWrite(4, MotorInput3);
  analogWrite(2, MotorInput4);
}
