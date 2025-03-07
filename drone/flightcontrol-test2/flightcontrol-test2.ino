#include <WiFi.h>
#include <WebServer.h>
#include <MPU6050.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// WiFi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Web server
WebServer server(80);

// MPU6050
MPU6050 mpu;

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PID parameters
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float error, previousError, integral, derivative, output;

// HTML page for PID tuning
const char* htmlPage = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>PID Tuning</title>
</head>
<body>
  <h1>PID Tuning</h1>
  <form action="/update" method="POST">
    <label for="kp">Kp:</label>
    <input type="text" id="kp" name="kp" value="%Kp%"><br><br>
    <label for="ki">Ki:</label>
    <input type="text" id="ki" name="ki" value="%Ki%"><br><br>
    <label for="kd">Kd:</label>
    <input type="text" id="kd" name="kd" value="%Kd%"><br><br>
    <input type="submit" value="Update PID">
  </form>
</body>
</html>
)=====";

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for ESCs and servos

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Start web server
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient(); // Handle client requests

  // Read MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate roll and pitch
  float roll = atan2(ay, az) * RAD_TO_DEG;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // PID Control
  PID_Control(0, roll); // Setpoint = 0 (level)
  float throttle = 1500; // Neutral throttle
  controlMotors(throttle, output, 0); // Adjust motors based on PID output
}

void handleRoot() {
  // Send HTML page with current PID values
  String page = htmlPage;
  page.replace("%Kp%", String(Kp));
  page.replace("%Ki%", String(Ki));
  page.replace("%Kd%", String(Kd));
  server.send(200, "text/html", page);
}

void handleUpdate() {
  // Update PID values from form submission
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();

  // Send response
  server.send(200, "text/plain", "PID Updated: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
}

void PID_Control(float setpoint, float input) {
  error = setpoint - input;
  integral += error;
  derivative = error - previousError;
  output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
}

void controlMotors(float throttle, float rollOutput, float pitchOutput) {
  int motor1 = throttle + rollOutput + pitchOutput;
  int motor2 = throttle - rollOutput + pitchOutput;
  int motor3 = throttle + rollOutput - pitchOutput;
  int motor4 = throttle - rollOutput - pitchOutput;

  motor1 = constrain(motor1, 1000, 2000);
  motor2 = constrain(motor2, 1000, 2000);
  motor3 = constrain(motor3, 1000, 2000);
  motor4 = constrain(motor4, 1000, 2000);

  // Send PWM signals to ESCs and servos via PCA9685
  pwm.setPWM(0, 0, pulseWidth(motor1)); // ESC1
  pwm.setPWM(1, 0, pulseWidth(motor2)); // ESC2
  pwm.setPWM(2, 0, pulseWidth(motor3)); // ESC3
  pwm.setPWM(3, 0, pulseWidth(motor4)); // ESC4
  pwm.setPWM(4, 0, pulseWidth(1500));   // Servo1 (neutral)
  pwm.setPWM(5, 0, pulseWidth(1500));   // Servo2 (neutral)
  pwm.setPWM(6, 0, pulseWidth(1500));   // Servo3 (neutral)
}

int pulseWidth(int microseconds) {
  return map(microseconds, 1000, 2000, 0, 4095); // Convert to PCA9685 resolution
}