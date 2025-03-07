#include <WiFi.h>
#include <WebServer.h>
#include <MPU6050.h>
#include <Wire.h>
#include <ESP32Servo.h>

// WiFi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Web server
WebServer server(80);

// MPU6050
MPU6050 mpu;

// PID parameters
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float error, previousError, integral, derivative, output;

// Complementary filter variables
float roll = 0, pitch = 0;
unsigned long lastTime = 0;

// Motor and servo pins
#define ESC1_PIN 12
#define ESC2_PIN 13
#define ESC3_PIN 14
#define ESC4_PIN 15
#define SERVO1_PIN 16
#define SERVO2_PIN 17
#define SERVO3_PIN 18

Servo esc1, esc2, esc3, esc4;
Servo servo1, servo2, servo3;

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

  // Attach ESCs and servos
  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

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

  // Calculate time step
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  lastTime = currentTime;

  // Calculate roll and pitch from accelerometer
  float accelRoll = atan2(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Complementary filter
  float gyroRollRate = gx / 131.0; // Convert to degrees/sec
  float gyroPitchRate = gy / 131.0;
  roll = 0.98 * (roll + gyroRollRate * dt) + 0.02 * accelRoll;
  pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accelPitch;

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

  // Send PWM signals to ESCs and servos
  esc1.writeMicroseconds(motor1);
  esc2.writeMicroseconds(motor2);
  esc3.writeMicroseconds(motor3);
  esc4.writeMicroseconds(motor4);
  servo1.write(90); // Example servo position
  servo2.write(90);
  servo3.write(90);
}