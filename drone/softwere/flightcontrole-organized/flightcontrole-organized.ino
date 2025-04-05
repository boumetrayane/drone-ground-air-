#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* ssid = "drone";
const char* password = "rayane2005";
WebServer server(80);

static const char PROGMEM FLIGHT_HTML[] = R"rawliteral(
<html>
    <head>
        <title>Drone Control - Flight Mode</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
        <style>
            body {
                font-family: Arial, sans-serif;
                background-color: #2c3e50;
                color: #ecf0f1;
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
                background-color: #34495e;
                border-radius: 50%;
                position: relative;
                display: flex;
                align-items: center;
                justify-content: center;
                box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
            }

            .throttle-line {
                width: 30px;
                height: 120px;
                background-color: rgba(255, 255, 255, 0.6);
                position: absolute;
                left: 50%;
                transform: translateX(-50%);
            }

            .joystick-inner {
                width: 30px;
                height: 30px;
                background-color: #e74c3c;
                border-radius: 50%;
                position: absolute;
                cursor: pointer;
                transition: transform 0.1s ease;
            }

            .right-inner {
                width: 60px;
                height: 60px;
                background-color: #e74c3c;
                border-radius: 50%;
                position: absolute;
                cursor: pointer;
                transition: transform 0.1s ease;
            }

            .button {
                padding: 10px 20px;
                background-color: #3498db;
                border: none;
                border-radius: 5px;
                color: white;
                font-size: 16px;
                cursor: pointer;
                margin-top: 20px;
                transition: background-color 0.2s ease;
            }

            .button:active {
                background-color: #2980b9;
            }

            .emergency-button {
                background-color: #e74c3c;
            }

            .emergency-button:active {
                background-color: #c0392b;
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
                .throttle-line {
                    width: 25px;
                    height: 100px;
                }
                .joystick-inner {
                    width: 25px;
                    height: 25px;
                }
                .right-inner {
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
        <h1>Flight Mode</h1>
        <div class="controls">
            <div class="joystick" id="joystickLeft">
                <div class="throttle-line"></div>
                <div class="joystick-inner" id="joystickLeftInner"></div>
            </div>
            <div class="joystick" id="joystickRight">
                <div class="right-inner" id="joystickRightInner"></div>
            </div>
        </div>
        <button class="button" onclick="switchToRoadMode()">Switch to Road Mode</button>
        <button class="button emergency-button" onclick="emergencyStop()">Emergency Stop</button>
        <div class="status">
            <p>Throttle: <span id="throttleValue">1000</span></p>
            <p>Roll: <span id="rollValue">1500</span></p>
            <p>Pitch: <span id="pitchValue">1500</span></p>
            <p>Yaw: <span id="yawValue">1500</span></p>
        </div>

        <script>
            let lastThrottle = 1000;
            let lastThrottleX = 0;
            let lastThrottleY = 15;

            window.onload = () => {
                sendCommand('throttle', 1000);
                document.getElementById('joystickLeftInner').style.transform = 'translate(0px, 15px)';
            };

            function switchToRoadMode() {
                fetch('/control?channel=roadMode&value=1')
                    .then(response => response.text())
                    .then(data => {
                        console.log(`roadMode: 1 -> ${data}`);
                        window.location.href = '/road';
                    });
            }

            function emergencyStop() {
                fetch('/control?channel=emergency&value=1')
                    .then(response => response.text())
                    .then(data => console.log(`emergency: 1 -> ${data}`));
                alert("Emergency Stop Activated");
            }

            function sendCommand(channel, value) {
                fetch(`/control?channel=${channel}&value=${value}`)
                    .then(response => response.text())
                    .then(data => console.log(`${channel}: ${value} -> ${data}`));
                
                if (channel === 'throttle') document.getElementById('throttleValue').textContent = value;
                if (channel === 'roll') document.getElementById('rollValue').textContent = value;
                if (channel === 'pitch') document.getElementById('pitchValue').textContent = value;
                if (channel === 'yaw') document.getElementById('yawValue').textContent = value;
            }

            const joystickLeft = document.getElementById('joystickLeft');
            const joystickLeftInner = document.getElementById('joystickLeftInner');
            const joystickRight = document.getElementById('joystickRight');
            const joystickRightInner = document.getElementById('joystickRightInner');

            // Shared movement handler for left joystick (throttle/yaw)
            function handleLeftMove(event) {
                event.preventDefault();
                const rect = joystickLeft.getBoundingClientRect();
                const touch = event.type.includes('touch') ? event.touches[0] : event;
                const x = touch.clientX - rect.left - 60;
                const y = touch.clientY - rect.top - 60;

                // Restrict inner circle to outer circle's 60px radius
                const distance = Math.sqrt(x * x + y * y);
                const maxRadius = 60;
                let moveX = x;
                let moveY = y;
                if (distance > maxRadius) {
                    moveX = (x / distance) * maxRadius;
                    moveY = (y / distance) * maxRadius;
                }

                // Throttle always based on current Y-position within outer circle
                const throttle = Math.round((-moveY / 60) * 500 + 1500);

                // Yaw dead zone as vertical line (30px width)
                const yawDeadZoneWidth = 15; // Half-width of dead zone (30px total)
                let yaw;
                if (Math.abs(moveX) <= yawDeadZoneWidth) {
                    yaw = 1500;
                } else {
                    // Lock visual Y to last throttle position outside dead zone, but yaw uses X
                    moveY = lastThrottleY;
                    const lockedDistance = Math.sqrt(moveX * moveX + moveY * moveY);
                    if (lockedDistance > maxRadius) {
                        moveX = (moveX / lockedDistance) * maxRadius;
                        moveY = (moveY / lockedDistance) * maxRadius;
                    }
                    yaw = calculateYaw(moveX);
                }

                joystickLeftInner.style.transform = `translate(${moveX}px, ${moveY}px)`;
                sendCommand('throttle', throttle);
                sendCommand('yaw', yaw);

                // Update last throttle values only when inside dead zone
                if (Math.abs(moveX) <= yawDeadZoneWidth) {
                    lastThrottle = throttle;
                    lastThrottleX = moveX;
                    lastThrottleY = moveY;
                }
            }

            // Shared movement handler for right joystick (pitch/roll)
            function handleRightMove(event) {
                event.preventDefault();
                const rect = joystickRight.getBoundingClientRect();
                const touch = event.type.includes('touch') ? event.touches[0] : event;
                const x = touch.clientX - rect.left - 60;
                const y = touch.clientY - rect.top - 60;
                const pitch = Math.round((y / 60) * 500 + 1500);
                const roll = Math.round((x / 60) * 500 + 1500);

                const distance = Math.sqrt(x * x + y * y);
                const maxRadius = 30;
                let moveX = x;
                let moveY = y;
                if (distance > maxRadius) {
                    moveX = (x / distance) * maxRadius;
                    moveY = (y / distance) * maxRadius;
                }

                joystickRightInner.style.transform = `translate(${moveX}px, ${moveY}px)`;
                sendCommand('pitch', pitch);
                sendCommand('roll', roll);
            }

            function calculateYaw(x) {
                const yawDeadZoneWidth = 15;
                const yawRange = (x > 0) ? (2000 - 1501) : (1499 - 1000); // 499µs on each side
                const xBeyondDeadZone = Math.abs(x) - yawDeadZoneWidth; // Distance past dead zone
                const maxXBeyond = 60 - yawDeadZoneWidth; // Max distance past dead zone (45px)
                const yawIncrement = (xBeyondDeadZone / maxXBeyond) * yawRange;
                return (x > 0) ? Math.round(1501 + yawIncrement) : Math.round(1499 - yawIncrement);
            }

            // Reset handlers
            function resetLeftJoystick() {
                joystickLeftInner.style.transform = `translate(0px, ${lastThrottleY}px)`;
                sendCommand('yaw', 1500);
                sendCommand('throttle', lastThrottle); // Persist throttle
            }

            function resetRightJoystick() {
                joystickRightInner.style.transform = 'translate(0px, 0px)';
                sendCommand('pitch', 1500);
                sendCommand('roll', 1500);
            }

            // Event listeners
            joystickLeft.addEventListener('mousemove', handleLeftMove);
            joystickLeft.addEventListener('mouseleave', resetLeftJoystick);
            joystickRight.addEventListener('mousemove', handleRightMove);
            joystickRight.addEventListener('mouseleave', resetRightJoystick);

            // Touch events
            joystickLeft.addEventListener('touchstart', (e) => e.preventDefault());
            joystickLeft.addEventListener('touchmove', handleLeftMove);
            joystickLeft.addEventListener('touchend', resetLeftJoystick);

            joystickRight.addEventListener('touchstart', (e) => e.preventDefault());
            joystickRight.addEventListener('touchmove', handleRightMove);
            joystickRight.addEventListener('touchend', resetRightJoystick);
        </script>
    </body>
</html>
)rawliteral";

static const char PROGMEM ROAD_HTML[] = R"rawliteral(
<html>
    <head>
        <title>Drone Control - Road Mode</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
        <style>
            body {
                font-family: Arial, sans-serif;
                background-color: #2c3e50;
                color: #ecf0f1;
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

            .button-group {
                display: flex;
                flex-direction: column;
                align-items: center;
            }

            .horizontal-group {
                display: flex;
                flex-direction: row;
                align-items: center;
            }

            .button {
                padding: 15px 30px;
                background-color: #3498db;
                border: none;
                border-radius: 5px;
                color: white;
                font-size: 16px;
                cursor: pointer;
                margin: 10px;
                transition: background-color 0.2s ease;
            }

            .button:active {
                background-color: #2980b9;
            }

            .emergency-button {
                background-color: #e74c3c;
            }

            .emergency-button:active {
                background-color: #c0392b;
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
                .button {
                    padding: 10px 20px;
                    font-size: 14px;
                }
            }
        </style>
    </head>
    <body>
        <h1>Road Mode</h1>
        <div class="controls">
            <div class="button-group">
                <button class="button" id="upButton">Up</button>
                <button class="button" id="downButton">Down</button>
            </div>
            <div class="horizontal-group">
                <button class="button" id="leftButton">Left</button>
                <button class="button" id="rightButton">Right</button>
            </div>
        </div>
        <button class="button" onclick="switchToFlightMode()">Switch to Flight Mode</button>
        <button class="button emergency-button" onclick="emergencyStop()">Emergency Stop</button>
        <div class="status">
            <p>Acceleration: <span id="accelValue">1500</span></p>
            <p>Left Accel: <span id="leftAccelValue">1500</span></p>
            <p>Right Accel: <span id="rightAccelValue">1500</span></p>
        </div>

        <script>
            let accel = 1500;
            let leftAccel = 1500;
            let rightAccel = 1500;
            let accelInterval = null;
            let steerInterval = null;

            window.onload = () => {
                fetch('/control?channel=roadMode&value=1');
            };

            function switchToFlightMode() {
                fetch('/control?channel=roadMode&value=0')
                    .then(response => response.text())
                    .then(data => {
                        console.log(`roadMode: 0 -> ${data}`);
                        window.location.href = '/';
                    });
            }

            function emergencyStop() {
                fetch('/control?channel=emergency&value=1')
                    .then(response => response.text())
                    .then(data => console.log(`emergency: 1 -> ${data}`));
                alert("Emergency Stop Activated");
            }

            function sendCommand(channel, value) {
                fetch(`/control?channel=${channel}&value=${value}`)
                    .then(response => response.text())
                    .then(data => console.log(`${channel}: ${value} -> ${data}`));
                
                if (channel === 'acceleration' || channel === 'deceleration') {
                    document.getElementById('accelValue').textContent = value;
                }
                if (channel === 'leftaccel') {
                    document.getElementById('leftAccelValue').textContent = value;
                }
                if (channel === 'rightaccel') {
                    document.getElementById('rightAccelValue').textContent = value;
                }
            }

            function logarithmicScale(start, end, t) {
                return Math.round(start + (end - start) * (1 - Math.pow(1 - t, 2)));
            }

            function startAccel(direction) {
                if (accelInterval) clearInterval(accelInterval);
                let t = 0;
                accelInterval = setInterval(() => {
                    t += 0.05;
                    if (t > 1) t = 1;
                    if (direction === 'up') {
                        accel = logarithmicScale(1500, 2000, t);
                        sendCommand('acceleration', accel);
                    } else if (direction === 'down') {
                        accel = logarithmicScale(1500, 1000, t);
                        sendCommand('deceleration', accel);
                    }
                }, 50);
            }

            function stopAccel(direction) {
                if (accelInterval) clearInterval(accelInterval);
                let t = 0;
                accelInterval = setInterval(() => {
                    t += 0.05;
                    if (t > 1) {
                        clearInterval(accelInterval);
                        accel = 1500;
                        sendCommand('acceleration', accel);
                        return;
                    }
                    if (direction === 'up') {
                        accel = logarithmicScale(accel, 1500, t);
                        sendCommand('acceleration', accel);
                    } else if (direction === 'down') {
                        accel = logarithmicScale(accel, 1500, t);
                        sendCommand('deceleration', accel);
                    }
                }, 50);
            }

            function startSteer(direction) {
                if (steerInterval) clearInterval(steerInterval);
                let t = 0;
                steerInterval = setInterval(() => {
                    t += 0.05;
                    if (t > 1) t = 1;
                    if (direction === 'left') {
                        leftAccel = logarithmicScale(1500, 2000, t);
                        rightAccel = 1500;
                        sendCommand('leftaccel', leftAccel);
                        sendCommand('rightaccel', rightAccel);
                    } else {
                        rightAccel = logarithmicScale(1500, 2000, t);
                        leftAccel = 1500;
                        sendCommand('rightaccel', rightAccel);
                        sendCommand('leftaccel', leftAccel);
                    }
                }, 50);
            }

            function stopSteer() {
                if (steerInterval) clearInterval(steerInterval);
                leftAccel = 1500;
                rightAccel = 1500;
                sendCommand('leftaccel', leftAccel);
                sendCommand('rightaccel', rightAccel);
            }

            // Event Listeners
            document.getElementById('upButton').addEventListener('mousedown', () => startAccel('up'));
            document.getElementById('downButton').addEventListener('mousedown', () => startAccel('down'));
            document.getElementById('leftButton').addEventListener('mousedown', () => startSteer('left'));
            document.getElementById('rightButton').addEventListener('mousedown', () => startSteer('right'));

            document.getElementById('upButton').addEventListener('mouseup', () => stopAccel('up'));
            document.getElementById('downButton').addEventListener('mouseup', () => stopAccel('down'));
            document.getElementById('leftButton').addEventListener('mouseup', stopSteer);
            document.getElementById('rightButton').addEventListener('mouseup', stopSteer);
        </script>
    </body>
</html>
)rawliteral";

// Updated variables
int throttleValue = 1000;
int rollValue = 1500;
int pitchValue = 1500;
int yawValue = 1500;
int acceleration = 1500;  // New for road mode
int deceleration = 1500;  // New for road mode
int leftaccel = 1500;     // New for road mode
int rightaccel = 1500;    // New for road mode
bool roadMode = false;
bool emergencyStop = false;
// ... other variables
Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 17;
const int mot2_pin = 16;
const int mot3_pin = 4;
const int mot4_pin = 2;
const int ESCfreq = 500; // Increased to 500Hz

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float InputThrottle;
float AccXCalibration, AccYCalibration, AccZCalibration;
//float AccXCalibration = 0.03;  // value from calibration
//float AccYCalibration = -0.02; // value from calibration
//float AccZCalibration = -0.95; // value from calibration (AccZ - 1g) change after calibration
uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.625, PRatePitch = PRateRoll, PRateYaw = 4; // Updated from found code
float IRateRoll = 2.1, IRatePitch = IRateRoll, IRateYaw = 3;
float DRateRoll = 0.0088, DRatePitch = DRateRoll, DRateYaw = 0;
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
float IAngleRoll = 0.5, IAnglePitch = IAngleRoll; // Updated from found code
float DAngleRoll = 0.007, DAnglePitch = DAngleRoll; // Updated from found code
float complementaryAngleRoll = 0.0f, complementaryAnglePitch = 0.0f;

void handleRoot() {
  server.send(200, "text/html", FLIGHT_HTML);
}

void handleRoad() {
  server.send(200, "text/html", ROAD_HTML);
}

void handleControl() {
  String channel = server.arg("channel");
  int value = server.arg("value").toInt();
  if (channel == "throttle") {
    throttleValue = value;
    Serial.printf("Received Throttle: %d\n", throttleValue);
  }
  else if (channel == "roll") {
    rollValue = value;
    Serial.printf("Received Roll: %d\n", rollValue);
  }
  else if (channel == "pitch") {
    pitchValue = value;
    Serial.printf("Received Pitch: %d\n", pitchValue);
  }
  else if (channel == "yaw") {
    yawValue = value;
    Serial.printf("Received Yaw: %d\n", yawValue);
  }
  else if (channel == "acceleration") {
    acceleration = value;
    Serial.printf("Received Acceleration: %d\n", acceleration);
  }
  else if (channel == "deceleration") {
    deceleration = value;
    Serial.printf("Received Deceleration: %d\n", deceleration);
  }
  else if (channel == "leftaccel") {
    leftaccel = value;
    Serial.printf("Received Left Accel: %d\n", leftaccel);
  }
  else if (channel == "rightaccel") {
    rightaccel = value;
    Serial.printf("Received Right Accel: %d\n", rightaccel);
  }
  else if (channel == "roadMode") {
    roadMode = (value == 1);
    Serial.printf("Road Mode: %d\n", roadMode);
  }
  else if (channel == "emergency") {
    emergencyStop = (value == 1);
    Serial.println("Emergency Stop Triggered");
  }
  server.send(200, "text/plain", "OK");
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 I2C error at 0x1A!");
    // Optionally set emergencyStop = true; (uncomment if desired)
  }
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
  AccX = (float)AccXLSB / 4096; //- 0.05; add in case of hardcoded calibration offset
  AccY = (float)AccYLSB / 4096; //+ 0.01; ""
  AccZ = (float)AccZLSB / 4096; //- 0.11; ""
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}

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
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/road", handleRoad);
  server.on("/control", handleControl);
  server.begin();
  Serial.println("HTTP server started");

  throttleValue = 1000;

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("Calibrating MPU6050... Place quadcopter flat and press Enter.");
  //while (!Serial.available()) {}  // Wait for user to press Enter
  //Serial.read();  // Clear the input buffer
  Serial.println("Starting calibration...");
for (int i = 0; i < 2000; i++) {
  gyro_signals();
  RateCalibrationRoll += RateRoll;
  RateCalibrationPitch += RatePitch;
  RateCalibrationYaw += RateYaw;
  AccXCalibration += AccX;
  AccYCalibration += AccY;
  AccZCalibration += AccZ;
  delay(1);
}
RateCalibrationRoll /= 2000;
RateCalibrationPitch /= 2000;
RateCalibrationYaw /= 2000;
AccXCalibration /= 2000;
AccYCalibration /= 2000;
AccZCalibration /= 2000;
AccZCalibration -= 1;  // Subtract 1g from Z-axis (assuming flat orientation)

Serial.println("Calibration complete:");
Serial.print("RateCalibrationRoll="); Serial.print(RateCalibrationRoll); Serial.println(";");
Serial.print("RateCalibrationPitch="); Serial.print(RateCalibrationPitch); Serial.println(";");
Serial.print("RateCalibrationYaw="); Serial.print(RateCalibrationYaw); Serial.println(";");
Serial.print("AccXCalibration="); Serial.print(AccXCalibration); Serial.println(";");
Serial.print("AccYCalibration="); Serial.print(AccYCalibration); Serial.println(";");
Serial.print("AccZCalibration="); Serial.print(AccZCalibration); Serial.println(";");

//gyro calibration only from old code
//  for (int i = 0; i < 2000; i++) {
//    gyro_signals();
//    RateCalibrationRoll += RateRoll;
//    RateCalibrationPitch += RatePitch;
//   RateCalibrationYaw += RateYaw;
//    delay(1);
//  }
//  RateCalibrationRoll /= 2000;
//  RateCalibrationPitch /= 2000;
//  RateCalibrationYaw /= 2000;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
  mot3.setPeriodHertz(ESCfreq);
  mot4.setPeriodHertz(ESCfreq);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  delay(2000);

  LoopTimer = micros();
}

void loop() {
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
  server.handleClient();

  if (emergencyStop) {
    MotorInput1 = 1000;
    MotorInput2 = 1000;
    MotorInput3 = 1000;
    MotorInput4 = 1000;
    mot1.writeMicroseconds(MotorInput1);
    mot2.writeMicroseconds(MotorInput2);
    mot3.writeMicroseconds(MotorInput3);
    mot4.writeMicroseconds(MotorInput4);
    return;
  }

  if (roadMode) {
    // Road mode: Use acceleration/deceleration and leftaccel/rightaccel
    if (acceleration != 1500) {
      MotorInput1 = acceleration;
      MotorInput2 = acceleration;
      MotorInput3 = acceleration;
      MotorInput4 = acceleration;
    } else if (deceleration != 1500) {
      MotorInput1 = deceleration;
      MotorInput2 = deceleration;
      MotorInput3 = deceleration;
      MotorInput4 = deceleration;
    } else {
      MotorInput1 = leftaccel;
      MotorInput2 = leftaccel;
      MotorInput3 = rightaccel;
      MotorInput4 = rightaccel;
    }
  } else {
    // Flight mode: Existing logic
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    AccX -= AccXCalibration; // remove for hardcoded calibration offset
    AccY -= AccYCalibration; // ""
    AccZ -= AccZCalibration; // ""

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    DesiredAngleRoll = 0.10 * (rollValue - 1500);
    DesiredAnglePitch = 0.10 * (pitchValue - 1500);
    DesiredAngleRoll = (DesiredAngleRoll > 20) ? 20 : ((DesiredAngleRoll < -20) ? -20 : DesiredAngleRoll);
    DesiredAnglePitch = (DesiredAnglePitch > 20) ? 20 : ((DesiredAnglePitch < -20) ? -20 : DesiredAnglePitch);
    InputThrottle = throttleValue;
    DesiredRateYaw = 0.15 * (yawValue - 1500);

    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll, PIDReturn);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch, PIDReturn);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, PIDReturn);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, PIDReturn);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw, PIDReturn);
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

    int ThrottleIdle = 1170;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    int ThrottleCutOff = 1000;
    if (throttleValue < 1050) {
      MotorInput1 = ThrottleCutOff;
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff;
      MotorInput4 = ThrottleCutOff;
      reset_pid();
    }
  }

  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  Serial.printf("Motor Outputs - M1: %.0f, M2: %.0f, M3: %.0f, M4: %.0f\n",
                 MotorInput1, MotorInput2, MotorInput3, MotorInput4);

  Serial.printf("Acc: X=%.3f, Y=%.3f, Z=%.3f | Gyro: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n",
                 AccX, AccY, AccZ, RateRoll, RatePitch, RateYaw);
}