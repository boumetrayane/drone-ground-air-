const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>Drone Control</title>
  <style>
    .joystick {
      width: 100px;
      height: 100px;
      background-color: #ccc;
      border-radius: 50%;
      position: relative;
      margin: 20px;
    }
    .joystick-inner {
      width: 50px;
      height: 50px;
      background-color: #333;
      border-radius: 50%;
      position: absolute;
      top: 25px;
      left: 25px;
    }
    .button {
      padding: 10px 20px;
      margin: 10px;
      font-size: 16px;
    }
  </style>
</head>
<body>
  <h1>Drone Control</h1>
  <div>
    <div class="joystick" id="joystickLeft">
      <div class="joystick-inner"></div>
    </div>
    <div class="joystick" id="joystickRight">
      <div class="joystick-inner"></div>
    </div>
  </div>
  <button class="button" onclick="toggleRoadMode()">Road Mode</button>

  <script>
    let roadMode = false;

    function toggleRoadMode() {
      roadMode = !roadMode;
      alert(roadMode ? "Road Mode ON" : "Road Mode OFF");
    }

    function sendCommand(channel, value) {
      fetch(`/control?channel=${channel}&value=${value}`)
        .then(response => response.text())
        .then(data => console.log(data));
    }

    joystickLeft.addEventListener('mousemove', (e) => {
      const rect = joystickLeft.getBoundingClientRect();
      const x = e.clientX - rect.left - 50;
      const y = e.clientY - rect.top - 50;
      const throttle = Math.round((y / 50) * 500 + 1500);
      const roll = Math.round((x / 50) * 500 + 1500);
      sendCommand('throttle', throttle);
      sendCommand('roll', roll);
    });

    joystickRight.addEventListener('mousemove', (e) => {
      const rect = joystickRight.getBoundingClientRect();
      const x = e.clientX - rect.left - 50;
      const y = e.clientY - rect.top - 50;
      const pitch = Math.round((y / 50) * 500 + 1500);
      const yaw = Math.round((x / 50) * 500 + 1500);
      sendCommand('pitch', pitch);
      sendCommand('yaw', yaw);
    });
  </script>
</body>
</html>
)=====";