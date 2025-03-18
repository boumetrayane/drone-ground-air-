<!DOCTYPE html>
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