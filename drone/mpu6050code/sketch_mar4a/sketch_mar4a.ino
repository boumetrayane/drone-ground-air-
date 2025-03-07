#include <Wire.h>
#include<Wifi.h>
#include<PulsePosition.h>
#include <WebServer.h>
#include <web_radio-transmeter-index.h>

//web_server part
const char* ssid = "drone";
const char* password = "rayane2005";

WebServer server(80);

void handdelroot() {
  Server.send (200, "text/html", MAIN_page );
}

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

//mpu and receiver, radio-transmiter part
PulsePositionInput espReceiverInput(RISSING);

float ReceiverValue()=[0,0,0,0,0,0,0,0]; //number of availabale chanels
int ChanelNumber=();

float RoleRate, RolePitch, RoleYaw;
float RateRole, RatePitch, RateYaw;
float RateCalibrationRole, RateCalibrationPitch, RateCalibrationYaw; //declare sartup rate calprations
float RateCalibrationNumber; //number of variable 
float inputthroatle;

void gyro_signals(void){
  Wire.beginTransmission(0x68); //get starting code from the datasheet register table
  Wire.write(0x1A); //activate the register with the filters
  Wire.write(0x05); //activate the  low-pass filter in 1A
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //activate sensivity scale factore setting
  Wire.write(0x1B); //register adress to ajust the factor
  Wire.write(0x8); //messurement are recorded in LSByt for sensivity of 65.5 we get 00001000 in hex it's 0x08
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //import the messurment values from the dedicated registers
  Wire.write(0x43); //from x43 to x48 are the register for meassurment storing
  Wire.endTransmission();

  Wire.requestFrom(0x43,6);//request 6bytes from mpu to get the info from the register 43 to 48
  int16_t GyroX=Wire.read()<<8 | Wire.read(); //the measurment of each axis is in 2 register with 8vits each that why we request 16bits
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  RateRole =(float)GyroX/65.5; //the info is in LSbyts but we have to covert it to degrees/second
  RatePitch =(float)GyroY/65.5;
  RateYaw =(float)GyroZ/65.5;
}

void setup() {
  Serial.begin(57600); //Initializes the serial communication at a baud rate of 57600 bits per second.
  pinMode(13, OUTPUT); //Sets the digital pin 13 as an output.
  digitalWrite(13, HIGH); //
  Wire.setClock(400000); //Sets the I2C clock speed to 400kHz. This is used for communication with I2C devices like the esp32
  Wire.begin();//Initializes the I2C communication.
  delay(250);//give mpu time to start

  Wire.beginTransmission(0x68); //activate the alimentation
  Wire.write(0x6B); //write to power managment register
  Wire.write(0x00); // hex code to activate power mode and start the 
  
  for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++ ){ //we take 2000 measurment 1ms each for the clibration 
    gyro_signals();
    RateCalibrationRole+=RateRole; //we add al the mesured values to the calibration variables (don't move the sensor)
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }

  RateCalibrationRole/=2000;  //find the avrage calibration value by deviding the sum of all 2000 on 2000 to find the rate at 0
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

 //web_server part
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  server.o,("/" ;handdelroot);
  server.begin();
  Serial.println("htp server started");
}

void loop() {
  gyro_signals();
  RateRole -= RateCalibrationRole; //now we subtract the mesurments from the calibration to get the final correct values (should =~ 0)
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  Serial.print("Role Rate[°/s]="); //call the function to read the meassurments and diplay them in the serial monitor
  Serial.print(RateRole);
  Serial.print("Pitch Rate[°/s]=");
  Serial.print(RatePitch);
  Serial.print("Yaw Rate[°/s]=");
  Serial.print(RateYaw);
  delay(50);

  read_receiver();
  Serial.print("Numberofchanels:");
  Serial.print(ChannelNumber);
  Serial.print("Role [us]:");
  Serial.print(ReceivrValue[0]);
  Serial.print("Pitch [us]:");
  Serial.print(ReceivrValue[1]);
  Serial.print("Yaw [us]:");
  Serial.print(ReceivrValue[3]);
  Serial.print("throttle[us]:");
  Serial.print(ReceivrValue[2]);
  delay(50);

 //wrb_server part

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
      
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

