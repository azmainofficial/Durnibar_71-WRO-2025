#include <Servo.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>

#define servoPin 11
#define MotorF 6
#define MotorB 5
#define BUTTON1 13
#define GreenLedPin A3
#define RedLedPin A2
#define Display_I2C_ADDRESS 0x3C

// Motor control variables
int motor_forward_speed = 90;
int servo_angle = 110;

// Sensor objects
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;
Servo myservo;
SSD1306AsciiWire oled;

// Timing for sensor readings
unsigned long previousMillis = 0;
const long interval = 100;  // Send sensor data every 100ms
bool start = 0;

void setup() {
  Serial.begin(9600);   // Arduino and Jetson should use the same baud rate

  Wire.begin();
  Wire.setClock(400000L);

  // Initialize OLED display
  oled.begin(&Adafruit128x64, Display_I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Starting up...");

  // Initialize MPU6050
  if (!mpu.begin()) {
    oled.setCursor(0, 3);
    oled.print("MPU Fail!");
    while (1) {
      digitalWrite(RedLedPin, HIGH);
      delay(500);
      digitalWrite(RedLedPin, LOW);
      delay(500);
    }
  }

  // Initialize QMC5883L
  compass.init();

  // Set up pins
  pinMode(GreenLedPin, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(MotorF, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(BUTTON1, INPUT);
  myservo.attach(servoPin);

  // Initial state
  myservo.write(100);
  analogWrite(MotorF, 0);
  analogWrite(MotorB, 0);

  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Awaiting Start");
}

void loop() {
  connectionStatus();
  analogWrite(MotorF, 0);
  analogWrite(MotorB, 0);

  // Check if the start button is pressed
  if (digitalRead(BUTTON1) == HIGH) {
    start = 1;
    oled.clear();
    oled.setCursor(0, 0);
    oled.print("Autonomy On");
    digitalWrite(GreenLedPin, HIGH);

    while (start != 0) {
      if (digitalRead(BUTTON1) == HIGH) {
        start = 0;
        delay(1000);
      }

      // Sends sensor data to the Jetson Nano for processing
      sendSensorData();
      
      // Receives the final angle command from the Jetson Nano
      receiveMotorCommand();

      // Updates the servo angle
      myservo.write(servo_angle);
      
      // Sets the motor speed to a constant value
      analogWrite(MotorF, 90);
      analogWrite(MotorB, 0);
    }
  }

  digitalWrite(GreenLedPin, LOW);
}

void sendSensorData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  compass.read();
  float heading = compass.getAzimuth();

  // Format and send data to the Jetson Nano
  Serial.print("GY:");
  Serial.print(g.gyro.y);
  Serial.print(",H:");
  Serial.print(heading);
  Serial.print(",AX:");
  Serial.print(a.acceleration.x);
  Serial.print(",AY:");
  Serial.print(a.acceleration.y);
  Serial.print(",AZ:");
  Serial.print(a.acceleration.z);
  Serial.print(",GX:");
  Serial.print(g.gyro.x);
  Serial.print(",GZ:");
  Serial.print(g.gyro.z);
  Serial.println();
}

void connectionStatus(){
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    if (data.startsWith("RL")) {
      digitalWrite(RedLedPin, HIGH);
      return;
    }
  }
}

void receiveMotorCommand() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    // Check for "STOP" command
    if (data.startsWith("S")) {
      analogWrite(MotorF, 0);
      analogWrite(MotorB, 0);
      return;
    }

    if (data.startsWith("RL")) {
      digitalWrite(RedLedPin, HIGH);
      return;
    }

    // Expected format: "A<servo_angle>"
    int a_index = data.indexOf("A");

    if (a_index != -1) {
      String angleStr = data.substring(a_index + 1);
      int received_angle = angleStr.toInt();
      
      // Ensure values are within a safe range
      servo_angle = constrain(received_angle, 75, 145);
    }
  }
}
