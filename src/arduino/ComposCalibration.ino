#include <Wire.h>
#include <QMC5883LCompass.h>
#include <EEPROM.h>

QMC5883LCompass compass;

// Manual calibration variables
int x_min = 32767, x_max = -32768;
int y_min = 32767, y_max = -32768;
int z_min = 32767, z_max = -32768;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  // Initialize compass
  compass.init();
  
  Serial.println("Calibrating QMC5883L. Rotate the sensor in a figure 8 pattern.");
  Serial.println("Press 'S' to save the calibration values.");
}

void loop() {
  // Read sensor data
  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  // Update min/max values
  if (x < x_min) x_min = x;
  if (x > x_max) x_max = x;
  if (y < y_min) y_min = y;
  if (y > y_max) y_max = y;
  if (z < z_min) z_min = z;
  if (z > z_max) z_max = z;
  
  // Print current raw values and min/max ranges
  Serial.print("X: "); Serial.print(x);
  Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Z: "); Serial.print(z);
  Serial.print(" | Range X: "); Serial.print(x_min); Serial.print(" to "); Serial.print(x_max);
  Serial.print(", Y: "); Serial.print(y_min); Serial.print(" to "); Serial.print(y_max);
  Serial.print(", Z: "); Serial.print(z_min); Serial.print(" to "); Serial.print(z_max);
  Serial.println();

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'S' || command == 's') {
      // Save to EEPROM
      EEPROM.put(0, x_min);
      EEPROM.put(sizeof(int), x_max);
      EEPROM.put(2 * sizeof(int), y_min);
      EEPROM.put(3 * sizeof(int), y_max);
      EEPROM.put(4 * sizeof(int), z_min);
      EEPROM.put(5 * sizeof(int), z_max);

      Serial.println("Calibration values saved to EEPROM!");
      Serial.print("X_low: "); Serial.println(x_min);
      Serial.print("X_high: "); Serial.println(x_max);
      Serial.print("Y_low: "); Serial.println(y_min);
      Serial.print("Y_high: "); Serial.println(y_max);
      Serial.print("Z_low: "); Serial.println(z_min);
      Serial.print("Z_high: "); Serial.println(z_max);

      while (1); // Halt the sketch
    }
  }
}
