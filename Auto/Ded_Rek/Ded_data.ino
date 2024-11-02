#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // Check if the MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Send the data over the serial port
  Serial.print(ax);
  Serial.print(","); 
  Serial.print(ay);
  Serial.print(","); 
  Serial.print(az);
  Serial.print(","); 
  Serial.print(gx);
  Serial.print(","); 
  Serial.print(gy);
  Serial.print(","); 
  Serial.println(gz);

  delay(100);
}
