#include <Arduino.h>
#include <enigma.h>
#define BMP_ADDR 0x77
#define IMU_ADDR 0x68

BMP bmp(BMP_ADDR);
// IMU imu(IMU_ADDR);

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  float alt = bmp.getalt();
  Serial.println(alt);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);
  // Serial.println("This is the loop");
}