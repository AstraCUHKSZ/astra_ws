#include <Arduino.h>
#include "main.h"
#include "comm.h"
#include <lwip/sockets.h>

void setup() {
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  pinMode(15, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);

  comm_init();
}

void loop() {
  // ADC 2 is used by wifi
  // see: https://blog.mjyai.com/2022/10/28/esp32-arduino-ads1115-adc-benchmark/
  uint16_t analog[8];
  analog[0] = digitalRead(15);
  analog[1] = digitalRead(17);
  analog[2] = 0;
  analog[3] = analogRead(4);
  analog[4] = analogRead(5);
  analog[5] = analogRead(6);
  analog[6] = analogRead(7);
  analog[7] = 0;

  // Serial.printf("%hu %hu %hu %hu %hu %hu %hu %hu\n", analog[0], analog[1], analog[2], analog[3], analog[4], analog[5], analog[6], analog[7]);
  
  for (int i = 0; i < 8; ++i) analog[i] = htons(analog[i]);
  comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)analog);

  delay(100);
}
