// mpu6050_pico_with_encoder.ino

#include <Arduino.h>
#include <Wire.h>

// MPU6050
#define MPU_ADDR 0x68

// Pin definitions
#define PIN_LED  25   // Onboard Pico LED

// Rotary encoder pins (changed to avoid I2C conflict)
#define PIN_A 14
#define PIN_B 15


// Rotary encoder variables

volatile bool rotaryEncoder = false;
int rotationCounter = 0;

void rotary() {
  rotaryEncoder = true;
}

// Decode direction
int8_t checkRotaryEncoder() {
  rotaryEncoder = false;

  static uint8_t lrmem = 3;
  static int lrsum = 0;

  static int8_t TRANS[] = {
    0, -1, 1, 14,
    1, 0, 14, -1,
    -1, 14, 0, 1,
    14, 1, -1, 0
  };

  int8_t l = digitalRead(PIN_A);
  int8_t r = digitalRead(PIN_B);

  lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;
  lrsum += TRANS[lrmem];

  if (lrsum % 4 != 0) return 0;

  if (lrsum == 4) {
    lrsum = 0;
    return 1;   // CW
  }

  if (lrsum == -4) {
    lrsum = 0;
    return -1;  // CCW
  }

  lrsum = 0;
  return 0;
}


// Timing

unsigned long start_time_ms = 0;
unsigned long loop_time_us  = 0;


// MPU data

int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
int16_t raw_temp = 0;


// Wake MPU6050

void mpu_wake() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0x00);
  Wire1.endTransmission();

  delay(100);

  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x1B);
  Wire1.write(0x00);
  Wire1.endTransmission();
}


// Read raw accel + gyro + temp

void read_raw(int16_t &ax, int16_t &ay, int16_t &az,
              int16_t &gx, int16_t &gy, int16_t &gz,
              int16_t &raw_temp) {

  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);
  Wire1.endTransmission(false);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  uint8_t data[14];
  for (int i = 0; i < 14; i++) data[i] = Wire1.read();

  ax       = (int16_t)((data[0]  << 8) | data[1]);
  ay       = (int16_t)((data[2]  << 8) | data[3]);
  az       = (int16_t)((data[4]  << 8) | data[5]);
  raw_temp = (int16_t)((data[6]  << 8) | data[7]);
  gx       = (int16_t)((data[8]  << 8) | data[9]);
  gy       = (int16_t)((data[10] << 8) | data[11]);
  gz       = (int16_t)((data[12] << 8) | data[13]);
}

float temp_celsius(int16_t raw) {
  return raw / 340.0f + 36.53f;
}

// setup()

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // I2C setup
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Wire1.setClock(400000);

  mpu_wake();
  Serial.println("#MPU6050 + Encoder ready");

  // LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Rotary encoder setup
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_A), rotary, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), rotary, CHANGE);

  // Startup delay
  delay(2000);
  digitalWrite(PIN_LED, HIGH);

  start_time_ms = millis();
  loop_time_us  = micros();

  Serial.println("#elapsed_ms,ax,ay,az,gx,gy,gz,temp_c,encoder");
}


// loop()

void loop() {
  // Handle rotary encoder
  if (rotaryEncoder) {
    int8_t dir = checkRotaryEncoder();
    if (dir != 0) {
      rotationCounter += dir;
    }
  }

  // Sample MPU6050 at ~250 Hz
  unsigned long now_us = micros();
  if ((now_us - loop_time_us) >= 4000) {
    loop_time_us = now_us;

    unsigned long elapsed = millis() - start_time_ms;
    read_raw(ax, ay, az, gx, gy, gz, raw_temp);
    float temp_c = temp_celsius(raw_temp);

    Serial.print(elapsed);   Serial.print(",");
    Serial.print(ax);        Serial.print(",");
    Serial.print(ay);        Serial.print(",");
    Serial.print(az);        Serial.print(",");
    Serial.print(gx);        Serial.print(",");
    Serial.print(gy);        Serial.print(",");
    Serial.print(gz);        Serial.print(",");
    Serial.print(temp_c, 2); Serial.print(",");
    Serial.println(rotationCounter);
  }
}
