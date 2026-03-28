#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ----------- Pins -----------
#define ACS712_PIN 34
#define DHT_PIN 4
#define DHT_TYPE DHT11

// Motors
#define IN1 26
#define IN2 27
#define ENA 25

#define IN3 14
#define IN4 12
#define ENB 33

// ----------- Objects -----------
Adafruit_MPU6050 mpu;
DHT dht(DHT_PIN, DHT_TYPE);

// ----------- Variables -----------
int baseline_raw = 0;
bool baseline_set = false;

String state = "NORMAL";
String command = "normal";  // normal / moving / blocked / auto

// Time filtering
int blocked_count = 0;
int moving_count = 0;

const int CONFIRM_COUNT = 4;

// ----------- Setup -----------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  dht.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.println("time,raw,delta,vibration,temp,state");

  delay(2000);
}

// ----------- Motor Control -----------
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ----------- Loop -----------
void loop() {

  // ----------- Serial Control -----------
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'm') command = "moving";
    if (c == 'b') command = "blocked";
    if (c == 'n') command = "normal";
    if (c == 'a') command = "auto";
  }

  int raw = analogRead(ACS712_PIN);

  // ----------- Baseline (ONLY when normal) -----------
  if (!baseline_set && command == "normal") {
    baseline_raw = raw;
    baseline_set = true;
  }

  int delta = raw - baseline_raw;

  // ----------- MPU6050 -----------
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  float accel_mag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  float vibration = abs(accel_mag - 9.8);

  // ----------- Temperature -----------
  float temperature = dht.readTemperature();

  if (isnan(temperature) || temperature < 10) {
    temperature = 30.0;
  }

  temperature += 2.0;

  // ----------- Decision Logic -----------

  if (command == "moving") {
    state = "MOVING";
    moveForward();
  }
  else if (command == "blocked") {
    state = "BLOCKED";
    moveForward();
  }
  else if (command == "normal") {
    state = "NORMAL";
    stopMotor();
  }
  else {
    // -------- AUTO MODE --------

    if (delta > 150 && vibration < 0.5) {
      blocked_count++;
    } else {
      blocked_count = 0;
    }

    if (delta > 50 && delta <= 150) {
      moving_count++;
    } else {
      moving_count = 0;
    }

    if (blocked_count >= CONFIRM_COUNT) {
      state = "BLOCKED";
      stopMotor();
    }
    else if (moving_count >= CONFIRM_COUNT) {
      state = "MOVING";
      moveForward();
    }
    else {
      state = "NORMAL";
      stopMotor();
    }
  }

  // ----------- Output -----------
  Serial.print(millis());
  Serial.print(",");
  Serial.print(raw);
  Serial.print(",");
  Serial.print(delta);
  Serial.print(",");
  Serial.print(vibration);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.println(state);

  delay(200);
}