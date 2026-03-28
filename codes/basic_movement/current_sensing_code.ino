#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ----------- Pins -----------
#define ACS712_PIN 34
#define DHT_PIN 4
#define DHT_TYPE DHT11

// LEFT MOTOR
#define IN1 26
#define IN2 27
#define ENA 25

// RIGHT MOTOR
#define IN3 14
#define IN4 12
#define ENB 33

// ----------- Objects -----------
Adafruit_MPU6050 mpu;
DHT dht(DHT_PIN, DHT_TYPE);

// ----------- Variables -----------
String label = "moving";

int baseline_raw = 0;
bool baseline_set = false;

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

  Serial.println("time,current_ratio,accel_mag,temp,label");

  delay(2000);
}

// ----------- Motor Control -----------

// Forward motion (corrected direction)
void moveForward() {
  // LEFT motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // RIGHT motor (reversed logic)
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

  // ----------- Label Control -----------
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'n') {
      label = "normal";
      stopMotor();          // motor OFF for baseline
      baseline_set = false;
    }

    if (c == 'm') {
      label = "moving";
      moveForward();
    }

    if (c == 'b') {
      label = "blocked";
      moveForward();
    }
  }

  // ----------- Current (RAW + normalized) -----------
  int raw = analogRead(ACS712_PIN);
  Serial.print("RAW:");
  Serial.println(raw);

  if (!baseline_set && label == "normal") {
    baseline_raw = raw;
    baseline_set = true;
  }

  float current_ratio = baseline_set ? (float)raw / baseline_raw : 1.0;

  // ----------- MPU6050 (Vibration) -----------
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  float accel_mag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  // Remove gravity effect (~9.8)
  float vibration = abs(accel_mag - 9.8);

  // ----------- Temperature (calibrated) -----------
  float temperature = dht.readTemperature();

  if (isnan(temperature) || temperature < 10) {
    temperature = 30.0;   // fallback (room temp)
  }

  temperature += 2.0;     // calibration offset

  // ----------- Output CSV -----------
  Serial.print(millis());
  Serial.print(",");
  Serial.print(current_ratio, 3);
  Serial.print(",");
  Serial.print(vibration);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.println(label);

  delay(200);
}