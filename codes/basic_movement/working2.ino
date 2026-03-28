#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 33

int pwmFreq = 1000;
int pwmResolution = 8;

int startBoost = 255;   // strong start power
int runSpeed  = 190;    // normal running speed

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);
}

void moveForward() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Start boost
  ledcWrite(ENA, startBoost);
  ledcWrite(ENB, startBoost);

  delay(500);

  // Normal speed
  ledcWrite(ENA, runSpeed);
  ledcWrite(ENB, runSpeed);
}

void moveBackward() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, startBoost);
  ledcWrite(ENB, startBoost);

  delay(500);

  ledcWrite(ENA, runSpeed);
  ledcWrite(ENB, runSpeed);
}

void stopMotors() {

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void loop() {

  moveForward();
  delay(3000);

  stopMotors();
  delay(2000);

  moveBackward();
  delay(3000);

  stopMotors();
  delay(5000);
}