#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ----------- WiFi HOTSPOT -----------
const char* ssid = "ESP32_ROBOT";
const char* password = "12345678";

WiFiServer server(80);        // Webpage
WiFiServer dataServer(12345); // Data stream
WiFiClient dataClient;

// ----------- Pins -----------
#define ACS712_PIN 34
#define DHT_PIN 4
#define DHT_TYPE DHT11

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

String command = "normal";

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
    while (1);
  }

  WiFi.softAP(ssid, password);
  server.begin();
  dataServer.begin();

  Serial.println("\n===== ESP32 HOTSPOT STARTED =====");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
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

  // ----------- Handle DATA client (FIRST) -----------
  if (!dataClient || !dataClient.connected()) {
    dataClient = dataServer.available();
  }

  // ----------- Handle WEB client -----------
  WiFiClient client = server.available();

  if (client) {
    String request = client.readStringUntil('\r');

    if (request.indexOf("/m") != -1) command = "moving";
    if (request.indexOf("/b") != -1) command = "blocked";
    if (request.indexOf("/n") != -1) command = "normal";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html\n");

    client.println("<html><body>");
    client.println("<h2>Robot Control</h2>");
    client.println("<a href=\"/m\"><button>Moving</button></a>");
    client.println("<a href=\"/b\"><button>Blocked</button></a>");
    client.println("<a href=\"/n\"><button>Normal</button></a>");
    client.println("</body></html>");

    client.stop();
  }

  // ----------- Sensor Reading -----------
  int raw = analogRead(ACS712_PIN);

  if (!baseline_set && command == "normal") {
    baseline_raw = raw;
    baseline_set = true;
  }

  int delta = raw - baseline_raw;

  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  float accel_mag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  float vibration = abs(accel_mag - 9.8);

  float temperature = dht.readTemperature();
  if (isnan(temperature) || temperature < 10) {
    temperature = 30.0;
  }
  temperature += 2.0;

  // ----------- Motor Logic -----------
  if (command == "moving" || command == "blocked") {
    moveForward();
  } else {
    stopMotor();
  }

  // ----------- CREATE DATA STRING (FIXED) -----------
  String data = String(millis()) + "," +
                String(raw) + "," +
                String(delta) + "," +
                String(vibration) + "," +
                String(temperature) + "," +
                command;

  // ----------- OUTPUT -----------
  Serial.println(data);

  if (dataClient && dataClient.connected()) {
    dataClient.println(data);
  }

  delay(200);
}