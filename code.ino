#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define BUZZER_PIN 6  
#define BUTTON_PIN 5  

SoftwareSerial gpsSerial(10, 11);
SoftwareSerial gsmSerial(2, 3);
TinyGPSPlus gps;

const int trigPin1 = A2;
const int echoPin1 = A1;
const int trigPin2 = A4;
const int echoPin2 = A3;

float demoLatitude = 18.5849324;
float demoLongitude = 73.6553194;

bool gpsFixAvailable = false;
bool demoSent = false;

long duration1, duration2;
int distance1, distance2;

float latitude, longitude;
float gpsData[2];
float* p;

unsigned long buzzerTimer = 0;
unsigned long buzzerInterval = 0;
int buzzerState = LOW;
int buzzerBeepCount = 0;
int buzzerPattern = 0;  

// Function prototypes
void readGsmForTrackCommand();
void measureUltrasonicDistances();
void setBuzzerPattern(int pattern);
void handleBuzzerPattern();
void checkRainSensorAndAlert();
float* get_gps();
void SendMessage();

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  delay(1000);
  Serial.println("—Tracking–***Location***");
  gsmSerial.println("AT+CNMI=2,2,0,0,0");
  delay(3000);
  Serial.println("Initializing……");
  delay(2000);
  Serial.println("System Ready");
  delay(1000);
}

void loop() {
  measureUltrasonicDistances();
  handleBuzzerPattern();
  checkRainSensorAndAlert();

  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50); // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("button pressed");
      SendMessage();
      delay(200);
    }
  }

  readGsmForTrackCommand();
}

void measureUltrasonicDistances() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH, 30000);
  if (duration1 == 0) duration1 = 30000;
  distance1 = duration1 * 0.034 / 2;

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH, 30000);
  if (duration2 == 0) duration2 = 30000;
  distance2 = duration2 * 0.034 / 2;

  Serial.print("Distance1: "); Serial.println(distance1);
  Serial.print("Distance2: "); Serial.println(distance2);

  if (distance1 <= 15) {
    setBuzzerPattern(1);
  } else if (distance2 <= 15) {
    setBuzzerPattern(2);
  } else if (distance1 <= 20 || distance2 <= 20) {
    setBuzzerPattern(3);
  } else {
    setBuzzerPattern(0);
  }
}

void setBuzzerPattern(int pattern) {
  if (buzzerPattern != pattern) {
    buzzerPattern = pattern;
    buzzerBeepCount = 0;
    buzzerState = LOW;
    digitalWrite(BUZZER_PIN, buzzerState);
    buzzerTimer = millis();
    switch (pattern) {
      case 1: buzzerInterval = 200; break;
      case 2: buzzerInterval = 600; break;
      case 3: buzzerInterval = 1000; break;
      default: buzzerInterval = 0; break;
    }
  }
}

void handleBuzzerPattern() {
  if (buzzerPattern == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - buzzerTimer >= buzzerInterval) {
    buzzerTimer = currentMillis;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);

    if (buzzerState == LOW) {
      buzzerBeepCount++;
      if ((buzzerPattern == 1 && buzzerBeepCount >= 3) ||
          (buzzerPattern == 2 && buzzerBeepCount >= 2)) {
        buzzerPattern = 0;
        digitalWrite(BUZZER_PIN, LOW);
      }
      if (buzzerPattern == 3) {
        buzzerPattern = 0;
        digitalWrite(BUZZER_PIN, LOW);
      }
    }
  }
}

void checkRainSensorAndAlert() {
  static unsigned long lastRainAlert = 0;
  int rainValue = digitalRead(A4);
  unsigned long now = millis();
  if (rainValue == 1 && now - lastRainAlert > 4000) {
    Serial.println("Rain Alert");
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(300);
      digitalWrite(BUZZER_PIN, LOW);
      delay(300);
    }
    lastRainAlert = now;
  }
}

float* get_gps() {
  gpsSerial.listen();
  Serial.println("INSIDE get_gps");
  unsigned long start = millis();

  while (millis() - start < 5000) {
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      gpsFixAvailable = true;
      demoSent = false;
      Serial.print("LAT="); Serial.println(latitude, 6);
      Serial.print("LONG="); Serial.println(longitude, 6);
      gpsData[0] = latitude;
      gpsData[1] = longitude;
      return gpsData;
    }
  }

  if (!demoSent) {
    gpsFixAvailable = false;
    demoSent = true;
    Serial.println("GPS signal not available, sending demo location.");
    gpsData[0] = demoLatitude;
    gpsData[1] = demoLongitude;
    return gpsData;
  }

  return nullptr;
}

void readGsmForTrackCommand() {
  gsmSerial.listen();
  while (gsmSerial.available() > 0) {
    Serial.println("INSIDE gsmSerial.available");
    if (gsmSerial.find("Track")) {
      Serial.println("INSIDE track");
      gsmSerial.println("AT+CMGF=1");
      delay(1000);
      gsmSerial.println("AT+CMGS=\"+917019415675\"\r");
      delay(1000);
      p = get_gps();

      if (p != nullptr) {
        Serial.print("Your Car Location: ");
        gsmSerial.print("Your Car Location: ");

        gsmSerial.print("https://www.google.com/maps?q=");
        gsmSerial.print(*p, 6);
        gsmSerial.print(",");
        gsmSerial.print(*(p + 1), 6);

        delay(100);
        gsmSerial.println((char)26);
        delay(1000);
      } else {
        Serial.println("Demo location already sent, skipping repeated send.");
      }
    }
  }
}

void SendMessage() {
  gsmSerial.println("AT+CMGF=1");
  delay(500);
  gsmSerial.println("AT+CMGS=\"+917019415675\"\r"); //7019415675
  delay(500);

  gsmSerial.print("I Am In Problem Plz Help Me. My Location: ");
  p = get_gps();
  gsmSerial.listen();

  if (p != nullptr) {
    gsmSerial.print("https://www.google.com/maps?q=");
    gsmSerial.print(*p, 6);
    gsmSerial.print(",");
    gsmSerial.print(*(p + 1), 6);
  } else {
    gsmSerial.print("GPS signal not available.");
  }

  delay(500);
  gsmSerial.write(26);  

  unsigned long start = millis();
  while (millis() - start < 4000) {
    while (gsmSerial.available()) {
      Serial.write(gsmSerial.read());
    }
  }
}
