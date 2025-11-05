// Pins definieren
const int stepPin = 2;
const int dirPin = 3;
const int endstopPin = 4;
const int vorwaertsPin = 5;     // 5V kommt hier an bei Vorwärts-Knopf
const int rueckwaertsPin = 6;   // 5V kommt hier an bei Rückwärts-Knopf
const int geschwindigkeitPin = 7;
const int enablePin = 8;

// Motorparameter
const int stepsPerRevolution = 200;
int aktuelleGeschwindigkeit = 1;  // 1 = Stufe 1, 2 = Stufe 2

// Geschwindigkeitseinstellungen (Verzögerung in Mikrosekunden)
long geschwindigkeiten[2] = {1500, 300};  // 200 U/min und 1000 U/min

// Variablen
bool motorLaeuft = false;
bool richtungVorwaerts = true;
unsigned long letzterStepZeit = 0;
long stepIntervall = geschwindigkeiten[0];

void setup() {
  // Pins konfigurieren
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(endstopPin, INPUT_PULLUP);
  pinMode(vorwaertsPin, INPUT);    // Als Input ohne Pull-up, weil 5V extern kommt
  pinMode(rueckwaertsPin, INPUT);  // Als Input ohne Pull-up, weil 5V extern kommt
  pinMode(geschwindigkeitPin, INPUT_PULLUP);
  
  // Motor initial DEAKTIVIEREN (A4988: HIGH = deaktiviert)
  digitalWrite(enablePin, HIGH);
  digitalWrite(dirPin, HIGH);  // Initial Vorwärts
  digitalWrite(stepPin, LOW);
  
  Serial.begin(9600);
  Serial.println("=== Motorsteuerung Start ===");
  Serial.println("Warte auf 5V Signal an Pin 5 (Vorwärts) oder Pin 6 (Rückwärts)");
  Serial.println("Aktuell: Stufe 1 - 200 U/min");
  Serial.println("============================");
}

void loop() {
  // Geschwindigkeitswechsel prüfen
  if (digitalRead(geschwindigkeitPin) == LOW) {
    wechsleGeschwindigkeit();
    delay(300);  // Entprellung
  }
  
  // Endschalter prüfen
  if (digitalRead(endstopPin) == LOW) {
    if (motorLaeuft) {
      motorStoppen();
      Serial.println("⚠️  Endschalter aktiviert - Motor gestoppt");
    }
    return;
  }
  
  // Vorwärts-Knopf hat 5V bekommen
  if (digitalRead(vorwaertsPin) == HIGH) {
    if (!motorLaeuft) {
      richtungVorwaerts = true;
      digitalWrite(dirPin, HIGH);
      motorStarten();
      Serial.println("➡️  Vorwärts gestartet (5V an Pin 5 erkannt)");
    }
  }
  
  // Rückwärts-Knopf hat 5V bekommen
  if (digitalRead(rueckwaertsPin) == HIGH) {
    if (!motorLaeuft) {
      richtungVorwaerts = false;
      digitalWrite(dirPin, LOW);
      motorStarten();
      Serial.println("⬅️  Rückwärts gestartet (5V an Pin 6 erkannt)");
    }
  }
  
  // Wenn keine 5V mehr an den Richtungs-Pins, Motor stoppen
  if (digitalRead(vorwaertsPin) == LOW && digitalRead(rueckwaertsPin) == LOW && motorLaeuft) {
    motorStoppen();
    Serial.println("⏹️  Keine 5V Signal - Motor gestoppt");
  }
  
  // Motorsteuerung
  if (motorLaeuft) {
    motorSteuern();
  }
}

void wechsleGeschwindigkeit() {
  if (aktuelleGeschwindigkeit == 1) {
    aktuelleGeschwindigkeit = 2;
    stepIntervall = geschwindigkeiten[1];
    Serial.println("🚀 Geschwindigkeitsstufe: 2 (1000 U/min)");
  } else {
    aktuelleGeschwindigkeit = 1;
    stepIntervall = geschwindigkeiten[0];
    Serial.println("🐢 Geschwindigkeitsstufe: 1 (200 U/min)");
  }
}

void motorStarten() {
  // Motor aktivieren (A4988: LOW = aktiv)
  digitalWrite(enablePin, LOW);
  motorLaeuft = true;
  letzterStepZeit = micros();
}

void motorStoppen() {
  motorLaeuft = false;
  // Motor deaktivieren (A4988: HIGH = deaktiviert)
  digitalWrite(enablePin, HIGH);
}

void motorSteuern() {
  unsigned long aktuelleZeit = micros();
  
  if (aktuelleZeit - letzterStepZeit >= stepIntervall) {
    // Step-Pin toggeln
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    
    letzterStepZeit = aktuelleZeit;
  }
}