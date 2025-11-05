// Pins definieren
const int stepPin = 2;
const int dirPin = 3;
const int endstopPin = 4;
const int vorwaertsPin = 5;     // 5V kommt hier an bei Vorwärts-Knopf
const int rueckwaertsPin = 6;   // 5V kommt hier an bei Rückwärts-Knopf
const int enablePin = 8;

// Motorparameter
const int stepsPerRevolution = 200;
const long maxSteps = 2064;     // Maximale Steps für Roboter-Arm

// Geschwindigkeit (Verzögerung in Mikrosekunden)
const long stepIntervall = 1500;  // 200 U/min

// Variablen
bool motorLaeuft = false;
bool richtungVorwaerts = true;
unsigned long letzterStepZeit = 0;
long stepZaehler = 0;           // Zählt die Steps
bool endstopBeruehrt = false;   // Merkt ob Endstop schon mal berührt wurde
bool homingCompleted = false;   // Homing abgeschlossen

void setup() {
  // Pins konfigurieren
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(endstopPin, INPUT_PULLUP);    // Pull-up aktiv, LOW = aktiv
  pinMode(vorwaertsPin, INPUT);    // Als Input ohne Pull-up, weil 5V extern kommt
  pinMode(rueckwaertsPin, INPUT);  // Als Input ohne Pull-up, weil 5V extern kommt
  
  // Motor initial DEAKTIVIEREN (A4988: HIGH = deaktiviert)
  digitalWrite(enablePin, HIGH);
  digitalWrite(dirPin, HIGH);  // Initial Vorwärts
  digitalWrite(stepPin, LOW);
  
  Serial.begin(9600);
  Serial.println("=== Roboter-Arm Steuerung Start ===");
  Serial.println("Vorwärts: 5V an Pin 5");
  Serial.println("Rückwärts: 5V an Pin 6");
  Serial.println("Endschalter: LOW an Pin 4 = Homing Position");
  Serial.println("Geschwindigkeit: 200 U/min");
  Serial.println("Maximale Steps: 2064 (digitale Endposition)");
  Serial.println("Steps-Zählung: 's' im Serial Monitor");
  Serial.println("⚠️  FREIFAHRT MODUS: Alles möglich bis zum ersten Homing");
  Serial.println("============================");
}

void loop() {
  bool endstopAktiv = (digitalRead(endstopPin) == LOW);
  
  // Homing: Erste Endstop-Berührung erkennen
  if (!endstopBeruehrt && endstopAktiv) {
    endstopBeruehrt = true;
    homingCompleted = true;
    stepZaehler = 0;
    Serial.println("🎯 HOMING ABGESCHLOSSEN - Position auf 0 gesetzt");
    Serial.println("✅ Roboter-Arm bereit für Bewegung (0-2064 Steps)");
  }
  
  // Vorwärts-Knopf hat 5V bekommen
  if (digitalRead(vorwaertsPin) == HIGH) {
    if (!motorLaeuft) {
      // Vor Homing: Alles erlaubt
      // Nach Homing: Nur wenn nicht an max Steps
      if (homingCompleted && stepZaehler >= maxSteps) {
        Serial.println("❌ MAXIMUM ERREICHT - Keine Vorwärtsbewegung möglich!");
        return;
      }
      
      richtungVorwaerts = true;
      digitalWrite(dirPin, HIGH);
      motorStarten();
      if (!homingCompleted) {
        Serial.println("➡️  Vorwärts gestartet (FREIFAHRT MODUS)");
      } else {
        Serial.println("➡️  Vorwärts gestartet");
      }
    }
  }
  
  // Rückwärts-Knopf hat 5V bekommen (immer erlaubt)
  if (digitalRead(rueckwaertsPin) == HIGH) {
    if (!motorLaeuft) {
      richtungVorwaerts = false;
      digitalWrite(dirPin, LOW);
      motorStarten();
      if (!homingCompleted) {
        Serial.println("⬅️  Rückwärts gestartet (FREIFAHRT MODUS)");
      } else {
        Serial.println("⬅️  Rückwärts gestartet");
      }
    }
  }
  
  // Wenn Endstop aktiv ist und Rückwärts gedrückt wird (nur nach Homing relevant)
  if (digitalRead(rueckwaertsPin) == HIGH && endstopAktiv && homingCompleted) {
    if (motorLaeuft && !richtungVorwaerts) {
      // Zurücksetzen auf Home-Position
      stepZaehler = 0;
      Serial.println("🏠 Zur Home-Position zurückgesetzt");
    }
  }
  
  // Wenn keine 5V mehr an den Richtungs-Pins, Motor stoppen
  if (digitalRead(vorwaertsPin) == LOW && digitalRead(rueckwaertsPin) == LOW && motorLaeuft) {
    motorStoppen();
    if (!homingCompleted) {
      Serial.println("⏹️  Motor gestoppt (FREIFAHRT MODUS)");
    } else {
      Serial.println("⏹️  Motor gestoppt");
    }
  }
  
  // Motorsteuerung
  if (motorLaeuft) {
    motorSteuern();
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
    
    // Steps zählen
    if (richtungVorwaerts) {
      stepZaehler++;  // Vorwärts: Steps erhöhen
      
      // Nach Homing: Prüfen ob maximale Steps erreicht
      if (homingCompleted && stepZaehler >= maxSteps) {
        motorStoppen();
        Serial.println("🛑 MAXIMALE POSITION ERREICHT (2064 Steps)");
        Serial.println("❌ Keine weitere Vorwärtsbewegung möglich!");
      }
    } else {
      stepZaehler--;  // Rückwärts: Steps verringern
      
      // Nach Homing: Verhindern negative Steps
      if (homingCompleted && stepZaehler < 0) {
        stepZaehler = 0;
      }
    }
    
    letzterStepZeit = aktuelleZeit;
  }
}

// Steps manuell abfragen (kann über Serial Monitor aufgerufen werden)
void serialEvent() {
  while (Serial.available()) {
    char input = Serial.read();
    if (input == 's' || input == 'S') {
      Serial.print("📊 Step-Zähler: ");
      Serial.println(stepZaehler);
      Serial.print("📏 Umdrehungen: ");
      Serial.println((float)stepZaehler / stepsPerRevolution, 2);
      
      if (homingCompleted) {
        Serial.print("📐 Verbleibende Steps bis Maximum: ");
        Serial.println(maxSteps - stepZaehler);
        Serial.print("📊 Progress: ");
        Serial.print((float)stepZaehler / maxSteps * 100, 1);
        Serial.println("%");
        Serial.println("✅ Homing abgeschlossen");
      } else {
        Serial.println("⚠️  FREIFAHRT MODUS - Homing noch nicht durchgeführt");
      }
    }
    if (input == 'r' || input == 'R') {
      // Manuelles Reset
      endstopBeruehrt = false;
      homingCompleted = false;
      stepZaehler = 0;
      Serial.println("🔄 FREIFAHRT MODUS aktiviert - Alles möglich!");
    }
  }
}