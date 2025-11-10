// ===== PIN DEFINITIONEN =====
// Basis (Rotation) Pins
const int stepPin_basis = 3;
const int dirPin_basis = 4;
const int endstopPin_basis = 5;
const int vorwaertsPin_basis = 6;
const int rueckwaertsPin_basis = 8;
const int enablePin_basis = 9;

// Oberarm (Shoulder) Pins
const int stepPin_oberarm = 22;
const int dirPin_oberarm = 23;
const int endstopPin_oberarm = 24;
const int vorwaertsPin_oberarm = 25;
const int rueckwaertsPin_oberarm = 26;
const int enablePin_oberarm = 28;

// Ellbogen (Elbow) Pins
const int stepPin_ellbogen = 40;
const int dirPin_ellbogen = 41;
const int endstopPin_ellbogen = 42;
const int vorwaertsPin_ellbogen = 43;
const int rueckwaertsPin_ellbogen = 44;
const int enablePin_ellbogen = 46;

// ===== MOTORPARAMETER =====
const int stepsPerRevolution = 200;
const long maxSteps_basis = 1700;      // Maximale Steps für Basis
const long maxSteps_oberarm = 2064;    // Maximale Steps für Oberarm (ANGEPASST)
const long maxSteps_ellbogen = 2064;   // Maximale Steps für Ellbogen

// Geschwindigkeit (Verzögerung in Mikrosekunden)
const long stepIntervall = 1500;  // 200 U/min

// ===== VARIABLEN FÜR BASIS =====
bool motorLaeuft_basis = false;
bool richtungVorwaerts_basis = true;
unsigned long letzterStepZeit_basis = 0;
long stepZaehler_basis = 0;
bool endstopBeruehrt_basis = false;
bool homingCompleted_basis = false;

// ===== VARIABLEN FÜR OBERARM =====
bool motorLaeuft_oberarm = false;
bool richtungVorwaerts_oberarm = true;
unsigned long letzterStepZeit_oberarm = 0;
long stepZaehler_oberarm = 0;
bool endstopBeruehrt_oberarm = false;
bool homingCompleted_oberarm = false;

// ===== VARIABLEN FÜR ELLBOGEN =====
bool motorLaeuft_ellbogen = false;
bool richtungVorwaerts_ellbogen = true;
unsigned long letzterStepZeit_ellbogen = 0;
long stepZaehler_ellbogen = 0;
bool endstopBeruehrt_ellbogen = false;
bool homingCompleted_ellbogen = false;

// ===== GEMEINSAME VARIABLEN =====
unsigned long letzteTastenZeit_basis = 0;
unsigned long letzteTastenZeit_oberarm = 0;
unsigned long letzteTastenZeit_ellbogen = 0;
const unsigned long entprellDelay = 50;

void setup() {
  // ===== PIN KONFIGURATION BASIS =====
  pinMode(stepPin_basis, OUTPUT);
  pinMode(dirPin_basis, OUTPUT);
  pinMode(enablePin_basis, OUTPUT);
  pinMode(endstopPin_basis, INPUT_PULLUP);
  pinMode(vorwaertsPin_basis, INPUT);
  pinMode(rueckwaertsPin_basis, INPUT);
  
  // ===== PIN KONFIGURATION OBERARM =====
  pinMode(stepPin_oberarm, OUTPUT);
  pinMode(dirPin_oberarm, OUTPUT);
  pinMode(enablePin_oberarm, OUTPUT);
  pinMode(endstopPin_oberarm, INPUT_PULLUP);
  pinMode(vorwaertsPin_oberarm, INPUT);
  pinMode(rueckwaertsPin_oberarm, INPUT);
  
  // ===== PIN KONFIGURATION ELLBOGEN =====
  pinMode(stepPin_ellbogen, OUTPUT);
  pinMode(dirPin_ellbogen, OUTPUT);
  pinMode(enablePin_ellbogen, OUTPUT);
  pinMode(endstopPin_ellbogen, INPUT_PULLUP);
  pinMode(vorwaertsPin_ellbogen, INPUT);
  pinMode(rueckwaertsPin_ellbogen, INPUT);
  
  // ===== MOTOREN INITIALISIEREN =====
  // Basis initial DEAKTIVIEREN
  digitalWrite(enablePin_basis, HIGH);
  digitalWrite(dirPin_basis, HIGH);  // Vorwärts = HIGH
  digitalWrite(stepPin_basis, LOW);
  
  // Oberarm initial DEAKTIVIEREN
  digitalWrite(enablePin_oberarm, HIGH);
  digitalWrite(dirPin_oberarm, HIGH);  // Vorwärts = HIGH
  digitalWrite(stepPin_oberarm, LOW);
  
  // Ellbogen initial DEAKTIVIEREN (UMGEKEHRT für Ellbogen)
  digitalWrite(enablePin_ellbogen, HIGH);
  digitalWrite(dirPin_ellbogen, HIGH);  // VORWÄRTS = HIGH für Ellbogen (umgekehrt)
  digitalWrite(stepPin_ellbogen, LOW);
  
  Serial.begin(9600);
  Serial.println("=== ROBOTER-ARM 3-ACHSEN STEUERUNG ===");
  Serial.println("=== BASIS (ROTATION) ===");
  Serial.println("Vorwärts: 5V an Pin 6");
  Serial.println("Rückwärts: 5V an Pin 8");
  Serial.println("Endschalter: LOW an Pin 5");
  Serial.println("Maximale Steps: 1700");
  Serial.println("=== OBERARM (SHOULDER) ===");
  Serial.println("Vorwärts: 5V an Pin 25");
  Serial.println("Rückwärts: 5V an Pin 26");
  Serial.println("Endschalter: LOW an Pin 24");
  Serial.println("Maximale Steps: 2064");
  Serial.println("=== ELLBOGEN (ELBOW) ===");
  Serial.println("Vorwärts: 5V an Pin 43");
  Serial.println("Rückwärts: 5V an Pin 44");
  Serial.println("Endschalter: LOW an Pin 42");
  Serial.println("Maximale Steps: 2064");
  Serial.println("DIR-REVERSE: Vorwärts = HIGH, Rückwärts = LOW");
  Serial.println("=== ALLGEMEIN ===");
  Serial.println("Geschwindigkeit: 200 U/min");
  Serial.println("🔄 RESET: Vorwärts + Rückwärts gleichzeitig drücken");
  Serial.println("Steps-Zählung: 's' für alle, '1' für Basis, '2' für Oberarm, '3' für Ellbogen");
  Serial.println("============================");
}

void loop() {
  // ===== BASIS STEUERUNG =====
  steuereAchse(
    "BASIS",
    stepPin_basis, dirPin_basis, endstopPin_basis,
    vorwaertsPin_basis, rueckwaertsPin_basis, enablePin_basis,
    motorLaeuft_basis, richtungVorwaerts_basis, letzterStepZeit_basis,
    stepZaehler_basis, endstopBeruehrt_basis, homingCompleted_basis,
    maxSteps_basis,
    letzteTastenZeit_basis,
    false  // Normal für Basis
  );
  
  // ===== OBERARM STEUERUNG =====
  steuereAchse(
    "OBERARM",
    stepPin_oberarm, dirPin_oberarm, endstopPin_oberarm,
    vorwaertsPin_oberarm, rueckwaertsPin_oberarm, enablePin_oberarm,
    motorLaeuft_oberarm, richtungVorwaerts_oberarm, letzterStepZeit_oberarm,
    stepZaehler_oberarm, endstopBeruehrt_oberarm, homingCompleted_oberarm,
    maxSteps_oberarm,
    letzteTastenZeit_oberarm,
    false  // Normal für Oberarm
  );
  
  // ===== ELLBOGEN STEUERUNG =====
  steuereAchse(
    "ELLBOGEN", 
    stepPin_ellbogen, dirPin_ellbogen, endstopPin_ellbogen,
    vorwaertsPin_ellbogen, rueckwaertsPin_ellbogen, enablePin_ellbogen,
    motorLaeuft_ellbogen, richtungVorwaerts_ellbogen, letzterStepZeit_ellbogen,
    stepZaehler_ellbogen, endstopBeruehrt_ellbogen, homingCompleted_ellbogen,
    maxSteps_ellbogen,
    letzteTastenZeit_ellbogen,
    true   // UMGEKEHRT für Ellbogen
  );
}

void steuereAchse(
  const char* achsenName,
  int stepPin, int dirPin, int endstopPin,
  int vorwaertsPin, int rueckwaertsPin, int enablePin,
  bool &motorLaeuft, bool &richtungVorwaerts, unsigned long &letzterStepZeit,
  long &stepZaehler, bool &endstopBeruehrt, bool &homingCompleted,
  long maxSteps,
  unsigned long &letzteTastenZeit,
  bool dirUmgekehrt  // true = DIR umgekehrt für Ellbogen
) {
  bool endstopAktiv = (digitalRead(endstopPin) == LOW);
  
  // Homing: Erste Endstop-Berührung erkennen
  if (!endstopBeruehrt && endstopAktiv) {
    endstopBeruehrt = true;
    homingCompleted = true;
    stepZaehler = 0;
    Serial.print("🎯 ");
    Serial.print(achsenName);
    Serial.println(" - HOMING ABGESCHLOSSEN - Position auf 0 gesetzt");
  }
  
  // Reset-Funktion: Vorwärts + Rückwärts gleichzeitig drücken
  if (digitalRead(vorwaertsPin) == HIGH && digitalRead(rueckwaertsPin) == HIGH) {
    if (millis() - letzteTastenZeit > entprellDelay) {
      resetHoming(achsenName, enablePin, motorLaeuft, endstopBeruehrt, homingCompleted, stepZaehler);
      letzteTastenZeit = millis();
      delay(entprellDelay);
    }
    return;
  }
  
  // Vorwärts-Knopf hat 5V bekommen
  if (digitalRead(vorwaertsPin) == HIGH && digitalRead(rueckwaertsPin) == LOW) {
    if (!motorLaeuft && (millis() - letzteTastenZeit) > entprellDelay) {
      if (homingCompleted && stepZaehler >= maxSteps) {
        Serial.print("❌ ");
        Serial.print(achsenName);
        Serial.println(" - MAXIMUM ERREICHT - Keine Vorwärtsbewegung möglich!");
        letzteTastenZeit = millis();
        return;
      }
      
      richtungVorwaerts = true;
      if (dirUmgekehrt) {
        // ELLBOGEN: Vorwärts = HIGH (umgekehrt)
        digitalWrite(dirPin, HIGH);
      } else {
        // BASIS/OBERARM: Vorwärts = HIGH (normal)
        digitalWrite(dirPin, HIGH);
      }
      motorStarten(enablePin, motorLaeuft, letzterStepZeit);
      
      Serial.print("➡️  ");
      Serial.print(achsenName);
      Serial.print(" Vorwärts");
      if (!homingCompleted) {
        Serial.println(" (FREIFAHRT MODUS)");
      } else {
        Serial.println();
      }
      letzteTastenZeit = millis();
    }
  }
  
  // Rückwärts-Knopf hat 5V bekommen
  if (digitalRead(rueckwaertsPin) == HIGH && digitalRead(vorwaertsPin) == LOW) {
    if (!motorLaeuft && (millis() - letzteTastenZeit) > entprellDelay) {
      richtungVorwaerts = false;
      if (dirUmgekehrt) {
        // ELLBOGEN: Rückwärts = LOW (umgekehrt)
        digitalWrite(dirPin, LOW);
      } else {
        // BASIS/OBERARM: Rückwärts = LOW (normal)
        digitalWrite(dirPin, LOW);
      }
      motorStarten(enablePin, motorLaeuft, letzterStepZeit);
      
      Serial.print("⬅️  ");
      Serial.print(achsenName);
      Serial.print(" Rückwärts");
      if (!homingCompleted) {
        Serial.println(" (FREIFAHRT MODUS)");
      } else {
        Serial.println();
      }
      letzteTastenZeit = millis();
    }
  }
  
  // Wenn Endstop aktiv ist und Rückwärts gedrückt wird
  if (digitalRead(rueckwaertsPin) == HIGH && endstopAktiv && homingCompleted && digitalRead(vorwaertsPin) == LOW) {
    if (motorLaeuft && !richtungVorwaerts) {
      stepZaehler = 0;
      Serial.print("🏠 ");
      Serial.print(achsenName);
      Serial.println(" - Zur Home-Position zurückgesetzt");
    }
  }
  
  // Wenn keine 5V mehr an den Richtungs-Pins, Motor stoppen
  if (digitalRead(vorwaertsPin) == LOW && digitalRead(rueckwaertsPin) == LOW && motorLaeuft) {
    motorStoppen(enablePin, motorLaeuft);
    Serial.print("⏹️  ");
    Serial.print(achsenName);
    if (!homingCompleted) {
      Serial.println(" gestoppt (FREIFAHRT MODUS)");
    } else {
      Serial.println(" gestoppt");
    }
  }
  
  // Motorsteuerung
  if (motorLaeuft) {
    motorSteuern(stepPin, letzterStepZeit, richtungVorwaerts, stepZaehler, homingCompleted, maxSteps, achsenName, enablePin, motorLaeuft);
  }
}

void resetHoming(const char* achsenName, int enablePin, bool &motorLaeuft, bool &endstopBeruehrt, bool &homingCompleted, long &stepZaehler) {
  endstopBeruehrt = false;
  homingCompleted = false;
  stepZaehler = 0;
  if (motorLaeuft) {
    motorStoppen(enablePin, motorLaeuft);
  }
  Serial.print("🔄 ");
  Serial.print(achsenName);
  Serial.println(" - FREIFAHRT MODUS aktiviert - Homing zurückgesetzt");
}

void motorStarten(int enablePin, bool &motorLaeuft, unsigned long &letzterStepZeit) {
  digitalWrite(enablePin, LOW);
  motorLaeuft = true;
  letzterStepZeit = micros();
}

void motorStoppen(int enablePin, bool &motorLaeuft) {
  motorLaeuft = false;
  digitalWrite(enablePin, HIGH);
}

void motorSteuern(int stepPin, unsigned long &letzterStepZeit, bool richtungVorwaerts, long &stepZaehler, bool homingCompleted, long maxSteps, const char* achsenName, int enablePin, bool &motorLaeuft) {
  unsigned long aktuelleZeit = micros();
  
  if (aktuelleZeit - letzterStepZeit >= stepIntervall) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    
    if (richtungVorwaerts) {
      stepZaehler++;
      if (homingCompleted && stepZaehler >= maxSteps) {
        motorStoppen(enablePin, motorLaeuft);
        Serial.print("🛑 ");
        Serial.print(achsenName);
        Serial.print(" - MAXIMALE POSITION ERREICHT (");
        Serial.print(maxSteps);
        Serial.println(" Steps)");
      }
    } else {
      stepZaehler--;
      if (homingCompleted && stepZaehler < 0) {
        stepZaehler = 0;
      }
    }
    
    letzterStepZeit = aktuelleZeit;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char input = Serial.read();
    if (input == 's' || input == 'S') {
      // Alle Achsen anzeigen
      zeigeAchsenStatus();
    }
    else if (input == '1') {
      // Nur Basis anzeigen
      zeigeAchsenStatus("BASIS", stepZaehler_basis, homingCompleted_basis, maxSteps_basis);
    }
    else if (input == '2') {
      // Nur Oberarm anzeigen
      zeigeAchsenStatus("OBERARM", stepZaehler_oberarm, homingCompleted_oberarm, maxSteps_oberarm);
    }
    else if (input == '3') {
      // Nur Ellbogen anzeigen
      zeigeAchsenStatus("ELLBOGEN", stepZaehler_ellbogen, homingCompleted_ellbogen, maxSteps_ellbogen);
    }
    else if (input == 'r' || input == 'R') {
      // Alle Achsen resetten
      resetHoming("BASIS", enablePin_basis, motorLaeuft_basis, endstopBeruehrt_basis, homingCompleted_basis, stepZaehler_basis);
      resetHoming("OBERARM", enablePin_oberarm, motorLaeuft_oberarm, endstopBeruehrt_oberarm, homingCompleted_oberarm, stepZaehler_oberarm);
      resetHoming("ELLBOGEN", enablePin_ellbogen, motorLaeuft_ellbogen, endstopBeruehrt_ellbogen, homingCompleted_ellbogen, stepZaehler_ellbogen);
    }
  }
}

void zeigeAchsenStatus() {
  Serial.println("=== AKTUELLER STATUS ===");
  zeigeAchsenStatus("BASIS", stepZaehler_basis, homingCompleted_basis, maxSteps_basis);
  zeigeAchsenStatus("OBERARM", stepZaehler_oberarm, homingCompleted_oberarm, maxSteps_oberarm);
  zeigeAchsenStatus("ELLBOGEN", stepZaehler_ellbogen, homingCompleted_ellbogen, maxSteps_ellbogen);
  Serial.println("========================");
}

void zeigeAchsenStatus(const char* achsenName, long stepZaehler, bool homingCompleted, long maxSteps) {
  Serial.print("📊 ");
  Serial.print(achsenName);
  Serial.print(" - Step-Zähler: ");
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
  Serial.println("---");
}