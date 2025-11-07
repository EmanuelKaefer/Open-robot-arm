// ===== PIN DEFINITIONEN =====
// Oberarm (Base) Pins
const int stepPin_oberarm = 2;
const int dirPin_oberarm = 3;
const int endstopPin_oberarm = 4;
const int vorwaertsPin_oberarm = 5;
const int rueckwaertsPin_oberarm = 6;
const int enablePin_oberarm = 8;

// Ellbogen (Elbow) Pins - beginnen bei Pin 22
const int stepPin_ellbogen = 22;
const int dirPin_ellbogen = 23;
const int endstopPin_ellbogen = 24;
const int vorwaertsPin_ellbogen = 25;
const int rueckwaertsPin_ellbogen = 26;
const int enablePin_ellbogen = 28;

// ===== MOTORPARAMETER =====
const int stepsPerRevolution = 200;
const long maxSteps_oberarm = 1700;    // Maximale Steps für Oberarm
const long maxSteps_ellbogen = 2064;   // Maximale Steps für Ellbogen

// Geschwindigkeit (Verzögerung in Mikrosekunden)
const long stepIntervall = 1500;  // 200 U/min

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
unsigned long letzteTastenZeit_oberarm = 0;
unsigned long letzteTastenZeit_ellbogen = 0;
const unsigned long entprellDelay = 50;

void setup() {
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
  // Oberarm initial DEAKTIVIEREN
  digitalWrite(enablePin_oberarm, HIGH);
  digitalWrite(dirPin_oberarm, HIGH);  // Vorwärts = HIGH
  digitalWrite(stepPin_oberarm, LOW);
  
  // Ellbogen initial DEAKTIVIEREN
  digitalWrite(enablePin_ellbogen, HIGH);
  digitalWrite(dirPin_ellbogen, LOW);  // Vorwärts = LOW für Ellbogen
  digitalWrite(stepPin_ellbogen, LOW);
  
  Serial.begin(9600);
  Serial.println("=== ROBOTER-ARM 2-ACHSEN STEUERUNG ===");
  Serial.println("=== OBERARM (BASE) ===");
  Serial.println("Vorwärts: 5V an Pin 5");
  Serial.println("Rückwärts: 5V an Pin 6");
  Serial.println("Endschalter: LOW an Pin 4");
  Serial.println("Maximale Steps: 1700");
  Serial.println("=== ELLBOGEN (ELBOW) ===");
  Serial.println("Vorwärts: 5V an Pin 25");
  Serial.println("Rückwärts: 5V an Pin 26");
  Serial.println("Endschalter: LOW an Pin 24");
  Serial.println("Maximale Steps: 2064");
  Serial.println("=== ALLGEMEIN ===");
  Serial.println("Geschwindigkeit: 200 U/min");
  Serial.println("🔄 RESET: Vorwärts + Rückwärts gleichzeitig drücken");
  Serial.println("Steps-Zählung: 's' für beide, '1' für Oberarm, '2' für Ellbogen");
  Serial.println("============================");
}

void loop() {
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
    true   // Umgekehrt für Ellbogen
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
        digitalWrite(dirPin, LOW);   // Ellbogen: Vorwärts = LOW
      } else {
        digitalWrite(dirPin, HIGH);  // Oberarm: Vorwärts = HIGH
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
        digitalWrite(dirPin, HIGH);  // Ellbogen: Rückwärts = HIGH
      } else {
        digitalWrite(dirPin, LOW);   // Oberarm: Rückwärts = LOW
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
      // Beide Achsen anzeigen
      zeigeAchsenStatus();
    }
    else if (input == '1') {
      // Nur Oberarm anzeigen
      zeigeAchsenStatus("OBERARM", stepZaehler_oberarm, homingCompleted_oberarm, maxSteps_oberarm);
    }
    else if (input == '2') {
      // Nur Ellbogen anzeigen
      zeigeAchsenStatus("ELLBOGEN", stepZaehler_ellbogen, homingCompleted_ellbogen, maxSteps_ellbogen);
    }
    else if (input == 'r' || input == 'R') {
      // Beide Achsen resetten
      resetHoming("OBERARM", enablePin_oberarm, motorLaeuft_oberarm, endstopBeruehrt_oberarm, homingCompleted_oberarm, stepZaehler_oberarm);
      resetHoming("ELLBOGEN", enablePin_ellbogen, motorLaeuft_ellbogen, endstopBeruehrt_ellbogen, homingCompleted_ellbogen, stepZaehler_ellbogen);
    }
  }
}

void zeigeAchsenStatus() {
  Serial.println("=== AKTUELLER STATUS ===");
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