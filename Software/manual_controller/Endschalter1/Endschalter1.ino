// A4988 Stepper Motor Control mit Endschalter Toggle-Funktion
const int stepPin = 2;      // STEP Pin des A4988
const int dirPin = 3;       // DIRECTION Pin des A4988
const int enablePin = 4;    // ENABLE Pin des A4988
const int endschalterPin = 5; // Endschalter Pin (aktiv HIGH)
const int resetTastePin = 6;  // Reset-Taste Pin

bool motorAktiv = false;    // Motorstatus
bool letzterEndschalterStatus = false;
bool letzterResetStatus = false;
bool toggleGesperrt = false; // Verhindert mehrfaches Umschalten

void setup() {
  // Pin-Modi setzen
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(endschalterPin, INPUT_PULLUP);
  pinMode(resetTastePin, INPUT_PULLUP);
  
  // System initialisieren
  systemReset();
  
  Serial.begin(9600);
  Serial.println("System bereit - Endschalter toggelt Motor aktiv/schlaff");
  printStatus();
}

void loop() {
  // Endschalter Status lesen
  bool endschalterStatus = digitalRead(endschalterPin);
  
  // Reset-Taste Status lesen
  bool resetStatus = digitalRead(resetTastePin);
  
  // Prüfen auf steigende Flanke des Endschalters (gedrückt)
  if (endschalterStatus == HIGH && letzterEndschalterStatus == LOW && !toggleGesperrt) {
    toggleGesperrt = true; // Toggle sperren
    toggleMotorStatus();
    delay(500); // Entprellung
    toggleGesperrt = false; // Toggle freigeben
  }
  
  // Prüfen auf fallende Flanke der Reset-Taste (gedrückt → losgelassen)
  if (resetStatus == HIGH && letzterResetStatus == LOW) {
    systemReset();
  }
  
  letzterEndschalterStatus = endschalterStatus;
  letzterResetStatus = resetStatus;
  
  delay(10);
}

// Motorstatus umschalten
void toggleMotorStatus() {
  motorAktiv = !motorAktiv; // Status umkehren
  
  if (motorAktiv) {
    // Motor aktivieren
    digitalWrite(enablePin, LOW); // Motor aktiviert (nicht schlaff)
    Serial.println("=== MOTOR AKTIVIERT ===");
    Serial.println("Motor hält jetzt Position");
  } else {
    // Motor deaktivieren
    digitalWrite(enablePin, HIGH); // Motor deaktiviert (schlaff)
    Serial.println("=== MOTOR DEAKTIVIERT ===");
    Serial.println("Motor ist jetzt schlaff");
  }
  
  printStatus();
}

// Reset-Funktion für gesamtes System
void systemReset() {
  // Motor abschalten (schlaff)
  digitalWrite(enablePin, HIGH);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  
  // Status zurücksetzen
  motorAktiv = false;
  
  Serial.println("=== SYSTEM ZURÜCKGESETZT ===");
  Serial.println("Motor schlaff - Endschalter toggelt Motorstatus");
  printStatus();
}

// Motorsteuerungs-Funktion (nur wenn Motor aktiv)
void stepperSchritt(int schritte, int richtung) {
  if (!motorAktiv) {
    Serial.println("FEHLER: Motor ist schlaff - Endschalter drücken zum Aktivieren!");
    return;
  }
  
  Serial.print("Bewegung: ");
  Serial.print(schritte);
  Serial.println(richtung == HIGH ? " Schritte vorwärts" : " Schritte rückwärts");
  
  digitalWrite(dirPin, richtung);
  delay(10);
  
  for (int i = 0; i < schritte; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  
  Serial.println("Bewegung abgeschlossen");
}

// Serielle Schnittstelle für manuelle Steuerung
void serialEvent() {
  if (Serial.available()) {
    char befehl = Serial.read();
    
    switch(befehl) {
      case 'v': // Vorwärts
        stepperSchritt(100, HIGH);
        break;
      case 'r': // Rückwärts
        stepperSchritt(100, LOW);
        break;
      case 't': // Toggle Motorstatus (wie Endschalter)
        toggleMotorStatus();
        break;
      case 's': // Status
        printStatus();
        break;
      case 'x': // Reset über Serial
        systemReset();
        break;
      case '1': // 10 Schritte
        stepperSchritt(10, HIGH);
        break;
      case '2': // 50 Schritte
        stepperSchritt(50, HIGH);
        break;
    }
  }
}

// Systemstatus anzeigen
void printStatus() {
  Serial.println("=== SYSTEMSTATUS ===");
  Serial.print("Motor aktiv: ");
  Serial.println(motorAktiv ? "JA (hält Position)" : "NEIN (schlaff)");
  Serial.print("Endschalter: ");
  Serial.println(digitalRead(endschalterPin) == HIGH ? "AKTIV" : "INAKTIV");
  Serial.print("Reset-Taste: ");
  Serial.println(digitalRead(resetTastePin) == LOW ? "GEDRÜCKT" : "NICHT GEDRÜCKT");
  Serial.println("Befehle: v=vor(100), r=rück(100), 1=vor(10), 2=vor(50)");
  Serial.println("         t=toggle, s=status, x=reset");
  Serial.println("====================");
}
