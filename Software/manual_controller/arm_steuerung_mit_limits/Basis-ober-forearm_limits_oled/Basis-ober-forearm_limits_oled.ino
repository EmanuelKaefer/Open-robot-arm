#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>

// ===== SSD1306 I2C DISPLAY KONFIGURATION =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== KEYPAD KONFIGURATION FÜR MEGA =====
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {A4, A5, A6, A7};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ===== ROBOTER ARM PIN DEFINITIONEN =====
// Basis (Rotation) Pins - KEINE Endstops
const int stepPin_basis = 22;
const int dirPin_basis = 23;
const int enablePin_basis = 24;

// Oberarm Pins - MIT Endstops
const int stepPin_oberarm = 25;
const int dirPin_oberarm = 26;
const int endstopPin_oberarm = 27;
const int enablePin_oberarm = 28;

// Ellbogen Pins - MIT Endstops
const int stepPin_ellbogen = 29;
const int dirPin_ellbogen = 30;
const int endstopPin_ellbogen = 31;
const int enablePin_ellbogen = 32;

// ===== MOTORPARAMETER =====
const int stepsPerRevolution = 200;
const long maxSteps_basis = 1700;
const long maxSteps_oberarm = 1700;
const long maxSteps_ellbogen = 2064;
long stepIntervall = 1000;

// ===== VARIABLEN FÜR ROBOTER =====
bool motorLaeuft_basis = false;
bool motorLaeuft_oberarm = false;
bool motorLaeuft_ellbogen = false;
bool richtungVorwaerts_basis = true;
bool richtungVorwaerts_oberarm = true;
bool richtungVorwaerts_ellbogen = true;
long stepZaehler_basis = 0;
long stepZaehler_oberarm = 0;
long stepZaehler_ellbogen = 0;
bool endstopBeruehrt_oberarm = false;
bool endstopBeruehrt_ellbogen = false;
bool homingCompleted_oberarm = false;
bool homingCompleted_ellbogen = false;

// ===== MENÜZUSTÄNDE =====
enum MenuZustand {
  HAUPTMENU,
  ACHSE_AUSWAHL,
  ACHSE_STEUERUNG,
  GESCHWINDIGKEIT,
  STATUS_ANZEIGE
};
MenuZustand aktuellerZustand = HAUPTMENU;

// ===== ACHSENAUSWAHL =====
enum Achsen {
  BASIS,
  OBERARM,
  ELLBOGEN
};
Achsen aktuelleAchse = BASIS;
String achsenNamen[3] = {"BASIS", "OBERARM", "ELLBOGEN"};

// ===== GESCHWINDIGKEITSSTUFEN =====
const long geschwindigkeiten[] = {3000, 2000, 1500, 1000, 500};
const String geschwindigkeitNamen[] = {"LANGSAM", "MITTEL", "SCHNELL", "SEHR SCHNELL", "MAXIMAL"};
int aktuelleGeschwindigkeit = 2;

// ===== TIMING VARIABLEN =====
unsigned long letzteTastenZeit = 0;
const unsigned long entprellDelay = 200;
unsigned long letzterStepZeit_basis = 0;
unsigned long letzterStepZeit_oberarm = 0;
unsigned long letzterStepZeit_ellbogen = 0;

void setup() {
  // Serial beginnen
  Serial.begin(9600);
  
  // SSD1306 Display initialisieren
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 Allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Startbildschirm anzeigen
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.println("ROBOTER");
  display.setCursor(20, 35);
  display.println("SYSTEM");
  display.display();
  
  // ===== ROBOTER PIN KONFIGURATION =====
  // Basis Pins - KEINE Endstops
  pinMode(stepPin_basis, OUTPUT);
  pinMode(dirPin_basis, OUTPUT);
  pinMode(enablePin_basis, OUTPUT);
  
  // Oberarm Pins - MIT Endstops
  pinMode(stepPin_oberarm, OUTPUT);
  pinMode(dirPin_oberarm, OUTPUT);
  pinMode(enablePin_oberarm, OUTPUT);
  pinMode(endstopPin_oberarm, INPUT_PULLUP);
  
  // Ellbogen Pins - MIT Endstops
  pinMode(stepPin_ellbogen, OUTPUT);
  pinMode(dirPin_ellbogen, OUTPUT);
  pinMode(enablePin_ellbogen, OUTPUT);
  pinMode(endstopPin_ellbogen, INPUT_PULLUP);
  
  // Motoren initial DEAKTIVIEREN (Enable = HIGH)
  digitalWrite(enablePin_basis, HIGH);
  digitalWrite(enablePin_oberarm, HIGH);
  digitalWrite(enablePin_ellbogen, HIGH);
  
  // Direction Pins setzen
  digitalWrite(dirPin_basis, HIGH);
  digitalWrite(dirPin_oberarm, HIGH);
  digitalWrite(dirPin_ellbogen, LOW);
  
  delay(2000);
  zeigeHauptmenu();
  
  Serial.println("=== 3-ACHSEN ROBOTER MIT SSD1306 ===");
}

void loop() {
  // Tastatureingabe verarbeiten
  char taste = keypad.getKey();
  
  if (taste && (millis() - letzteTastenZeit > entprellDelay)) {
    letzteTastenZeit = millis();
    verarbeiteTaste(taste);
  }
  
  // Roboter Motoren steuern
  steuereRoboterMotoren();
  
  // Endstopps überprüfen (nur Oberarm und Ellbogen)
  pruefeEndstopps();
}

void pruefeEndstopps() {
  // Oberarm Endstopp
  bool endstopOberarm = (digitalRead(endstopPin_oberarm) == LOW);
  if (!endstopBeruehrt_oberarm && endstopOberarm) {
    endstopBeruehrt_oberarm = true;
    homingCompleted_oberarm = true;
    stepZaehler_oberarm = 0;
    Serial.println("🎯 OBERARM - HOMING ABGESCHLOSSEN");
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OBERARM HOME");
    display.setCursor(0, 20);
    display.println("Position: 0");
    display.display();
    delay(1000);
    
    if (aktuellerZustand == ACHSE_STEUERUNG && aktuelleAchse == OBERARM) {
      zeigeAchseSteuerung();
    }
  }
  
  // Ellbogen Endstopp
  bool endstopEllbogen = (digitalRead(endstopPin_ellbogen) == LOW);
  if (!endstopBeruehrt_ellbogen && endstopEllbogen) {
    endstopBeruehrt_ellbogen = true;
    homingCompleted_ellbogen = true;
    stepZaehler_ellbogen = 0;
    Serial.println("🎯 ELLBOGEN - HOMING ABGESCHLOSSEN");
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ELLBOGEN HOME");
    display.setCursor(0, 20);
    display.println("Position: 0");
    display.display();
    delay(1000);
    
    if (aktuellerZustand == ACHSE_STEUERUNG && aktuelleAchse == ELLBOGEN) {
      zeigeAchseSteuerung();
    }
  }
  
  // Endstopp Schutz
  if (endstopOberarm && motorLaeuft_oberarm && !richtungVorwaerts_oberarm) {
    stoppeAchseManuell(OBERARM);
    Serial.println("🛑 OBERARM - ENDSTOPP AKTIVIERT");
  }
  
  if (endstopEllbogen && motorLaeuft_ellbogen && !richtungVorwaerts_ellbogen) {
    stoppeAchseManuell(ELLBOGEN);
    Serial.println("🛑 ELLBOGEN - ENDSTOPP AKTIVIERT");
  }
}

void steuereRoboterMotoren() {
  // Basis Motorsteuerung
  if (motorLaeuft_basis) {
    if (richtungVorwaerts_basis && stepZaehler_basis >= maxSteps_basis) {
      stoppeAchseManuell(BASIS);
      Serial.println("🛑 BASIS - MAXIMUM ERREICHT");
    } else {
      motorSteuern(stepPin_basis, letzterStepZeit_basis, stepZaehler_basis, richtungVorwaerts_basis);
    }
  }
  
  // Oberarm Motorsteuerung
  if (motorLaeuft_oberarm) {
    if (richtungVorwaerts_oberarm && homingCompleted_oberarm && stepZaehler_oberarm >= maxSteps_oberarm) {
      stoppeAchseManuell(OBERARM);
      Serial.println("🛑 OBERARM - MAXIMUM ERREICHT");
    } else {
      motorSteuern(stepPin_oberarm, letzterStepZeit_oberarm, stepZaehler_oberarm, richtungVorwaerts_oberarm);
    }
  }
  
  // Ellbogen Motorsteuerung
  if (motorLaeuft_ellbogen) {
    if (richtungVorwaerts_ellbogen && homingCompleted_ellbogen && stepZaehler_ellbogen >= maxSteps_ellbogen) {
      stoppeAchseManuell(ELLBOGEN);
      Serial.println("🛑 ELLBOGEN - MAXIMUM ERREICHT");
    } else {
      motorSteuern(stepPin_ellbogen, letzterStepZeit_ellbogen, stepZaehler_ellbogen, richtungVorwaerts_ellbogen);
    }
  }
}

// ===== MENÜ FUNKTIONEN =====
void verarbeiteTaste(char taste) {
  Serial.print("Taste: ");
  Serial.println(taste);
  
  switch (aktuellerZustand) {
    case HAUPTMENU: verarbeiteHauptmenu(taste); break;
    case ACHSE_AUSWAHL: verarbeiteAchseAuswahl(taste); break;
    case ACHSE_STEUERUNG: verarbeiteAchseSteuerung(taste); break;
    case GESCHWINDIGKEIT: verarbeiteGeschwindigkeit(taste); break;
    case STATUS_ANZEIGE: verarbeiteStatus(taste); break;
  }
}

void verarbeiteHauptmenu(char taste) {
  switch (taste) {
    case 'A': aktuellerZustand = ACHSE_AUSWAHL; zeigeAchseAuswahl(); break;
    case 'B': aktuellerZustand = GESCHWINDIGKEIT; zeigeGeschwindigkeitMenu(); break;
    case 'C': fahreHomeAktuelleAchse(); break;
    case '1': aktuelleAchse = BASIS; aktuellerZustand = ACHSE_STEUERUNG; zeigeAchseSteuerung(); break;
    case '2': aktuelleAchse = OBERARM; aktuellerZustand = ACHSE_STEUERUNG; zeigeAchseSteuerung(); break;
    case '3': aktuelleAchse = ELLBOGEN; aktuellerZustand = ACHSE_STEUERUNG; zeigeAchseSteuerung(); break;
    case '#': aktuellerZustand = STATUS_ANZEIGE; zeigeStatus(); break;
    case '0': stopAlleMotoren(); break;
    case '*': zeigeHauptmenu(); break;
  }
}

void verarbeiteAchseAuswahl(char taste) {
  switch (taste) {
    case '1': aktuelleAchse = BASIS; break;
    case '2': aktuelleAchse = OBERARM; break;
    case '3': aktuelleAchse = ELLBOGEN; break;
    case 'D': aktuellerZustand = HAUPTMENU; zeigeHauptmenu(); return;
  }
  aktuellerZustand = ACHSE_STEUERUNG;
  zeigeAchseSteuerung();
}

void verarbeiteAchseSteuerung(char taste) {
  switch (taste) {
    case '4': starteAchseMenu(true); break;
    case '7': starteAchseMenu(false); break;
    case '0': stoppeAchseMenu(); break;
    case 'C': fahreHomeAktuelleAchse(); break;
    case 'D': aktuellerZustand = HAUPTMENU; zeigeHauptmenu(); break;
  }
}

void verarbeiteGeschwindigkeit(char taste) {
  switch (taste) {
    case '1': aktuelleGeschwindigkeit = 0; break;
    case '2': aktuelleGeschwindigkeit = 1; break;
    case '3': aktuelleGeschwindigkeit = 2; break;
    case '4': aktuelleGeschwindigkeit = 3; break;
    case '5': aktuelleGeschwindigkeit = 4; break;
    case 'D': aktuellerZustand = HAUPTMENU; zeigeHauptmenu(); return;
  }
  stepIntervall = geschwindigkeiten[aktuelleGeschwindigkeit];
  zeigeGeschwindigkeitAenderung();
  delay(1000);
  zeigeGeschwindigkeitMenu();
}

void verarbeiteStatus(char taste) {
  if (taste == 'D' || taste == 'A' || taste == '*') {
    aktuellerZustand = HAUPTMENU;
    zeigeHauptmenu();
  }
}

// ===== SSD1306 DISPLAY FUNKTIONEN =====
void zeigeHauptmenu() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("3-ACHSEN ROBOTER");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 15);
  display.println("A:Achse  B:Speed");
  display.setCursor(0, 25);
  display.println("C:Home   #:Status");
  
  display.setCursor(0, 40);
  display.println("1:Basis 2:Ober");
  display.setCursor(0, 50);
  display.println("3:Ellb  0:Stop");
  
  display.display();
}

void zeigeAchseAuswahl() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("ACHSEN AUSWAHL");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 20);
  display.println("1: BASIS (Rotation)");
  display.setCursor(0, 35);
  display.println("2: OBERARM");
  display.setCursor(0, 50);
  display.println("3: ELLBOGEN");
  
  display.display();
}

void zeigeAchseSteuerung() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("ACHS: " + achsenNamen[aktuelleAchse]);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Position anzeigen
  long position = 0;
  String homeStatus = "FREE";
  
  switch(aktuelleAchse) {
    case BASIS: 
      position = stepZaehler_basis;
      homeStatus = "FREE";
      break;
    case OBERARM: 
      position = stepZaehler_oberarm;
      homeStatus = homingCompleted_oberarm ? "OK" : "NO";
      break;
    case ELLBOGEN: 
      position = stepZaehler_ellbogen;
      homeStatus = homingCompleted_ellbogen ? "OK" : "NO";
      break;
  }
  
  display.setCursor(0, 20);
  display.println("Position: " + String(position));
  display.setCursor(0, 30);
  display.println("Home: " + homeStatus);
  
  display.setCursor(0, 45);
  display.println("4:VOR  7:ZURUECK");
  display.setCursor(0, 55);
  display.println("0:STOP D:ZURUCK");
  
  display.display();
}

void zeigeGeschwindigkeitMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("GESCHWINDIGKEIT");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  for(int i = 0; i < 5; i++) {
    display.setCursor(0, 15 + (i * 10));
    String marker = (i == aktuelleGeschwindigkeit) ? "> " : "  ";
    display.println(marker + String(i+1) + ": " + geschwindigkeitNamen[i]);
  }
  
  display.setCursor(0, 55);
  display.println("D: Zurueck");
  
  display.display();
}

void zeigeGeschwindigkeitAenderung() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("GESCHWINDIGKEIT");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(10, 30);
  display.println(geschwindigkeitNamen[aktuelleGeschwindigkeit]);
  
  display.display();
}

void zeigeStatus() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.println("STATUS 3-ACHSEN");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 15);
  display.println("BASIS:    " + String(stepZaehler_basis));
  display.setCursor(0, 25);
  display.println("OBERARM:  " + String(stepZaehler_oberarm));
  display.setCursor(0, 35);
  display.println("ELLBOGEN: " + String(stepZaehler_ellbogen));
  
  display.setCursor(0, 50);
  display.println("SPEED: " + geschwindigkeitNamen[aktuelleGeschwindigkeit]);
  
  display.display();
  delay(3000);
  zeigeHauptmenu();
}

// ===== ROBOTER STEUERFUNKTIONEN (unverändert) =====
void starteAchseManuell(Achsen achse, bool vorwaerts) {
  int stepPin, dirPin, enablePin;
  bool &motorLaeuft = (achse == BASIS) ? motorLaeuft_basis : 
                     (achse == OBERARM) ? motorLaeuft_oberarm : motorLaeuft_ellbogen;
  bool &richtung = (achse == BASIS) ? richtungVorwaerts_basis : 
                  (achse == OBERARM) ? richtungVorwaerts_oberarm : richtungVorwaerts_ellbogen;
  
  if (achse == BASIS) {
    stepPin = stepPin_basis;
    dirPin = dirPin_basis;
    enablePin = enablePin_basis;
    digitalWrite(dirPin, vorwaerts ? HIGH : LOW);
  } else if (achse == OBERARM) {
    stepPin = stepPin_oberarm;
    dirPin = dirPin_oberarm;
    enablePin = enablePin_oberarm;
    digitalWrite(dirPin, vorwaerts ? HIGH : LOW);
  } else {
    stepPin = stepPin_ellbogen;
    dirPin = dirPin_ellbogen;
    enablePin = enablePin_ellbogen;
    digitalWrite(dirPin, vorwaerts ? LOW : HIGH);
  }
  
  digitalWrite(enablePin, LOW);
  richtung = vorwaerts;
  motorLaeuft = true;
  
  if (achse == BASIS) letzterStepZeit_basis = micros();
  else if (achse == OBERARM) letzterStepZeit_oberarm = micros();
  else letzterStepZeit_ellbogen = micros();
  
  Serial.println((vorwaerts ? "➡️  " : "⬅️  ") + achsenNamen[achse] + " gestartet");
}

void starteAchseMenu(bool vorwaerts) {
  starteAchseManuell(aktuelleAchse, vorwaerts);
  zeigeAchseSteuerung();
}

void stoppeAchseManuell(Achsen achse) {
  bool &motorLaeuft = (achse == BASIS) ? motorLaeuft_basis : 
                     (achse == OBERARM) ? motorLaeuft_oberarm : motorLaeuft_ellbogen;
  int enablePin = (achse == BASIS) ? enablePin_basis : 
                 (achse == OBERARM) ? enablePin_oberarm : enablePin_ellbogen;
  
  motorLaeuft = false;
  digitalWrite(enablePin, HIGH);
  Serial.println("⏹️  " + achsenNamen[achse] + " gestoppt");
}

void stoppeAchseMenu() {
  stoppeAchseManuell(aktuelleAchse);
  zeigeAchseSteuerung();
}

void motorSteuern(int stepPin, unsigned long &letzterStepZeit, long &stepZaehler, bool richtungVorwaerts) {
  unsigned long aktuelleZeit = micros();
  if (aktuelleZeit - letzterStepZeit >= stepIntervall) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    
    if (richtungVorwaerts) stepZaehler++;
    else {
      stepZaehler--;
      if (stepZaehler < 0) stepZaehler = 0;
    }
    letzterStepZeit = aktuelleZeit;
  }
}

void fahreHomeAktuelleAchse() {
  switch(aktuelleAchse) {
    case BASIS: stepZaehler_basis = 0; break;
    case OBERARM: 
      stepZaehler_oberarm = 0;
      homingCompleted_oberarm = true;
      endstopBeruehrt_oberarm = true;
      break;
    case ELLBOGEN:
      stepZaehler_ellbogen = 0;
      homingCompleted_ellbogen = true;
      endstopBeruehrt_ellbogen = true;
      break;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(achsenNamen[aktuelleAchse]);
  display.setCursor(0, 20);
  display.println("HOME POSITION");
  display.display();
  delay(1000);
  zeigeAchseSteuerung();
}

void fahreAlleHome() {
  stepZaehler_basis = 0;
  stepZaehler_oberarm = 0;
  stepZaehler_ellbogen = 0;
  homingCompleted_oberarm = true;
  homingCompleted_ellbogen = true;
  endstopBeruehrt_oberarm = true;
  endstopBeruehrt_ellbogen = true;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ALLE ACHSEN");
  display.setCursor(0, 20);
  display.println("HOME POSITION");
  display.display();
  delay(1000);
  zeigeHauptmenu();
}

void stopAlleMotoren() {
  motorLaeuft_basis = false;
  motorLaeuft_oberarm = false;
  motorLaeuft_ellbogen = false;
  digitalWrite(enablePin_basis, HIGH);
  digitalWrite(enablePin_oberarm, HIGH);
  digitalWrite(enablePin_ellbogen, HIGH);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ALLE MOTOREN");
  display.setCursor(0, 20);
  display.println("GESTOPPT");
  display.display();
  delay(1000);
  zeigeHauptmenu();
}