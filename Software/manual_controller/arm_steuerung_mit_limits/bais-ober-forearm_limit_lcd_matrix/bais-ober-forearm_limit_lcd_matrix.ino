// === Improved version of your 3-Axis Robot Arm Control Code ===
// === Arduino-IDE + Mega 2560 kompatibel ===
// === Struktur korrigiert, Initialisierung fehlerfrei ===

#include <LiquidCrystal.h>
#include <Keypad.h>

// ===== LCD CONFIGURATION =====
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ===== KEYPAD CONFIGURATION =====
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

// ===== STRUCT FOR ONE AXIS =====
// AVR-gcc unterstützt Defaultwerte im Struct nicht sicher → Konstruktor benutzen
struct Axis {
  int stepPin;
  int dirPin;
  int enablePin;
  int endstopPin;

  long maxSteps;
  long stepCount;
  bool running;
  bool directionForward;
  bool homed;
  bool endstopTriggered;

  unsigned long lastStepTime;

  // ==== KONSTRUKTOR ====
  Axis(int s, int d, int e, int end, long max)
    : stepPin(s), dirPin(d), enablePin(e), endstopPin(end),
      maxSteps(max), stepCount(0), running(false),
      directionForward(true), homed(false), endstopTriggered(false),
      lastStepTime(0UL) {}
};

// ===== AXIS OBJECTS =====
Axis axisBase(22,23,24,-1,1700);   // kein Endstop
Axis axisUpper(25,26,27,27,1700);  // Endstop an Pin 27
Axis axisElbow(29,30,31,31,2064);  // Endstop an Pin 31

Axis* axes[3] = { &axisBase, &axisUpper, &axisElbow };

// ===== ENUMS =====
enum MenuState { MAIN_MENU, SELECT_AXIS, CONTROL_AXIS, SPEED_MENU, STATUS_SCREEN };
MenuState state = MAIN_MENU;

enum AxisID { BASE, UPPER, ELBOW };
AxisID currentAxis = BASE;
String axisNames[3] = {"BASIS", "OBERARM", "ELLBOGEN"};

// ===== SPEED =====
const long speedSteps[] = {3000, 2000, 1500, 1000, 500};
const String speedNames[] = {"LANGSAM", "MITTEL", "SCHNELL", "SEHR SCHNELL", "MAXIMAL"};
int speedIndex = 2;
long stepInterval = speedSteps[speedIndex];

// ===== TIMING =====
unsigned long lastKeyTime = 0;
const unsigned long debounceDelay = 200;

// =============================================
//                SETUP
// =============================================
void setup() {
  lcd.begin(16, 2);
  lcd.print("ROBOT ARM MEGA");
  lcd.setCursor(0, 1);
  lcd.print("INIT...");

  Serial.begin(9600);

  // Configure pins of all axes
  for (int i = 0; i < 3; i++) {
    Axis* a = axes[i];

    pinMode(a->stepPin, OUTPUT);
    pinMode(a->dirPin, OUTPUT);
    pinMode(a->enablePin, OUTPUT);

    if (a->endstopPin >= 0) {
      pinMode(a->endstopPin, INPUT_PULLUP);
    }

    digitalWrite(a->enablePin, HIGH); // motors off
  }

  digitalWrite(axisBase.dirPin, HIGH);
  digitalWrite(axisUpper.dirPin, HIGH);
  digitalWrite(axisElbow.dirPin, LOW);  // reversed

  delay(1500);
  showMainMenu();
}

// =============================================
//                MAIN LOOP
// =============================================
void loop() {
  char key = keypad.getKey();
  if (key && millis() - lastKeyTime > debounceDelay) {
    lastKeyTime = millis();
    handleKey(key);
  }

  updateAxisMotion(axisBase);
  updateAxisMotion(axisUpper);
  updateAxisMotion(axisElbow);

  checkEndstops(axisUpper);
  checkEndstops(axisElbow);
}

// =============================================
//           ENDSTOP CHECKING
// =============================================
void checkEndstops(Axis &a) {
  if (a.endstopPin < 0) return;

  bool pressed = (digitalRead(a.endstopPin) == LOW);

  if (!a.endstopTriggered && pressed) {
    a.endstopTriggered = true;
    a.homed = true;
    a.stepCount = 0;

    Serial.println("Homing completed: " + getAxisName(a));
    lcd.clear();
    lcd.print(getAxisName(a));
    lcd.setCursor(0, 1);
    lcd.print("HOME POSITION");
    delay(900);
    if (state == CONTROL_AXIS) showAxisControl();
  }

  if (pressed && a.running && !a.directionForward) {
    stopAxis(a);
    Serial.println("ENDSTOP triggered, stopping");
  }
}

// =============================================
//           MOTOR MOVEMENT UPDATE
// =============================================
void updateAxisMotion(Axis &a) {
  if (!a.running) return;

  if (a.directionForward && a.homed && a.stepCount >= a.maxSteps) {
    stopAxis(a);
    Serial.println("Axis reached max: " + getAxisName(a));
    return;
  }

  unsigned long now = micros();
  if (now - a.lastStepTime >= stepInterval) {
    digitalWrite(a.stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(a.stepPin, LOW);

    if (a.directionForward) a.stepCount++;
    else if (--a.stepCount < 0) a.stepCount = 0;

    a.lastStepTime = now;
  }
}

// =============================================
//                KEY HANDLING
// =============================================
void handleKey(char key) {
  Serial.print("Key: "); Serial.println(key);

  switch (state) {
    case MAIN_MENU:      handleMainMenu(key); break;
    case SELECT_AXIS:    handleAxisSelect(key); break;
    case CONTROL_AXIS:   handleAxisControl(key); break;
    case SPEED_MENU:     handleSpeedMenu(key); break;
    case STATUS_SCREEN:  
      if (key=='D'||key=='A'||key=='*') { state=MAIN_MENU; showMainMenu(); }
      break;
  }
}

// =============================================
//             MENU HANDLERS
// =============================================
void handleMainMenu(char key) {
  switch(key) {
    case 'A': state = SELECT_AXIS; showAxisSelect(); break;
    case 'B': state = SPEED_MENU;  showSpeedMenu(); break;
    case 'C': homeAxis(*axes[currentAxis]); break;

    case '1': currentAxis = BASE;   state = CONTROL_AXIS; showAxisControl(); break;
    case '2': currentAxis = UPPER;  state = CONTROL_AXIS; showAxisControl(); break;
    case '3': currentAxis = ELBOW;  state = CONTROL_AXIS; showAxisControl(); break;

    case '#': state = STATUS_SCREEN; showStatus(); break;
    case '0': stopAllAxes(); break;
    case '*': showMainMenu(); break;
  }
}

void handleAxisSelect(char key) {
  if (key == 'D') { state = MAIN_MENU; showMainMenu(); return; }
  if (key == '1') currentAxis = BASE;
  if (key == '2') currentAxis = UPPER;
  if (key == '3') currentAxis = ELBOW;
  state = CONTROL_AXIS;
  showAxisControl();
}

void handleAxisControl(char key) {
  Axis &a = *axes[currentAxis];

  if (key=='4') startAxis(a, true);
  if (key=='7') startAxis(a, false);
  if (key=='0') stopAxis(a);
  if (key=='C') homeAxis(a);
  if (key=='D') { state=MAIN_MENU; showMainMenu(); }
}

void handleSpeedMenu(char key) {
  if (key=='D') { state = MAIN_MENU; showMainMenu(); return; }
  if (key>='1' && key<='5') speedIndex = key - '1';
  stepInterval = speedSteps[speedIndex];
  showSpeedChange();
  delay(700);
  showSpeedMenu();
}

// =============================================
//               DISPLAY FUNCTIONS
// =============================================
String getAxisName(Axis &a) {
  if (&a == &axisBase) return "BASIS";
  if (&a == &axisUpper) return "OBERARM";
  return "ELLBOGEN";
}

void showMainMenu() {
  lcd.clear();
  lcd.print("3-ACHS ROBOTER");
  lcd.setCursor(0, 1);
  lcd.print("A:Achse B:Speed");
  delay(400);

  lcd.clear();
  lcd.print("1:B 2:O 3:E 0:Stop");
  lcd.setCursor(0, 1);
  lcd.print("#:Status *:Menu");
}

void showAxisSelect() {
  lcd.clear();
  lcd.print("ACHSE WAHLEN");
  lcd.setCursor(0, 1);
  lcd.print("1:B 2:O 3:E D:Zur");
}

void showAxisControl() {
  Axis &a = *axes[currentAxis];
  lcd.clear();
  lcd.print("AXIS: " + axisNames[currentAxis]);
  lcd.setCursor(0, 1);
  lcd.print("Pos:" + String(a.stepCount));
  delay(400);

  lcd.clear();
  lcd.print("4:Vor 7:Zuruck");
  lcd.setCursor(0, 1);
  lcd.print("0:Stop C:Home D:Zur");
}

void showSpeedMenu() {
  lcd.clear();
  lcd.print("SPEED 1-5");
  lcd.setCursor(0, 1);
  lcd.print("D:Zurueck");
}

void showSpeedChange() {
  lcd.clear();
  lcd.print("SPEED:");
  lcd.setCursor(0, 1);
  lcd.print(speedNames[speedIndex]);
}

void showStatus() {
  lcd.clear();
  lcd.print("Status");
  lcd.setCursor(0, 1);
  lcd.print("B:"+String(axisBase.stepCount)+" O:"+String(axisUpper.stepCount));
  delay(2000);

  lcd.clear();
  lcd.print("E:"+String(axisElbow.stepCount));
  lcd.setCursor(0, 1);
  lcd.print("Speed:"+speedNames[speedIndex]);
  delay(2000);
  showMainMenu();
}

// =============================================
//            AXIS CONTROL FUNCTIONS
// =============================================
void startAxis(Axis &a, bool forward) {
  digitalWrite(a.enablePin, LOW); // motor ON
  a.directionForward = forward;
  digitalWrite(a.dirPin, forward ? HIGH : LOW);

  a.running = true;
  a.lastStepTime = micros();

  Serial.println("Start axis: " + getAxisName(a));
}

void stopAxis(Axis &a) {
  a.running = false;
  digitalWrite(a.enablePin, HIGH); // motor OFF
  Serial.println("Stop axis: " + getAxisName(a));
}

void homeAxis(Axis &a) {
  a.stepCount = 0;
  a.homed = true;
  a.endstopTriggered = true;

  lcd.clear();
  lcd.print(getAxisName(a));
  lcd.setCursor(0, 1);
  lcd.print("HOME SET");
  delay(800);
  showAxisControl();
}

void stopAllAxes() {
  for (int i=0;i<3;i++) stopAxis(*axes[i]);

  lcd.clear();
  lcd.print("ALLE STOP");
  lcd.setCursor(0, 1);
  lcd.print("Motoren aus");
  delay(800);
  showMainMenu();
}
