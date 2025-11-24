/*
  Full integrated Arduino Mega sketch
  - 3x A4988 steppers (NEMA17)
  - Keypad on A0..A7 (4x4)
  - SSD1306 OLED (I2C SDA=20, SCL=21)
  - Physical endstops: NO -> normal HIGH, pressed LOW
  - Soft (digital) endstops per axis (EEPROM)
  - Speed (EEPROM), step-move (EEPROM)
  - Menus: axis selection in all submenus
  - Automatic motor enable/disable (EN active LOW)
*/

#include <AccelStepper.h>
#include <Keypad.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- PINS ----------
#define A1_STEP 22
#define A1_DIR  23
#define A1_EN   24

#define A2_STEP 25
#define A2_DIR  26
#define A2_EN   28
#define A2_ENDSTOP 27   // NO: HIGH normal, LOW triggered

#define A3_STEP 29
#define A3_DIR  30
#define A3_EN   32
#define A3_ENDSTOP 31   // NO: HIGH normal, LOW triggered

// ---------- Steppers ----------
AccelStepper axis1(AccelStepper::DRIVER, A1_STEP, A1_DIR);
AccelStepper axis2(AccelStepper::DRIVER, A2_STEP, A2_DIR);
AccelStepper axis3(AccelStepper::DRIVER, A3_STEP, A3_DIR);

// ---------- Keypad (A0..A7) ----------
const byte ROWS = 4;
const byte COLS = 4;
byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {A4, A5, A6, A7};

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ---------- Menus ----------
#define MENU_MAIN          0
#define MENU_MOVE_SELECT   10
#define MENU_MOVE          11
#define MENU_SPEED_SELECT  20
#define MENU_SPEED         21
#define MENU_SOFT_SELECT   30
#define MENU_SOFT          31
#define MENU_STEP_SELECT   40
#define MENU_STEP          41
#define MENU_POSITIONS     50

int currentMenu = MENU_MAIN;
int selectedAxis = 1;

// ---------- EEPROM ----------
long softEndstop[4];
long stepMoveValue;
long speedSetting;

const int ADDR_SPEED      = 0;
const int ADDR_SOFT1      = 10;
const int ADDR_SOFT2      = 20;
const int ADDR_SOFT3      = 30;
const int ADDR_STEPMOVE   = 40;

void saveLong(int addr, long val) { EEPROM.put(addr, val); }
long loadLong(int addr) { long v; EEPROM.get(addr, v); return v; }

// ---------- Positions ----------
long pos1 = 0, pos2 = 0, pos3 = 0;

// ---------- Auto EN/Disable ----------
#define MOTOR_DISABLE_TIMEOUT 5000UL
unsigned long lastMovementTime[4] = {0,0,0,0};
bool motorEnabled[4] = {false,false,false,false};

void enableMotorPin(int a) {
  if (a==1) digitalWrite(A1_EN, LOW);
  if (a==2) digitalWrite(A2_EN, LOW);
  if (a==3) digitalWrite(A3_EN, LOW);
  motorEnabled[a] = true;
  lastMovementTime[a] = millis();
}

void disableMotorPin(int a) {
  if (a==1) digitalWrite(A1_EN, HIGH);
  if (a==2) digitalWrite(A2_EN, HIGH);
  if (a==3) digitalWrite(A3_EN, HIGH);
  motorEnabled[a] = false;
}

void setupMotorEnablePins() {
  pinMode(A1_EN, OUTPUT);
  pinMode(A2_EN, OUTPUT);
  pinMode(A3_EN, OUTPUT);

  digitalWrite(A1_EN, HIGH);
  digitalWrite(A2_EN, HIGH);
  digitalWrite(A3_EN, HIGH);

  axis1.setEnablePin(A1_EN);
  axis2.setEnablePin(A2_EN);
  axis3.setEnablePin(A3_EN);

  axis1.setPinsInverted(false,false,true);
  axis2.setPinsInverted(false,false,true);
  axis3.setPinsInverted(false,false,true);

  axis1.disableOutputs();
  axis2.disableOutputs();
  axis3.disableOutputs();
}

// ---------- Helpers ----------
AccelStepper* getAxis(int a) {
  if (a==1) return &axis1;
  if (a==2) return &axis2;
  return &axis3;
}

void startRelativeMove(int a, long steps) {
  enableMotorPin(a);
  AccelStepper* ax = getAxis(a);
  ax->setMaxSpeed(speedSetting);
  ax->move(steps);
  lastMovementTime[a] = millis();
}

void startStepMove(int a, long steps) {
  enableMotorPin(a);
  AccelStepper* ax = getAxis(a);
  ax->setMaxSpeed(speedSetting);
  ax->move(steps);
  lastMovementTime[a] = millis();
}

void stopAxisImmediate(int a) {
  getAxis(a)->stop();
  lastMovementTime[a] = millis();
}

// ---------- NO Endstop check (LOW = triggered) ----------
void checkPhysicalEndstops() {
  if (digitalRead(A2_ENDSTOP) == LOW) {
    axis2.stop();
    axis2.setCurrentPosition(0);
    enableMotorPin(2);
  }

  if (digitalRead(A3_ENDSTOP) == LOW) {
    axis3.stop();
    axis3.setCurrentPosition(0);
    enableMotorPin(3);
  }
}

// ---------- EN timeout ----------
void handleMotorTimeouts() {
  unsigned long now = millis();
  if (axis1.isRunning()) lastMovementTime[1] = now;
  if (axis2.isRunning()) lastMovementTime[2] = now;
  if (axis3.isRunning()) lastMovementTime[3] = now;

  for (int a=1; a<=3; a++) {
    if (motorEnabled[a] && !getAxis(a)->isRunning()) {
      if (now - lastMovementTime[a] > MOTOR_DISABLE_TIMEOUT) {
        disableMotorPin(a);
      }
    }
  }
}

// ---------- Display ----------
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch(currentMenu) {

    case MENU_MAIN:
      display.println("Hauptmenue");
      display.println("1 Achse bewegen");
      display.println("2 Speed");
      display.println("3 Soft-Endstop");
      display.println("4 Step Move");
      display.println("5 Positionen");
      break;

    case MENU_MOVE_SELECT:
    case MENU_SPEED_SELECT:
    case MENU_SOFT_SELECT:
    case MENU_STEP_SELECT:
      display.println("Achse waehlen:");
      display.println("1: Achse 1");
      display.println("2: Achse 2");
      display.println("3: Achse 3");
      display.println("#: Zurueck");
      break;

    case MENU_MOVE:
      display.print("Achse "); display.println(selectedAxis);
      display.println("1 Vorwaerts");
      display.println("2 Rueckwaerts");
      display.println("3 STOP");
      display.println("# Zurueck");
      break;

    case MENU_SPEED:
      display.print("Speed: "); display.println(speedSetting);
      display.println("A +10");
      display.println("B -10");
      display.println("# Back");
      break;

    case MENU_SOFT:
      display.print("Soft-Endstop A"); display.println(selectedAxis);
      display.print("Wert: "); display.println(softEndstop[selectedAxis]);
      display.println("A +100");
      display.println("B -100");
      display.println("# Back");
      break;

    case MENU_STEP:
      display.print("StepMove A"); display.println(selectedAxis);
      display.print("Steps: "); display.println(stepMoveValue);
      display.println("A +10");
      display.println("B -10");
      display.println("D Move");
      display.println("# Back");
      break;

    case MENU_POSITIONS:
      display.println("Positionen:");
      display.print("A1: "); display.println(pos1);
      display.print("A2: "); display.println(pos2);
      display.print("A3: "); display.println(pos3);
      display.println("# Back");
      break;
  }

  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(A2_ENDSTOP, INPUT_PULLUP);
  pinMode(A3_ENDSTOP, INPUT_PULLUP);

  setupMotorEnablePins();

  axis1.setAcceleration(500);
  axis2.setAcceleration(500);
  axis3.setAcceleration(500);

  speedSetting = loadLong(ADDR_SPEED); 
  stepMoveValue = loadLong(ADDR_STEPMOVE);
  softEndstop[1] = loadLong(ADDR_SOFT1);
  softEndstop[2] = loadLong(ADDR_SOFT2);
  softEndstop[3] = loadLong(ADDR_SOFT3);

  if (speedSetting < 1 || speedSetting > 5000) speedSetting = 400;
  if (stepMoveValue <= 0) stepMoveValue = 200;
  if (softEndstop[1] <= 0) softEndstop[1] = 999999;
  if (softEndstop[2] <= 0) softEndstop[2] = 999999;
  if (softEndstop[3] <= 0) softEndstop[3] = 999999;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  updateDisplay();

  disableMotorPin(1);
  disableMotorPin(2);
  disableMotorPin(3);
}

// ---------- Loop ----------
void loop() {

  char k = keypad.getKey();

  if (k) {

    switch(currentMenu) {

      case MENU_MAIN:
        if (k=='1') currentMenu = MENU_MOVE_SELECT;
        if (k=='2') currentMenu = MENU_SPEED_SELECT;
        if (k=='3') currentMenu = MENU_SOFT_SELECT;
        if (k=='4') currentMenu = MENU_STEP_SELECT;
        if (k=='5') currentMenu = MENU_POSITIONS;
        break;

      case MENU_MOVE_SELECT:
        if (k>='1' && k<='3') { selectedAxis=k-'0'; currentMenu=MENU_MOVE; }
        if (k=='#') currentMenu=MENU_MAIN;
        break;

      case MENU_SPEED_SELECT:
        if (k>='1' && k<='3') { selectedAxis=k-'0'; currentMenu=MENU_SPEED; }
        if (k=='#') currentMenu=MENU_MAIN;
        break;

      case MENU_SOFT_SELECT:
        if (k>='1' && k<='3') { selectedAxis=k-'0'; currentMenu=MENU_SOFT; }
        if (k=='#') currentMenu=MENU_MAIN;
        break;

      case MENU_STEP_SELECT:
        if (k>='1' && k<='3') { selectedAxis=k-'0'; currentMenu=MENU_STEP; }
        if (k=='#') currentMenu=MENU_MAIN;
        break;

      case MENU_MOVE:
        if (k=='1') startRelativeMove(selectedAxis,+20);
        if (k=='2') startRelativeMove(selectedAxis,-20);
        if (k=='3') stopAxisImmediate(selectedAxis);
        if (k=='#') currentMenu=MENU_MOVE_SELECT;
        break;

      case MENU_SPEED:
        if (k=='A') { speedSetting+=10; saveLong(ADDR_SPEED,speedSetting); }
        if (k=='B') { speedSetting-=10; if(speedSetting<1)speedSetting=1; saveLong(ADDR_SPEED,speedSetting); }
        if (k=='#') currentMenu=MENU_SPEED_SELECT;
        break;

      case MENU_SOFT:
        if (k=='A') { softEndstop[selectedAxis]+=100; }
        if (k=='B') { softEndstop[selectedAxis]-=100; if (softEndstop[selectedAxis]<0) softEndstop[selectedAxis]=0; }
        saveLong(ADDR_SOFT1 + 10*(selectedAxis-1), softEndstop[selectedAxis]);
        if (k=='#') currentMenu=MENU_SOFT_SELECT;
        break;

      case MENU_STEP:
        if (k=='A') { stepMoveValue+=10; saveLong(ADDR_STEPMOVE, stepMoveValue); }
        if (k=='B') { stepMoveValue-=10; if(stepMoveValue<1)stepMoveValue=1; saveLong(ADDR_STEPMOVE, stepMoveValue); }
        if (k=='D') startStepMove(selectedAxis, stepMoveValue);
        if (k=='#') currentMenu=MENU_STEP_SELECT;
        break;

      case MENU_POSITIONS:
        if (k=='#') currentMenu=MENU_MAIN;
        break;
    }

    updateDisplay();
  }

  // background tasks
  checkPhysicalEndstops();

  if (axis1.isRunning() && axis1.currentPosition()>softEndstop[1]) axis1.stop();
  if (axis2.isRunning() && axis2.currentPosition()>softEndstop[2]) axis2.stop();
  if (axis3.isRunning() && axis3.currentPosition()>softEndstop[3]) axis3.stop();

  axis1.run();
  axis2.run();
  axis3.run();

  pos1=axis1.currentPosition();
  pos2=axis2.currentPosition();
  pos3=axis3.currentPosition();

  handleMotorTimeouts();
}
