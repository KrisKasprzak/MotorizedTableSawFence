
/*

  This program is a CNC fence mover for a table saw. It's purpose is to automatically set the fence distance
  from an input distance. User has control over fence parallel setting and "zeroing" out the current location
  by entering the current location

  This code or any portion of it may only be used for personal use. This code or any portion of it may not be included in an application 
  that is distributed for commercial or educational use.

*/

#include <ILI9341_t3.h>           // fast display driver lib
#include <XPT2046_Touchscreen.h>  // touch driver for a TFT display
#include <EEPROM.h>               // lib to talk to teensy EEPROM
#include <font_ArialBold.h>       // custom fonts that ships with ILI9341_t3.h
#include <font_Arial.h>           // custom fonts that ships with ILI9341_t3.h
#include <font_ArialBlack.h>      // custom fonts that ships with ILI9341_t3.h
#include <Arrow.h>                // custom font for cute little arrows
#include <D7_80_NP.h>             // custom font for large 7-segment looking position indicators
#include <ILI9341_t3_Controls.h>  // custom UI control library for better buttons and other tools
#include "Colors.h"               // custom color definition
#include "FlickerFreePrint.h"     // custom library for flicker free display on a TFT display

////////////////////////////////////////////////////////////////////////////////
//#define debug
////////////////////////////////////////////////////////////////////////////////

#define UOM_INCH 0
#define UOM_MM 1
#define DISPLAY_UPDATE 500

// fonts for various things
#define FONT_TITLE Arial_18_Bold
#define FONT_COORD D7_80_NP
#define FONT_BODY Arial_16_Bold
#define FONT_KEYPAD Arial_12_Bold
#define FONT_BUTTON Arial_18
#define FONT_ARROW Arrow

#define INCH_DIGIT_COLOR  C_LTGREEN
#define MM_DIGIT_COLOR    C_LTCYAN

#define TFT_DC  9      // DC pin on LCD
#define TFT_CS  A1     // chip select pin on LCD
#define LED_PIN A9     // lcd pin to control brightness
#define T_CS    A7
#define T_IRQ   A8

// pin defines
#define MOTOR1_STEP   1
#define MOTOR1_DIR    0
#define MOTOR1_ENABLE 2

#define MOTOR2_STEP   4
#define MOTOR2_DIR    3
#define MOTOR2_ENABLE 5

#define PULSE_DELAY   1
#define ACCEL_LIMIT   100

// defines for how far fence moves
#define STEP_PER_REV    200
#define THREAD_PER_INCH 20

int BtnX, BtnY;  // holders for screen coordinate drawing
int OptionUOM_IN, OptionUOM_MM;

byte col = 240;   // location for the setup variables
byte row = 20;    // height of each row in setup
long MaxDelay = 2000;
long MinDelay = 350; // 1700
long i;
int  StepperDelay = 0;
bool LiveUpdate = false;
float FenceDistance, Distance, DistanceToMove;
long StepsToMove = 0;
long AbsoluteSteps = 0;
float AbsolutePosition, OLDAbsolutePosition, MoveToPosition;

byte UOM = 0, OldUOM = 0;
char buf[8];
float JogFence = 0.1f;
byte Precision = 3;
float MaxFenceDistance = 24.0f;
float ParallelCorrectionJog = 0.01f;
float Overshoot = 0.2f;
uint16_t DigitColor = INCH_DIGIT_COLOR;
float ConversionFactor = 1.0f;

const uint8_t settings_icon[] = {
  // create icon in some painting program or import icon into some painting program
  // resize to your desired pixel size
  // reduce colors to 2 (inverted though)
  // uploade and get the c file https://lvgl.io/tools/imageconverter
  // 20 x 20 used here
  0x00, 0xf0, 0x00, 0x00, 0x90, 0x00,
  0x1d, 0x9b, 0x80, 0x37, 0x0e, 0xc0,
  0x20, 0x00, 0x40, 0x30, 0x00, 0xc0,
  0x11, 0xf8, 0x80, 0x33, 0x0c, 0xc0,
  0xe2, 0x04, 0x70, 0x82, 0x04, 0x10,
  0x82, 0x04, 0x10, 0xe2, 0x04, 0x70,
  0x33, 0x0c, 0xc0, 0x11, 0xf8, 0x80,
  0x30, 0x00, 0xc0, 0x20, 0x00, 0x40,
  0x37, 0x0e, 0xc0, 0x1d, 0x9b, 0x80,
  0x00, 0x90, 0x00, 0x00, 0xf0, 0x00,
};

ILI9341_t3 Display = ILI9341_t3(TFT_CS, TFT_DC);
XPT2046_Touchscreen Touch(T_CS, T_IRQ);
TS_Point p;

Button PauseBtn(&Display);
Button SetBtn(&Display);
Button SlewBtn(&Display);
Button StopBtn(&Display);
Button RedefineBtn(&Display);
Button JogInBtn(&Display);
Button JogOutBtn(&Display);
Button KeyPadBtn[16](&Display);
Button DoneBtn(&Display);
CheckBox Feedback(&Display);
SliderH StepDelay(&Display);
OptionButton OptionUOM(&Display);

FlickerFreePrint<ILI9341_t3> GText(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> CText(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> SText(&Display, C_WHITE, C_BLACK);

elapsedMillis DisplayUpdate;

void setup() {

  Serial.begin(38400);

  // while ((!Serial) || (millis() > 5000)) {}

  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_CS, OUTPUT);

  pinMode(MOTOR1_ENABLE, OUTPUT);
  pinMode(MOTOR2_ENABLE, OUTPUT);

  pinMode(MOTOR1_STEP, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_ENABLE, OUTPUT);

  pinMode(MOTOR2_STEP, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR2_ENABLE, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  digitalWrite(MOTOR1_ENABLE, LOW);
  digitalWrite(MOTOR2_ENABLE, LOW);

  GetParameters();

  // fire up the display
  Display.begin();

  delay(100);

  // create buttons and such
  JogInBtn.init(    25, 220, 40, 40, C_GREY, C_GREY, C_BLACK, C_BLACK, "L",  -10, -12, FONT_ARROW);
  SetBtn.init(      85, 220, 70, 40, C_GREY, C_YELLOW, C_BLACK, C_BLACK, "SET",  -15, -10, FONT_BUTTON);
  SlewBtn.init(    160, 220, 70, 40, C_GREY, C_GREEN, C_BLACK, C_BLACK, "GO",  -13, -10, FONT_BUTTON);
  StopBtn.init(    235, 220, 70, 40, C_GREY, C_RED, C_BLACK, C_BLACK, "STOP",  -22, -10, FONT_BUTTON);
  JogOutBtn.init(  295, 220, 40, 40, C_GREY, C_GREY, C_BLACK, C_BLACK, "R",  -10, -12, FONT_ARROW);
  RedefineBtn.init(260, 60, 90, 40, C_GREY, C_GREY, C_BLACK, C_BLACK, "SET",  -15, -10, FONT_BUTTON);
  DoneBtn.init(    260, 17, 90, 30, C_GREY, C_GREEN, C_BLACK, C_BLACK, "Done",  -15, -12, FONT_BUTTON);
  PauseBtn.init(50, 225, 93, 30, C_GREY, C_GREEN, C_BLACK, C_BLACK, "Pause",  -10, -10, FONT_BUTTON);
  OptionUOM.init(C_WHITE, C_ORANGE, C_DKGREY, C_WHITE, C_BLACK, 20, -2, FONT_BUTTON);

  Feedback.init(180, 197, C_WHITE, C_ORANGE, C_DKGREY, C_WHITE, C_BLACK, 20, 5, "Update", FONT_BUTTON );

  StepDelay.init (140, 160, 160, 350, 1250, 0, 20, C_WHITE, C_BLACK, C_ORANGE);

  JogInBtn.setCornerRadius(4);
  SetBtn.setCornerRadius(4);
  SlewBtn.setCornerRadius(4);
  StopBtn.setCornerRadius(4);
  JogOutBtn.setCornerRadius(4);
  RedefineBtn.setCornerRadius(4);
  DoneBtn.setCornerRadius(4);
  PauseBtn.setCornerRadius(4);


  OptionUOM_IN = OptionUOM.add(15, 200, "in", UOM_INCH);
  OptionUOM_MM = OptionUOM.add(90, 200, "mm", UOM_MM);

  Display.fillScreen(C_DKGREY);
  Display.setRotation(1);

  Touch.begin();
  Touch.setRotation(3);
  digitalWrite(LED_PIN, HIGH);
  
  OLDAbsolutePosition = AbsolutePosition;

  KeyPad(AbsolutePosition, 0.0, MaxFenceDistance , "Fence distance");


  AbsoluteSteps = GetAbsoluteSteps(AbsolutePosition);
  MoveToPosition = AbsolutePosition;

  if (OLDAbsolutePosition != AbsolutePosition) {
    SaveFenceLocation();
  }



  DrawMainScreen();

}

void loop() {

  if (Touch.touched()) {

    ProcessTouch();


    // did user press setup icon?
    if ( (BtnX > 285) && (BtnX < 319) && (BtnY > 0) && (BtnY < 35)) {

      // were on the option button
      drawBitmap(295, 5, settings_icon, 20, 20, C_GREY);
      delay(100);
      drawBitmap(295, 5, settings_icon, 20, 20, C_WHITE);
      delay(100);

      Settings();

      DrawMainScreen();

    }
    else if (ProcessButtonPress(SetBtn)) {
      // get some data

      OLDAbsolutePosition = AbsolutePosition;
      KeyPad(MoveToPosition, 0.0, MaxFenceDistance , "Cut width");
      if (OLDAbsolutePosition != AbsolutePosition) {
        SaveFenceLocation();
      }
      DrawMainScreen();
    }
    else if (ProcessButtonPress(SlewBtn)) {
      SetBtn.disable();
      SlewBtn.disable();
      StopBtn.enable();
      SetBtn.draw();
      SlewBtn.draw();
      StopBtn.draw();
      // got some new distacne so slew the steppers
      DistanceToMove = MoveToPosition - AbsolutePosition;
      if (DistanceToMove > 0) {
        // over slew and come back
        UpdateAbsoluteCoord(true);
        SlewFence(GetStepsToSlew(DistanceToMove + Overshoot), true);
        UpdateAbsoluteCoord(true);
        delay(500);
        SlewFence(GetStepsToSlew(-Overshoot), true);
        UpdateAbsoluteCoord(false);
      }
      else {
        UpdateAbsoluteCoord(true);
        SlewFence(GetStepsToSlew(DistanceToMove), true);
        UpdateAbsoluteCoord(false);
      }


    }

    else if (JogInBtn.press(BtnX, BtnY)) {
      // manual adjust fence in
      JogInBtn.draw(B_PRESSED);
      while (Touch.touched()) {
        Distance = -JogFence;

        SlewFence(GetStepsToSlew(Distance), false);

      }
      JogInBtn.draw(B_RELEASED);
    }
    else  if (JogOutBtn.press(BtnX, BtnY)) {
      // manually adjust fence out
      JogOutBtn.draw(B_PRESSED);
      while (Touch.touched()) {

        Distance = JogFence;

        SlewFence(GetStepsToSlew(Distance), false);
      }
      JogOutBtn.draw(B_RELEASED);
    }

    if (AbsolutePosition == MoveToPosition) {
      SlewBtn.disable();
    }
    else {
      SlewBtn.enable();
    }
    SetBtn.enable();
    StopBtn.disable();
    SetBtn.draw();
    SlewBtn.draw();
    StopBtn.draw();

  }
}


void SlewFence (long Steps, bool UpdateAbs) {

  long i = 0;

  if (Steps > 0) {

    digitalWriteFast(MOTOR1_DIR, LOW);
    digitalWriteFast(MOTOR2_DIR, LOW);

    digitalWriteFast(MOTOR1_ENABLE, HIGH);
    digitalWriteFast(MOTOR2_ENABLE, HIGH);

    for (i = 0; i < Steps; i++) {
      StepperDelay = GetStepperDelay(i, Steps);
      digitalWriteFast(MOTOR1_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWriteFast(MOTOR1_STEP, LOW);
      delayMicroseconds( StepperDelay);

      digitalWriteFast(MOTOR2_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWriteFast(MOTOR2_STEP, LOW);
      delayMicroseconds( StepperDelay);

      if (UpdateAbs) {
        AbsoluteSteps++;
      }

      if ((LiveUpdate) && (DisplayUpdate > DISPLAY_UPDATE)) {
        DisplayUpdate = 0;
        UpdateAbsoluteCoord(true);
      }


      if (Touch.touched()) {
        ProcessTouch();
        if (StopBtn.press(BtnX, BtnY)) {
          break;
        }
      }
    }
  }

  else if (Steps < 0) {

    digitalWriteFast(MOTOR1_DIR, HIGH);
    digitalWriteFast(MOTOR2_DIR, HIGH);

    digitalWriteFast(MOTOR1_ENABLE, HIGH);
    digitalWriteFast(MOTOR2_ENABLE, HIGH);

    Steps = -Steps;
    for (i = 0; i < Steps; i++) {
      StepperDelay = GetStepperDelay(i, Steps);

      digitalWriteFast(MOTOR1_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWriteFast(MOTOR1_STEP, LOW);
      delayMicroseconds( StepperDelay);

      digitalWriteFast(MOTOR2_STEP, HIGH);
      delayMicroseconds(PULSE_DELAY);
      digitalWriteFast(MOTOR2_STEP, LOW);
      delayMicroseconds( StepperDelay);
      if (UpdateAbs) {
        AbsoluteSteps--;
      }
      if ((LiveUpdate) && (DisplayUpdate > DISPLAY_UPDATE)) {
        DisplayUpdate = 0;
        UpdateAbsoluteCoord(true);
      }

      if (Touch.touched()) {
        ProcessTouch();
        if (StopBtn.press(BtnX, BtnY)) {
          break;
        }
      }
    }
  }

  digitalWriteFast(MOTOR1_ENABLE, LOW);
  digitalWriteFast(MOTOR2_ENABLE, LOW);

  SaveFenceLocation();

}



void UpdateAbsoluteCoord(bool ovr) {
  // draw absolute position
  GetAbsolutePosition(AbsoluteSteps);
  Display.setFont(FONT_COORD);
  Display.setCursor(5, 120);
  dtostrf(AbsolutePosition * ConversionFactor, 6, Precision, buf);
  CText.setTextColor(DigitColor, C_BLACK);
  if ((ovr) && (!LiveUpdate)) {
    CText.setTextColor(C_DKRED, C_BLACK);
  }
  CText.print(buf);
}

void UpdateGoalCoord(bool ovr) {
  // draw goal fence location
  Display.setFont(FONT_COORD);
  Display.setCursor(5, 40);
  dtostrf(MoveToPosition * ConversionFactor, 6, Precision, buf);
  GText.setTextColor(DigitColor, C_BLACK);
  if (ovr) {
    GText.setTextColor(C_DKRED, C_BLACK);
  }

  GText.print(buf);
}

void DrawMainScreen() {

  Display.fillScreen(C_BLACK);

  DrawHeader("Fence Mate");

  JogInBtn.resize(  25, 220, 40, 40);
  JogOutBtn.resize(295, 220, 40, 40);

  StopBtn.disable();
  if (AbsolutePosition == MoveToPosition) {
    SlewBtn.disable();
  }
  else {
    SlewBtn.enable();
  }
  JogInBtn.draw();
  SetBtn.draw();
  SlewBtn.draw();
  StopBtn.draw();
  JogOutBtn.draw();

  drawBitmap(295, 5, settings_icon, 20, 20, C_WHITE); //draw SD card icon

  UpdateGoalCoord(false);

  UpdateAbsoluteCoord(false);

}



void Settings() {

  bool KeepIn = false;
  long Steps = 0;
  long OldMinDelay = MinDelay;

  OldUOM = UOM;
  OLDAbsolutePosition = AbsolutePosition;

  DrawSettingsScreen();

  

  while (!KeepIn) {

    if (Touch.touched()) {
      ProcessTouch();

      OptionUOM.press(BtnX, BtnY);

      UOM = OptionUOM.value;

      if (OldUOM != UOM) {
        OldUOM = UOM;
       
        SetDefaults();
      }

      StepDelay.slide(BtnX, BtnY);

      if (OldMinDelay != StepDelay.value) {
        OldMinDelay = StepDelay.value;
        Display.setCursor(5, 150);
        SText.setTextColor(C_WHITE, C_BLACK);
        sprintf(buf, "Delay %d", (int) StepDelay.value);
        SText.print(buf);
        if (StepDelay.value > 700) {
          
          Feedback.enable();
        }
        else {
          Feedback.disable();
          LiveUpdate = false;
        }
        Feedback.draw(LiveUpdate);
      }

      if (Feedback.press(BtnX, BtnY)) {
       
        LiveUpdate = Feedback.value;
      }


      if (ProcessButtonPress(DoneBtn)) {
        KeepIn = true;
      }
      else if (ProcessButtonPress(RedefineBtn)) {

        KeyPad(AbsolutePosition, 0.0, 24.0, "Enter fence distance");
        AbsoluteSteps = GetAbsoluteSteps(AbsolutePosition);
        DrawSettingsScreen();
      }
      else if (JogInBtn.press(BtnX, BtnY)) {
        // slew motor 2 farther from blade
        JogInBtn.draw(B_PRESSED);
        Steps = GetStepsToSlew(ParallelCorrectionJog );
        digitalWrite(MOTOR2_ENABLE, HIGH);
        digitalWriteFast(MOTOR2_DIR, HIGH);
        for (i = 0; i < Steps; i++) {
          StepperDelay = GetStepperDelay(i, Steps);
          digitalWriteFast(MOTOR2_STEP, HIGH);
          delayMicroseconds(PULSE_DELAY);
          digitalWriteFast(MOTOR2_STEP, LOW);
          delayMicroseconds( StepperDelay);
        }
        JogInBtn.draw(B_RELEASED);
        digitalWrite(MOTOR2_ENABLE, LOW);
        delay(100); // debounce
      }
      else if (JogOutBtn.press(BtnX, BtnY)) {
        JogOutBtn.draw(B_PRESSED);
        Steps = GetStepsToSlew(ParallelCorrectionJog );
        digitalWrite(MOTOR2_ENABLE, HIGH);
        digitalWriteFast(MOTOR2_DIR, LOW);
        for (i = 0; i < Steps; i++) {
          StepperDelay = GetStepperDelay(i, Steps);
          digitalWriteFast(MOTOR2_STEP, HIGH);
          delayMicroseconds(PULSE_DELAY);
          digitalWriteFast(MOTOR2_STEP, LOW);
          delayMicroseconds( StepperDelay);
        }
        JogOutBtn.draw(B_RELEASED);
        digitalWrite(MOTOR2_ENABLE, LOW);
        delay(100); // debounce
      }
    }
  }

#ifdef debug
  Serial.println("Saving data to EEPROM ");
#endif

  EEPROM.put(30, UOM);
  LiveUpdate = Feedback.value;
  EEPROM.put(50, LiveUpdate);
  MinDelay = StepDelay.value;
  EEPROM.put(40, MinDelay);
  SaveFenceLocation();

}

void DrawSettingsScreen() {

  Display.fillScreen(C_BLACK);

  DrawHeader("Settings");

  JogInBtn.resize( 235, 113, 40, 40);
  JogOutBtn.resize(285, 113, 40, 40);

  DoneBtn.draw();
  RedefineBtn.draw();
  JogInBtn.draw();
  JogOutBtn.draw();
  OptionUOM.draw(UOM);
  StepDelay.draw(MinDelay);
  if (MinDelay > 500) {
    Feedback.enable();
  }
  else {
    Feedback.disable();
  }
  Feedback.draw(LiveUpdate);

  // set current position
  Display.setFont(FONT_BUTTON);
  Display.setTextColor(C_WHITE, C_BLACK);

  Display.setCursor(5, 50);
  Display.print(F("Current position"));

  Display.setCursor(5, 100);
  Display.print(F("Parallel fence"));

  Display.setCursor(5, 150);
  SText.setTextColor(C_WHITE, C_BLACK);
  sprintf(buf, "Delay %d", (int) MinDelay);
  SText.print(buf);

}

void DrawHeader(const char text[]) {

  Display.setFont(FONT_TITLE);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 35, C_DKBLUE);
  Display.setCursor(10 , 5 );
  Display.print(text);

}

long GetAbsoluteSteps(float inch) {
  long s = 0;

  s =  (inch * (float)(STEP_PER_REV * THREAD_PER_INCH));

#ifdef debug
  DebugPrint("GetAbsoluteSteps ");
  Serial.print("AbsoluteSteps "); Serial.println(s);
#endif

  return s;

}

long GetStepsToSlew(float inch) {
  long s = 0;

  s = (inch * (float)(STEP_PER_REV * THREAD_PER_INCH));
#ifdef debug
  DebugPrint("GetStepsToSlew ");
  Serial.print("inches to move "); Serial.println(inch);
  Serial.print("steps to move "); Serial.println(s);
#endif

  return s;
}

void GetAbsolutePosition(long steps) {

  AbsolutePosition = ((float) steps / ( STEP_PER_REV * THREAD_PER_INCH));
#ifdef debug
  DebugPrint("GetAbsolutePosition ");
  Serial.print("Absolute Position "); Serial.println(AbsolutePosition);
#endif

}

float GetDistance(long steps) {
  float d = 0;
  d = (float) steps / ( STEP_PER_REV * THREAD_PER_INCH);
#ifdef debug
  Serial.print("Distance "); Serial.println(d, 4);
#endif
  return d;
}


long GetStepperDelay(long i, long Steps) {

  int Delay = MinDelay;

  // ramp up
  if (i < ACCEL_LIMIT) {
    Delay = MaxDelay - (( i) * ((MaxDelay - MinDelay) / ACCEL_LIMIT));
  }

  // ramp down
  else if ( i > (Steps - ACCEL_LIMIT)) {
    Delay = MaxDelay - ((Steps - i) * ((MaxDelay - MinDelay) / ACCEL_LIMIT));
  }
  // full speed
  else {
    Delay = MinDelay;

  }

#ifdef debug
  Serial.print("Stepper Delay "); Serial.println(Delay);
#endif

  return Delay;

}


bool ProcessButtonPress(Button TheButton) {

  if (TheButton.press(BtnX, BtnY)) {
    TheButton.draw(B_PRESSED);
    while (Touch.touched()) {
      if (TheButton.press(BtnX, BtnY)) {
        TheButton.draw(B_PRESSED);
      }
      else {
        TheButton.draw(B_RELEASED);
        return false;
      }
      ProcessTouch();
    }

    TheButton.draw(B_RELEASED);
    return true;
  }
  return false;
}

// some day I'll write a better keypad function
// was taken from Adafruit example and made to work with floats (and decimal)
// has clear, back and cancel capability
// can use negative numbers, but since we don't allow, disable that number
// as user enter data, a char array is updated, upon done, char is converted to a float
void KeyPad(float & TheNumber, float TheMin, float TheMax, const char title[]) {

  int Left = 27;
  int Top = 5;
  int bWide = 60;
  int bHigh = 35;
  byte bSpacing = 5;
  bool hasDP = false;
  int Width, Height;
  byte Title = 50;
  int b;
  float TempNum;
  byte np = 0;
  char dn[12];
  bool KeepIn = true;

  TheNumber *= ConversionFactor;
  TheMin *= ConversionFactor;
  TheMax *= ConversionFactor;

  TempNum = TheNumber;

  Width = (4 * bWide) + (5 * bSpacing);
  Height = 5 * (bHigh +  bSpacing) + 30;

  KeyPadBtn[0].init(Left + (bWide / 2) + bSpacing + (0 * (bWide + bSpacing)), Top + 85 + (0 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "1", -5, -6, FONT_KEYPAD);
  KeyPadBtn[1].init(Left + (bWide / 2) + bSpacing + (1 * (bWide + bSpacing)), Top + 85 + (0 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "2", -5, -6, FONT_KEYPAD);
  KeyPadBtn[2].init(Left + (bWide / 2) + bSpacing + (2 * (bWide + bSpacing)), Top + 85 + (0 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "3", -5, -6, FONT_KEYPAD);
  KeyPadBtn[3].init(Left + (bWide / 2) + bSpacing + (3 * (bWide + bSpacing)), Top + 85 + (0 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_DKGREEN, C_WHITE, C_DKGREY, "Done", -10, -6, FONT_KEYPAD);

  KeyPadBtn[4].init(Left + (bWide / 2) + bSpacing + (0 * (bWide + bSpacing)), Top + 85 + (1 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "4", -5, -6, FONT_KEYPAD);
  KeyPadBtn[5].init(Left + (bWide / 2) + bSpacing + (1 * (bWide + bSpacing)), Top + 85 + (1 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "5", -5, -6, FONT_KEYPAD);
  KeyPadBtn[6].init(Left + (bWide / 2) + bSpacing + (2 * (bWide + bSpacing)), Top + 85 + (1 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "6", -5, -6, FONT_KEYPAD);
  KeyPadBtn[7].init(Left + (bWide / 2) + bSpacing + (3 * (bWide + bSpacing)), Top + 85 + (1 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "Back", -7, -6, FONT_KEYPAD);

  KeyPadBtn[8].init(Left + (bWide / 2) + bSpacing + (0 * (bWide + bSpacing)), Top + 85 + (2 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "7", -5, -6, FONT_KEYPAD);
  KeyPadBtn[9].init(Left + (bWide / 2) + bSpacing + (1 * (bWide + bSpacing)), Top + 85 + (2 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "8", -5, -6, FONT_KEYPAD);
  KeyPadBtn[10].init(Left + (bWide / 2) + bSpacing + (2 * (bWide + bSpacing)), Top + 85 + (2 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "9", -5, -6, FONT_KEYPAD);
  KeyPadBtn[11].init(Left + (bWide / 2) + bSpacing + (3 * (bWide + bSpacing)), Top + 85 + (2 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "Clear", -7, -6, FONT_KEYPAD);

  KeyPadBtn[12].init(Left + (bWide / 2) + bSpacing + (0 * (bWide + bSpacing)), Top + 85 + (3 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_GREY, "-", -5, -6, FONT_KEYPAD);
  KeyPadBtn[13].init(Left + (bWide / 2) + bSpacing + (1 * (bWide + bSpacing)), Top + 85 + (3 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "0", -5, -6, FONT_KEYPAD);
  KeyPadBtn[14].init(Left + (bWide / 2) + bSpacing + (2 * (bWide + bSpacing)), Top + 85 + (3 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, ".", -5, -6, FONT_KEYPAD);
  KeyPadBtn[15].init(Left + (bWide / 2) + bSpacing + (3 * (bWide + bSpacing)), Top + 85 + (3 * (bHigh + bSpacing)), bWide, bHigh, C_BLACK, C_WHITE, C_BLACK, C_DKGREY, "Cancel", -10, -6, FONT_KEYPAD);

  KeyPadBtn[0].setCornerRadius(4);
  KeyPadBtn[1].setCornerRadius(4);
  KeyPadBtn[2].setCornerRadius(4);
  KeyPadBtn[3].setCornerRadius(4);
  KeyPadBtn[4].setCornerRadius(4);
  KeyPadBtn[5].setCornerRadius(4);
  KeyPadBtn[6].setCornerRadius(4);
  KeyPadBtn[7].setCornerRadius(4);
  KeyPadBtn[8].setCornerRadius(4);
  KeyPadBtn[9].setCornerRadius(4);
  KeyPadBtn[10].setCornerRadius(4);
  KeyPadBtn[11].setCornerRadius(4);
  KeyPadBtn[12].setCornerRadius(4);
  KeyPadBtn[13].setCornerRadius(4);
  KeyPadBtn[14].setCornerRadius(4);
  KeyPadBtn[15].setCornerRadius(4);

  // main window
  Display.fillRect(Left, Top, Width , Height, C_GREY);
  Display.drawRect(Left, Top, Width , Height, C_BLACK);
  Display.drawRect(Left - 1, Top - 1, Width + 2 , Height + 2, C_DKGREY);
  Display.fillRect(Left, Top, Width , 30, C_BLACK);
  Display.setFont(FONT_BODY);
  Display.setCursor(Left  + 5 , Top + 5);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.print(title);
  if (UOM == UOM_INCH) {
    Display.print(F(" [in]"));
  }
  else {
    Display.print(F(" [mm]"));
  }

  // data entry
  Display.fillRect(Left + 5, Top + 35, Width - 10 , 30, C_WHITE);
  Display.drawRect(Left + 5, Top + 35, Width - 10 , 30, C_BLACK);
  Display.setCursor(Left  + 20 , Top + Title - 7);
  Display.setTextColor(C_BLACK, C_GREY);

  if (UOM == 0) {
    sprintf(dn, "%0.03f", TempNum);
  }
  else {
    sprintf(dn, "%0.02f", TempNum);
  }

  Display.print(dn);

  if (TheMin <= 0) {
    KeyPadBtn[12].disable();
  }

  for (i = 0; i < 16; i++) {
    KeyPadBtn[i].draw();
  }


  hasDP = false;
  np = 0;

  if ((int) TheNumber != TheNumber) {
    hasDP = true;
  }
  TempNum = TheNumber;

  while (KeepIn) {

    if (Touch.touched()) {
      ProcessTouch();
      // go thru all the KeyPadBtn, checking if they were pressed
      for (b = 0; b < 16; b++) {

        if (ProcessButtonPress(KeyPadBtn[b])) {
          // valid number
          if ( (b == 0) || (b == 1)  || (b == 2) || (b == 4) || (b == 5) || (b == 6 || (b == 8 || (b == 9 || (b == 10)))) ) {
            // 1 to 9
            b++;
            if (b > 3) {
              b--;
            }
            if (b > 7) {
              b--;
            }
            if (np == 0) {
              // clear and let new entry win
              strcpy(dn, "      ");
              hasDP = false;
            }

            dn[np] = (b) + '0';
            np++;

            break;
          }
          else if (b == 11) {
            // clear
            strcpy(dn, "      ");
            hasDP = false;
            np = 0;
            break;
          }
          else if (b == 13) {
            // special case the 0
            dn[np] = '0';
            np++;
            break;
          }
          else if (b == 14) {
            if (np == 0) {
              // clear and let new entry win
              strcpy(dn, "      ");
              hasDP = false;
              dn[np++] = '0';
            }

            if (!hasDP) {
              // handle decimal point
              dn[np++] = '.';
              hasDP = true;
            }
            break;
          }
          else if (b == 12) {
            // negative number
            if (dn[0] == '-') {
              for (i = 0; i < 7; i++) {
                dn[i] = dn[i + 1];
              }
            }
            else {
              for (i = 7; i > 0; i--) {
                dn[i] = dn[i - 1];
              }
              dn[0] = '-';
            }
            break;
          }
          else if (b == 7) {
            // back space
            if (np > 0) {
              if (dn[np] == '.') {
                hasDP = false;
              }
              dn[--np] = ' ';
            }
            break;
          }
          else if (b == 15) {
            // cancel
            KeepIn = false;
            break;
          }
          else if (b == 3) {
            // done
            TheNumber = atof(dn);
            if ((TheNumber < (TheMin)) || (TheNumber > (TheMax))) {
              TheNumber = TempNum;
            }
            KeepIn = false;
            break;
          }
        }
      }

      Display.fillRect(Left + 5, Top + 35, Width - 10 , 30, C_WHITE);
      Display.drawRect(Left + 5, Top + 35, Width - 10 , 30, C_BLACK);
      Display.setFont(FONT_BODY);
      Display.setCursor(Left  + 20 , Top + Title - 7);
      Display.setTextColor(C_BLACK, C_GREY);
      Display.print(dn);

    }
  }
  TheNumber /= ConversionFactor;

}

void ProcessTouch() {

  if (Touch.touched()) {
    p = Touch.getPoint();
    BtnX = p.x;
    BtnY = p.y;

#ifdef debug
    // Serial.print("real coordinates: ");
    // Serial.print(BtnX);
    // Serial.print(", ");
    // Serial.print (BtnY);
#endif

    // different displays may require reversing last 2 args
    BtnX = map(p.x, 3975, 169, 320, 0);
    BtnY = map(p.y, 3850, 304, 240, 0);

#ifdef debug
    // Serial.print(", Mapped coordinates: ");
    // Serial.print(BtnX);
    // Serial.print(", ");
    // Serial.println(BtnY);
    // Display.drawPixel(BtnX, BtnY, C_RED);
#endif

  }
}

void GetParameters() {

  byte Status = 0;
  int i = 0;

  EEPROM.get(0, Status);

  if ((Status == 255) || (Status == 0)) {
    // new unit zero everything out
    for (i = 0; i < 1000; i++) {
#ifdef debug
      Serial.print(F("Clearing EEPROM"));
#endif
      EEPROM.put(i, 0);
    }
    // now set defaults
    Status = 1;
    EEPROM.put(0, Status);
    EEPROM.put(10, Status);
    AbsolutePosition = 0.0f;
    EEPROM.put(20, AbsolutePosition);
    UOM = UOM_INCH;
    EEPROM.put(30, UOM);
    MinDelay = 350;
    EEPROM.put(40, MinDelay);
    LiveUpdate = false;
    EEPROM.put(50, LiveUpdate);
  }

  EEPROM.get(20, AbsolutePosition);
  EEPROM.get(30, UOM);
  EEPROM.get(40, MinDelay);
  EEPROM.get(50, LiveUpdate);

#ifdef debug
  Serial.print(F("AbsolutePosition: "));  Serial.println(AbsolutePosition, 3);
  Serial.print(F("UOM: "));  Serial.println(UOM, 3);
  Serial.print(F("MinDelay: "));  Serial.println(MinDelay);
  Serial.print(F("LiveUpdate: "));  Serial.println(LiveUpdate);
#endif

  SetDefaults();

}

void SetDefaults() {

  if (UOM == UOM_INCH) {
    DigitColor = INCH_DIGIT_COLOR;
    // we are working in inches, so make evertying around inches
    ConversionFactor = 1.0f;
    JogFence = 0.01f;
    Precision = 3;
    MaxFenceDistance = 24.0f;
    ParallelCorrectionJog = 0.01f;
    Overshoot = 0.2f;
  }

  else if (UOM == UOM_MM) {
    DigitColor = MM_DIGIT_COLOR;
    // we are working in inches, so make evertying around inches
    ConversionFactor = 25.4f;
    JogFence = (1.0f / ConversionFactor); // 1 mm
    Precision = 2;
    MaxFenceDistance = (600.0f / ConversionFactor); // 600 mm
    ParallelCorrectionJog = (1.0f / ConversionFactor); // 1 mm
    Overshoot = (10.0f / ConversionFactor); // 10 mm
  }
}

void SaveFenceLocation() {

  Serial.println("Saving data to EEPROM");
  GetAbsolutePosition(AbsoluteSteps);

#ifdef debug
  DebugPrint("SaveFenceLocation ");
  Serial.print("AbsolutePosition "); Serial.println(AbsolutePosition);
  Serial.print("AbsoluteSteps "); Serial.println(AbsoluteSteps);
#endif


  EEPROM.put(20, AbsolutePosition);

}



void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  byte byteWidth, j, i, sbyte = 0;

  byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7)  sbyte <<= 1;
      else sbyte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if (sbyte & 0x80) Display.drawPixel(x + i, y + j, color);
    }
  }
}

void DebugPrint(int line) {

  Serial.print("Code Executed at: ");
  Serial.println(line);

}

void DebugPrint(const char text[]) {

  Serial.print("Code Executed at: ");
  Serial.println(text);

}
