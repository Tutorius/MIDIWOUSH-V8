/* ---------------------------------------
   | MIDIWOUSH VoltoMax 8                |
   |                                     |
   | Software for Midi-Controller        |
   | USB-MIDI, 8 different Voltage-      |
   | Channels, can be configured each    |
   | for own Midi-Channel and Controller |
   |                                     |
   | Microcontroller: ESP32-S3           |
   | DISPLAY: OLED SH1106 128x64 1,4"    |
   | I2C-Switch: TCA9548A                |
   | 2 X DAC: MCP4728                    |
   | 4 X PUSH-BUTTONS                    |
   |                                     |
   | USB-MIDI BY ADAFRUIT TINY-USB       |
   |                                     |
   | COPYRIGHT HARTMUT WAGENER           |
   | TUTORIUS                            |
   ---------------------------------------
*/
#define SER 1
#define SER1 1

// #define LED17 1

// Inlcude Servo,SPI,Wire,Display,DAC,MIDIUSB,EEPROM
#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MCP4728.h>

#include "graphics.h"

//#define FRQ400KHZ 1
#define FRQ1MHZ 1

// Analog-Out aktivieren
// Voltage-Difference to lower DAC-interface-time
#define VOLTDIF 1
#define MILLIPLUS 10

#define TINYUSB 1


// Position of Activity-Circles
#define YACT 55
#define RACT 5
#define XACT 15
#define X1ACT 6

// MINMS,MAXMS -> times for a standard servo for maximum angles
#define MINMS 530
#define MAXMS 2400
// Same as Float
#define FMINMS 530.0
#define FMAXMS 2400.0

// Keypress-times
#define EXITPRESS 2000
#define LONGPRESS 500
#define REPEATPRESS 100
#define DEZIPRESS 150
#define PRELLO 50

// Display-Definitions
#define OLED_RESET -1
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Keyboard-Ports
#define KEYLEFT 6
#define KEYUP 7
#define KEYDOWN 15
#define KEYRIGHT 16

// Internal Key-Numbers
#define KeyLeft 1
#define KeyUp 2
#define KeyDown 3
#define KeyRight 4

// Port für RGB-LED
#define RGBLED 48

// Maximum channels
#define MAXCHANNELS 8

#define LENVERSION 16
#define LENDATA MAXCHANNELS* MAXPAR

// Preferences
Preferences preferences;

// USB MIDI object
#ifdef TINYUSB
Adafruit_USBD_MIDI usb_midi;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);
#endif

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO

#define FZEIL 28

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DAC
Adafruit_MCP4728 mcp1;
Adafruit_MCP4728 mcp2;

typedef struct midicontrol {
  byte channel;
  byte controller;
  byte value;
  byte pitch;
  byte velocity;
  byte flag;
  byte flag2;
} MIDICONTROL;

MIDICONTROL receive;

// Global variables
// Save old keypresses
uint8_t oldKey[4];
//Key-Ports
uint8_t keyPort[4] = { KEYLEFT, KEYUP, KEYDOWN, KEYRIGHT };

//Which keys can take Longpresses
uint8_t longMask[4] = { true, true, true, true };
//Which Keys can take Repeat-Presses
uint8_t repeatMask[4] = { false, false, false, false };
//Which keys can take Exit-Presses
uint8_t exitMask[4] = { false, false, false, false };
//Which keys can take Dezi-fastklick
uint8_t deziMask[4] = { false, true, true, false };

uint8_t ready;
uint8_t refresh;
uint32_t timer, diffTime;
uint32_t deziTimer;

uint32_t rgbtimer;

uint8_t i, j;

int8_t actPort, oldPort;

//Actual Midi-Config
uint8_t midiController[MAXCHANNELS];
uint8_t midiChannel[MAXCHANNELS];
uint8_t midiMin[MAXCHANNELS];
uint8_t midiMax[MAXCHANNELS];
uint8_t midiInvert[MAXCHANNELS];

// Voltages
uint16_t voltage[8];
uint16_t oldvoltage[8];

// Normale belegung
uint8_t voltnr[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

// Belegung geändert
// uint8_t voltnr[8] = { 3, 2, 1, 0, 7, 6, 5, 4 };

// Activity-"LEDs"
uint16_t activity[8];

uint32_t milliSave, milliSave2;
uint8_t milliflag;

//Input-Mode
int8_t inputMode;

uint8_t flag;
char s[80];

int rgbpos, rgbakt;
float rgbval;
float rgbhell;

void handleControllerChange(byte channel, byte controller, byte value) {
  receive.channel = channel;
  receive.controller = controller;
  receive.velocity=0;
  receive.pitch=0;
  receive.value = value;
  receive.flag = true;
  if (receive.flag2) receive.flag2 = false;
  else receive.flag2 = true;
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  receive.channel = channel;
  receive.velocity = velocity;
  receive.controller=0;
  receive.value=0;
  receive.flag = true;
  if (receive.flag2) receive.flag2 = false;
  else receive.flag2 = true;
}

void handleNoteOff(byte channel, byte pitch, byte velocity) {
  receive.flag = true;
  if (receive.flag2) receive.flag2 = false;
  else receive.flag2 = true;
}

// Select I2C-Bus 0...7
void selectBus(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

/* CalcVOLT(float val,uint8_t min,uint8_t max)
  Calculates the Voltage for DAC
  val: 0-127, min, max 0-100
*/
uint16_t CalcVOLT(float val, uint8_t min, uint8_t max) {
  float minf, maxf, diff;
  float p1, p2;
  float px;
  minf = 0.0;
  maxf = 4095.0;
  diff = maxf - minf;
  p1 = (float)min;
  p2 = (float)max;
  p1 = p1 * 0.01;
  p2 = p2 * 0.01;
  px = val * diff * (p2 - p1);
  if (p2 > p1)
    px = px + minf;
  else
    px = maxf + px;
  return ((uint16_t)px);
}

/* int8_t scankey(uint8_t keyNr)
    Scans the key number 'keyNr' for normal presses
    and long presses
*/
int8_t scanKey(uint8_t keyNr) {
  int8_t keyPressed1;
  // Button 'keyNr' pressed?
  keyPressed1 = -1;
  if (!digitalRead(keyPort[keyNr])) {
    // Key pressed, newly pressed?
    if (!oldKey[keyNr]) {
      // Yes, newly pressed
      timer = millis();
      diffTime = 0;
      oldKey[keyNr] = true;
      if ((!longMask[keyNr]) && (!repeatMask[keyNr]) && (!exitMask[keyNr])) {
        keyPressed1 = keyNr;
      }
      if (deziMask[keyNr]) {
        if (millis() - deziTimer <= DEZIPRESS) {
          if (millis() - deziTimer >= PRELLO) {
            keyPressed1 = keyNr | 0x20;
            deziTimer = millis();
          }
        }
      }
    } else {
      // No, not newly pressed
      if (longMask[keyNr]) {
        // Langdruck der Taste abfragen
        if (millis() - timer >= LONGPRESS) {
          keyPressed1 = keyNr | 0x40;
        }
      } else {
        if (repeatMask[keyNr]) {
          // Repeatdruck der Taste abfragen
          if (millis() - timer >= REPEATPRESS) {
            timer = millis();
            keyPressed1 = keyNr;
            oldKey[keyNr] = false;
          }
        } else {
          // Exitdruck der Tasten abfragen
          if (exitMask[keyNr]) {
            if (millis() - timer > EXITPRESS) {
              keyPressed1 = keyNr | 0x80;
              ready = true;
            }
          }
        }
      }
    }
  } else {
    // No, pressed before?
    if (oldKey[keyNr]) {
      if (millis() - timer > PRELLO) {
        oldKey[keyNr] = false;
        // Yes, pressed before
        if (deziMask[keyNr])
          deziTimer = millis();
        if (longMask[keyNr]) {
          if (keyPressed1 < 0) {
            if (millis() - timer < LONGPRESS) {
              keyPressed1 = keyNr;
            } else {
              keyPressed1 = keyNr | 0x40;
            }
          }
          timer = millis();
        } else {
          if (repeatMask[keyNr]) {
            oldKey[keyNr] = false;
          } else {
            if (exitMask[keyNr]) {
              oldKey[keyNr] = false;
            } else {
              oldKey[keyNr] = false;
            }
          }
        }
      } else {
        timer = millis();
      }
    }
  }
  return (keyPressed1);
}

/* int scanKeys()
  Scans all Keys for Key-presses
*/
int scanKeys() {
  int8_t keyPressed;
  uint8_t i;
  keyPressed = -1;
  for (i = 0; i < 4; i++) {
    keyPressed = scanKey(i);
    if (keyPressed >= 0) break;
  }
  return (keyPressed);
}

/* refreshDisplay(int mode)
  Refreshes the display
*/
void refreshDisplay(int mode) {
  uint8_t i, flag, posy;
  selectBus(2);
  display.clearDisplay();
  flag = false;
  switch (mode) {
    case 0:  // Display: Controller and Midi-Channel
      flag = false;
      for (i = 0; i < MAXCHANNELS; i++) {
        if (i != actPort) {
          if ((midiController[actPort] == midiController[i]) && (midiChannel[actPort] == midiChannel[i])) {
            flag = true;
          }
        }
      }
      if (flag) {
        display.setTextColor(SH110X_BLACK);
        display.fillRect(0, 0, 128, 64, SH110X_WHITE);
      } else {
        display.setTextColor(SH110X_WHITE);
      }
      display.setCursor(0, 0);
      display.println("Controller");
      sprintf(s, "PORT %d", actPort + 1);
      display.println(s);
      sprintf(s, "CONT %3d", midiController[actPort]);
      for (i = 5; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      sprintf(s, "CHN  %2d", midiChannel[actPort] + 1);
      for (i = 5; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      if (inputMode == 0) posy = 38;
      else posy = 54;
      if (!flag)
        display.fillCircle(117, posy, 5, SH110X_WHITE);
      else
        display.fillCircle(117, posy, 5, SH110X_BLACK);
      display.display();
      refresh = false;
      break;
    case 1:  // Display: Min and Max-Values
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 0);
      display.println("MinMax");
      sprintf(s, "PORT %d", actPort + 1);
      display.println(s);
      sprintf(s, "MIN %3d", midiMin[actPort]);
      for (i = 4; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      sprintf(s, "MAX %3d", midiMax[actPort]);
      for (i = 4; i < strlen(s); i++)
        if (s[i] == ' ') s[i] = '0';
      display.println(s);
      if (inputMode == 0) posy = 22 + 16;
      else posy = 38 + 16;
      if (!flag)
        display.fillCircle(128 - 11, posy, 5, SH110X_WHITE);
      else
        display.fillCircle(128 - 11, posy, 5, SH110X_BLACK);
      display.display();
      refresh = false;
      break;
  }
}

/* void EditMidi(int mode)
  Edits the actual Midi-Settings
*/
void EditMidi(int mode) {
  int keyPressed;
  int j;

  selectBus(2);
  display.setTextSize(2);
  do {
    if (refresh) {
      refreshDisplay(mode);
    }
    keyPressed = scanKeys();
    switch (keyPressed) {
      // Key Left Short
      case 0:
        if (actPort > 0) actPort--;
        else actPort = MAXCHANNELS - 1;
        refresh = true;
        break;
      // Key Up Short
      case 1:
        switch (mode) {
          case 0:
            switch (inputMode) {
              case 0:
                if (midiController[actPort] < 150) midiController[actPort]++;
                else midiController[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] < 15) midiChannel[actPort]++;
                else midiChannel[actPort] = 0;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode) {
              case 0:
                if (midiMin[actPort] < 100) midiMin[actPort]++;
                else midiMin[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] < 100) midiMax[actPort]++;
                else midiMax[actPort] = 0;
                refresh = true;
                break;
            }
            break;
        }
        break;
      // Key Left Short
      case 2:
        switch (mode) {
          case 0:
            switch (inputMode) {
              case 0:
                if (midiController[actPort] > 0) midiController[actPort]--;
                else midiController[actPort] = 150;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] > 0) midiChannel[actPort]--;
                else midiChannel[actPort] = 15;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode) {
              case 0:
                if (midiMin[actPort] > 0) midiMin[actPort]--;
                else midiMin[actPort] = 100;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] > 0) midiMax[actPort]--;
                else midiMax[actPort] = 100;
                refresh = true;
                break;
            }
            break;
        }
        break;
      // Key Right Short
      case 3:
        if (actPort < MAXCHANNELS - 1) actPort++;
        else actPort = 0;
        refresh = true;
        break;
      // Key up long
      case 0x41:
        inputMode = 0;
        refresh = true;
        break;
      // Key down long
      case 0x42:
        inputMode = 1;
        refresh = true;
        break;
      // Key Left Long
      case 0x40:
        ready = true;
        break;
      // Key Right Long - Learn Midi
      case 0x43:
        if (mode == 0) {
          // Delete buffered Midi-events

          selectBus(2);
          display.setCursor(0, 0);
          display.fillRect(0, 0, 128, 16, SH110X_WHITE);
          display.setTextColor(SH110X_BLACK);
          display.printf("   LEARN");
          display.display();
          j = 50;
          do {
            if (TinyUSBDevice.mounted()) {
              MIDI.read();
            }
            delay(10);
            if (!receive.flag) j--;
            receive.flag = false;
          } while (j > 0);

          receive.flag = false;
          do {
            if (TinyUSBDevice.mounted()) {
              MIDI.read();
            }

            flag = true;
            if (receive.flag) {
              midiController[actPort] = receive.controller;
              midiChannel[actPort] = receive.channel - 1;
              receive.flag = false;
              flag = false;
            }
            if (flag) {
              if (!digitalRead(KEYLEFT)) flag = false;
              if (!digitalRead(KEYRIGHT)) flag = false;
            }
          } while (flag);
          while ((!digitalRead(KEYLEFT)) || (!digitalRead(KEYRIGHT)))
            ;
          refreshDisplay(mode);
        }
        break;
      // Dezi Up
      case 0x21:
        deziTimer = millis();
        switch (mode) {
          case 0:
            switch (inputMode) {
              case 0:
                if (midiController[actPort] < 150) midiController[actPort] += 8;
                else midiController[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] < 15) midiChannel[actPort]++;
                else midiChannel[actPort] = 0;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode) {
              case 0:
                if (midiMin[actPort] < 90) midiMin[actPort] += 8;
                else midiMin[actPort] = 0;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] < 90) midiMax[actPort] += 8;
                else midiMax[actPort] = 0;
                refresh = true;
                break;
            }
            break;
        }
        break;
      // Dezi Down
      case 0x22:
        deziTimer = millis();
        switch (mode) {
          case 0:
            switch (inputMode) {
              case 0:
                if (midiController[actPort] >= 10) midiController[actPort] -= 8;
                else midiController[actPort] = 150;
                refresh = true;
                break;
              case 1:
                if (midiChannel[actPort] > 0) midiChannel[actPort]--;
                else midiChannel[actPort] = 15;
                refresh = true;
                break;
            }
            break;
          case 1:
            switch (inputMode) {
              case 0:
                if (midiMin[actPort] >= 10) midiMin[actPort] -= 8;
                else midiMin[actPort] = 100;
                refresh = true;
                break;
              case 1:
                if (midiMax[actPort] >= 10) midiMax[actPort] -= 8;
                else midiMax[actPort] = 100;
                refresh = true;
                break;
            }
            break;
        }
        break;
    }
  } while (!ready);
}

/* int ScanEditReset()
  scans Keys while performing for long keypresses
*/
int ScanEditReset() {
  int keyPressed;
  int action;
  action = 0;
  ready = false;
  keyPressed = scanKeys();
  switch (keyPressed) {
    case 0x40:
      action = 1;
      ready = true;
      break;
    case 0x43:
      action = 2;
      ready = true;
      break;
    case 0x41:
      action = 3;
      break;
    case 0x42:
      action = 4;
      break;
  }
  //  } while (!ready);
  return (action);
}

/* void setup()
  Main-program
*/
void setup() {
  float dummy;
  //uint16_t servoPos[3];

  //  midiEventPacket_t rx; ESP!!!
  int keyPressed;
  uint8_t displaystart;
  uint8_t displaymode;

#ifdef SER

  Serial.begin(115200);
#endif

  // Preferences
  preferences.begin("midiwoush", false);

  receive.flag = false;
  receive.flag2 = false;
  Wire.begin(4, 5);
#ifdef FRQ400KHZ
  Wire.setClock(400000);
#endif
#ifdef FRQ1MHZ
  Wire.setClock(1000000);
#endif

// ESP32 Tiny-USB MIDI EPS!!! MIDI
#ifdef TINYUSB
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  usb_midi.setStringDescriptor("TinyUSB MIDI");

  MIDI.begin(MIDI_CHANNEL_OMNI);

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  MIDI.setHandleControlChange(handleControllerChange);
  MIDI.setHandleNoteOn(handleNoteOn);
  // MIDI.setHandleNoteOff(handleNoteOff);
#endif

  midiController[0] = preferences.getUChar("MidiCont1", 7);
  midiChannel[0] = preferences.getUChar("MidiChan1", 0);
  midiController[1] = preferences.getUChar("MidiCont2", 7);
  midiChannel[1] = preferences.getUChar("MidiChan2", 1);
  midiController[2] = preferences.getUChar("MidiCont3", 7);
  midiChannel[2] = preferences.getUChar("MidiChan3", 2);
  midiController[3] = preferences.getUChar("MidiCont4", 7);
  midiChannel[3] = preferences.getUChar("MidiChan4", 3);
  midiController[4] = preferences.getUChar("MidiCont5", 7);
  midiChannel[4] = preferences.getUChar("MidiChan5", 4);
  midiController[5] = preferences.getUChar("MidiCont6", 7);
  midiChannel[5] = preferences.getUChar("MidiChan6", 5);
  midiController[6] = preferences.getUChar("MidiCont7", 7);
  midiChannel[6] = preferences.getUChar("MidiChan7", 6);
  midiController[7] = preferences.getUChar("MidiCont8", 7);
  midiChannel[7] = preferences.getUChar("MidiChan8", 7);
  midiMin[0] = preferences.getUChar("ValMin1", 0);
  midiMin[1] = preferences.getUChar("ValMin2", 0);
  midiMin[2] = preferences.getUChar("ValMin3", 0);
  midiMin[3] = preferences.getUChar("ValMin4", 0);
  midiMin[4] = preferences.getUChar("ValMin5", 0);
  midiMin[5] = preferences.getUChar("ValMin6", 0);
  midiMin[6] = preferences.getUChar("ValMin7", 0);
  midiMin[7] = preferences.getUChar("ValMin8", 0);
  midiMax[0] = preferences.getUChar("ValMax1", 100);
  midiMax[1] = preferences.getUChar("ValMax2", 100);
  midiMax[2] = preferences.getUChar("ValMax3", 100);
  midiMax[3] = preferences.getUChar("ValMax4", 100);
  midiMax[4] = preferences.getUChar("ValMax5", 100);
  midiMax[5] = preferences.getUChar("ValMax6", 100);
  midiMax[6] = preferences.getUChar("ValMax7", 100);
  midiMax[7] = preferences.getUChar("ValMax8", 100);

  rgbpos = 0;
  rgbval = 0;
  rgbakt = 0;
  rgbhell = 50;
  rgbtimer = millis();
  inputMode = 0;
  oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
  actPort = 0;
  delay(250);

  selectBus(2);
  display.begin(0x3C, true);
  display.setContrast(0);
  display.display();
  display.clearDisplay();
  display.display();
  //delay(2000);
  //display.fillRect(0,0,128,64,SH110X_WHITE);
  //display.display();
  //delay(1000);
  //display.fillRect(0,0,128,64,SH110X_BLACK);
  //display.display();
  //delay(1000);
  display.setTextColor(SH110X_WHITE);
  display.cp437(true);  // Use full 256 char 'Code Page 437' font
  display.clearDisplay();
  display.setTextSize(2);              // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);  // Draw white text

  display.println("INIT UNIT");
  display.display();
  delay(500);
  // #endif
  selectBus(0);
  if (!mcp1.begin(0x60)) {
    selectBus(2);
    display.printf("MCPERR1");
    display.display();
#ifdef SER1
    Serial.println("DAC1 Error");
#endif
    for (;;)
      ;  // Don't proceed, loop forever
  }
  selectBus(2);
  display.println("MCP1");
  display.display();
  delay(500);
  selectBus(1);
  if (!mcp2.begin()) {
    selectBus(2);
    display.printf("MCPERR2");
    display.display();
#ifdef SER1
    Serial.println("DAC2 Error");
#endif
    for (;;)
      ;  // Don't proceed, loop forever
  }
  selectBus(2);
  display.println("MCP2");
  display.display();
  delay(500);
#ifdef SER1
  Serial.println("DAC OK");
#endif
  delay(1000);
  selectBus(0);
  mcp1.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp1.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp1.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp1.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  selectBus(1);
  mcp2.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp2.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp2.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);
  mcp2.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);

  pinMode(KEYLEFT, INPUT_PULLUP);
  pinMode(KEYUP, INPUT_PULLUP);
  pinMode(KEYDOWN, INPUT_PULLUP);
  pinMode(KEYRIGHT, INPUT_PULLUP);

#ifdef LED17
  pinMode(17, OUTPUT);

  // Test der LED
  for (i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      digitalWrite(17, true);
    } else {
      digitalWrite(17, false);
    }
    delay(100);
  }

  digitalWrite(17, false);
#endif
  selectBus(2);
  display.display();
  display.setTextSize(2);              // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);  // Draw white text

  display.cp437(true);  // Use full 256 char 'Code Page 437' font
  display.println("RGB-LED");
  display.display();
  ready = false;
  refresh = true;
  displaystart = 1;
  displaymode = 1;
  oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
  ready = false;
  refresh = true;
  inputMode = 0;
  actPort = 0;
  for (i = 0; i < 8; i++) {
    oldvoltage[i] = voltage[i];
    activity[i] = 0;
  }
  milliSave2 = milliSave = millis();

#ifdef RGBLED
  for (j = 0; j < 3; j++) {
    for (i = 0; i < 100; i++) {
      switch (j) {
        case 0:
          rgbLedWrite(RGBLED, i, 0, 0);
          break;
        case 1:
          rgbLedWrite(RGBLED, 0, i, 0);
          break;
        case 2:
          rgbLedWrite(RGBLED, 0, 0, i);
          break;
      }
      delay(10);
    }
  }
  rgbLedWrite(RGBLED, 0, 0, 0);
#endif

  do {
    if (millis() - rgbtimer > 50)
    {
      rgbtimer = millis();
      rgbval += 1;
      rgbakt = (int)(rgbval * rgbhell / 100.0);
      switch (rgbpos) {
        case 0:  // Wird Rot aus Weiss
          rgbLedWrite(RGBLED, (int)rgbhell, (int)rgbhell - rgbakt, (int)rgbhell - rgbakt);
          break;
        case 1:  // Wird Grün aus Rot
          rgbLedWrite(RGBLED, (int)rgbhell - rgbakt, rgbakt, 0);
          break;
        case 2:  // Wird Blau aus Grün
          rgbLedWrite(RGBLED, 0, (int)rgbhell - rgbakt, rgbakt);
          break;
        case 3:  // Wird Gelb aus Blau
          rgbLedWrite(RGBLED, rgbakt, rgbakt, (int)rgbhell - rgbakt);
          break;
        case 4:  // Wird Cyan aus Gelb
          rgbLedWrite(RGBLED, (int)rgbhell - rgbakt, (int)rgbhell, rgbakt);
          break;
        case 5:  // Wird Magenta aus Cyan
          rgbLedWrite(RGBLED, rgbakt, (int)rgbhell - rgbakt, (int)rgbhell);
          break;
        case 6:  // Wird Weiss aus Magenta
          rgbLedWrite(RGBLED, (int)rgbhell, rgbakt, (int)rgbhell);
          break;
      }
      if (rgbval >= 100) {
        rgbval = 0;
        rgbpos++;
        if (rgbpos > 6) rgbpos = 0;
      }
    }

#ifdef TINYUSB_NEED_POLLING_TASK
    // Manual call tud_task since it isn't called by Core's background
    TinyUSBDevice.task();
#endif
    if (TinyUSBDevice.mounted()) {
      MIDI.read();

    }
#ifdef LED17
    else {
      for (i = 0; i < 10; i++) {
        if (i % 2 == 0) {
          digitalWrite(17, true);
        } else {
          digitalWrite(17, false);
        }
        delay(100);
      }
      digitalWrite(17, false);
    }

    digitalWrite(17, receive.flag2);
#endif
    /* if (receive.flag)
    {
      selectBus(2);
      display.fillRect(0, 0, 128,64,SH110X_BLACK );
      display.display();
      delay(1000);  
      } */
    if (displaystart > 0) {
      switch (displaymode) {
        case 1:
          selectBus(2);
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20, SH110X_WHITE);
          if (displaystart == 1) {
            display.setTextSize(1);
            display.setCursor(0, FZEIL);
            display.println("V8 Vers. 0.5");
            display.println("by Hartmut Wagener");
            display.display();
            delay(2000);
          }
          display.fillRect(0, 20, 128, 108, SH110X_BLACK);
          display.setTextSize(1);
          display.setCursor(0, FZEIL);
          display.println("L-LONG:PREFERENCES");
          display.println("R-LG:VIEW DOT/CHANNEL");
          display.display();
          displaystart = 0;
          break;
        case 2:
          selectBus(2);
          display.clearDisplay();
          display.drawBitmap(0, 0, myBitmap, 128, 20, SH110X_WHITE);
          display.setTextSize(1);
          display.setCursor(0, FZEIL);
          display.println("ESP32-Ver 0.5");
          display.println("by Hartmut Wagener");
          display.display();
          delay(2000);
          display.clearDisplay();
          for (i = 0; i < SCREEN_WIDTH; i += SCREEN_WIDTH / 8) {
            display.drawRect(i, 0, i + SCREEN_WIDTH / 8, SCREEN_WIDTH / 8, SCREEN_HEIGHT);
          }
          display.display();
          displaystart = 0;
          break;
      }
    }
    ready = false;
    keyPressed = ScanEditReset();
    ready = false;
    if (keyPressed > 0) {
      ready = false;
      do {
        if ((digitalRead(KEYLEFT)) && (digitalRead(KEYRIGHT))) ready = true;
      } while (!ready);
    }
    //oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
    switch (keyPressed) {
      case 1:  // Edit Midi-Config
        ready = false;
        refresh = true;
        inputMode = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        actPort = 0;
        EditMidi(0);
        ready = false;
        do {
          if ((digitalRead(KEYLEFT)) && (digitalRead(KEYRIGHT))) ready = true;
        } while (!ready);
        keyPressed = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        ready = false;
        refresh = true;
        inputMode = 0;
        EditMidi(1);
        ready = false;
        do {
          if ((digitalRead(KEYLEFT)) && (digitalRead(KEYRIGHT))) ready = true;
        } while (!ready);
        // Preferences abspeichern
        // ***
        preferences.putUChar("MidiCont1", midiController[0]);
        preferences.putUChar("MidiCont2", midiController[1]);
        preferences.putUChar("MidiCont3", midiController[2]);
        preferences.putUChar("MidiCont4", midiController[3]);
        preferences.putUChar("MidiCont5", midiController[4]);
        preferences.putUChar("MidiCont6", midiController[5]);
        preferences.putUChar("MidiCont7", midiController[6]);
        preferences.putUChar("MidiCont8", midiController[7]);
        preferences.putUChar("MidiChan1", midiChannel[0]);
        preferences.putUChar("MidiChan2", midiChannel[1]);
        preferences.putUChar("MidiChan3", midiChannel[2]);
        preferences.putUChar("MidiChan4", midiChannel[3]);
        preferences.putUChar("MidiChan5", midiChannel[4]);
        preferences.putUChar("MidiChan6", midiChannel[5]);
        preferences.putUChar("MidiChan7", midiChannel[6]);
        preferences.putUChar("MidiChan8", midiChannel[7]);
        preferences.putUChar("ValMin1", midiMin[0]);
        preferences.putUChar("ValMin2", midiMin[1]);
        preferences.putUChar("ValMin3", midiMin[2]);
        preferences.putUChar("ValMin4", midiMin[3]);
        preferences.putUChar("ValMin5", midiMin[4]);
        preferences.putUChar("ValMin6", midiMin[5]);
        preferences.putUChar("ValMin7", midiMin[6]);
        preferences.putUChar("ValMin8", midiMin[7]);
        preferences.putUChar("ValMax1", midiMax[0]);
        preferences.putUChar("ValMax2", midiMax[1]);
        preferences.putUChar("ValMax3", midiMax[2]);
        preferences.putUChar("ValMax4", midiMax[3]);
        preferences.putUChar("ValMax5", midiMax[4]);
        preferences.putUChar("ValMax6", midiMax[5]);
        preferences.putUChar("ValMax7", midiMax[6]);
        preferences.putUChar("ValMax8", midiMax[7]);

        keyPressed = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        ready = false;
        refresh = true;
        displaymode = 1;
        displaystart = 1;
        break;
      case 2:  // View Channels as Bars, Exit by L-Press
        if (displaymode++ == 2) displaymode = 1;
        ready = false;
        refresh = true;
        inputMode = 0;
        oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
        actPort = 0;
        // ChannelView();
        displaystart = 2;
        if (displaymode == 2)
          displaystart = 0;
        break;
      case 3:
        if (rgbhell <= 90) rgbhell = rgbhell + 10;
        if (rgbhell > 90) rgbhell = 100;
        break;
      case 4:
        if (rgbhell >= 10) rgbhell = rgbhell - 10;
        if (rgbhell < 10) rgbhell = 0;
        break;
    }
    if (keyPressed > 0) {
      ready = false;
      do {
        if ((digitalRead(KEYLEFT)) && (digitalRead(KEYRIGHT))) ready = true;
      } while (!ready);
      oldKey[0] = oldKey[1] = oldKey[2] = oldKey[3] = false;
    }
    keyPressed = 0;
    // rx=MidiUSB.read(); ESP!!!

    if (receive.flag)  // ESP!!! MIDI
    {

      receive.flag = false;
      for (i = 0; i < MAXCHANNELS; i++)
      {
        if (midiController[i] != 3)
        {
          if ((receive.channel == midiChannel[i] + 1) && (receive.controller == midiController[i])) {
            dummy = (float)receive.value;
            dummy = dummy / 127.0;
            voltage[i] = CalcVOLT(dummy, midiMin[i], midiMax[i]);
            if (activity[i] < 2)
              activity[i]++;
          }
        }
        else
        {
          if ((receive.channel == midiChannel[i]+1)&&(receive.controller==0))
          // Note lesen, Velocity ausgeben auf Servo oder Port
          {
            //ESP!!! MIDI
            dummy=(float)receive.velocity;
            dummy = dummy / 127.0;
            voltage[i] = CalcVOLT(dummy, midiMin[i], midiMax[i]);
            if(activity[i]<2)
              activity[i]++;
          }
        }
      }
      milliflag = milliSave2 + MILLIPLUS > millis();
      if (milliflag) milliSave2 = millis();
      for (i = 0; i < 4; i++) {
        //sprintf(s,"%d",voltage[i]);
        //Serial.println(s);
        switch (i) {
          case 0:
            selectBus(0);

            if ((abs(oldvoltage[voltnr[i]] - voltage[voltnr[i]]) > VOLTDIF) || (milliflag)) {
              mcp1.setChannelValue(MCP4728_CHANNEL_A, voltage[voltnr[i]]);
            }

            selectBus(1);

            if ((abs(oldvoltage[voltnr[i + 4]] - voltage[voltnr[i + 4]]) > VOLTDIF) || (milliflag)) {
              mcp2.setChannelValue(MCP4728_CHANNEL_A, voltage[voltnr[i + 4]]);
            }
            break;
          case 1:
            selectBus(0);

            if ((abs(oldvoltage[voltnr[i]] - voltage[voltnr[i]]) > VOLTDIF) || (milliflag)) {
              mcp1.setChannelValue(MCP4728_CHANNEL_B, voltage[voltnr[i]]);
            }

            selectBus(1);

            if ((abs(oldvoltage[voltnr[i + 4]] - voltage[voltnr[i + 4]]) > VOLTDIF) || (milliflag)) {
              mcp2.setChannelValue(MCP4728_CHANNEL_B, voltage[voltnr[i + 4]]);
            }
            break;
          case 2:
            selectBus(0);

            if ((abs(oldvoltage[voltnr[i]] - voltage[voltnr[i]]) > VOLTDIF) || (milliflag)) {
              mcp1.setChannelValue(MCP4728_CHANNEL_C, voltage[voltnr[i]]);
            }

            selectBus(1);

            if ((abs(oldvoltage[voltnr[i + 4]] - voltage[voltnr[i + 4]]) > VOLTDIF) || (milliflag)) {
              mcp2.setChannelValue(MCP4728_CHANNEL_C, voltage[voltnr[i + 4]]);
            }
            break;
          case 3:
            selectBus(0);

            if ((abs(oldvoltage[voltnr[i]] - voltage[voltnr[i]]) > VOLTDIF) || (milliflag)) {
              mcp1.setChannelValue(MCP4728_CHANNEL_D, voltage[voltnr[i]]);
            }

            selectBus(1);

            if ((abs(oldvoltage[voltnr[i + 4]] - voltage[voltnr[i + 4]]) > VOLTDIF) || (milliflag)) {
              mcp2.setChannelValue(MCP4728_CHANNEL_D, voltage[voltnr[i + 4]]);
            }
            break;
        }
      }
    }
    if (((displaymode == 1) && (millis() > milliSave + 500)) || ((displaymode == 2) && (millis() > milliSave + 100))) {
      milliSave = millis();
      selectBus(2);
      if (displaymode == 1)
        display.fillRect(X1ACT - RACT, YACT - RACT, 9 * XACT + 2 * RACT, 2 * RACT, SH110X_BLACK);
      else
        display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_BLACK);
      for (i = 0; i < MAXCHANNELS; i++) {
        selectBus(2);
        if (activity[i] > 0) {
          switch (displaymode) {
            case 1:
              display.fillCircle(X1ACT + XACT / 2 + i * XACT, YACT, RACT, SH110X_WHITE);
              activity[i]--;
              break;
            case 2:
              display.drawRect(i * SCREEN_WIDTH / 8, 0, SCREEN_WIDTH / 8, 64, SH110X_WHITE);
              display.fillRect(i * SCREEN_WIDTH / 8, 63, SCREEN_WIDTH / 8, -voltage[i] / 64, SH110X_WHITE);
              activity[i]--;
              break;
          }
        } else {
          switch (displaymode) {
            case 1:
              display.drawCircle(X1ACT + XACT / 2 + i * XACT, YACT, RACT, SH110X_WHITE);
              break;
            case 2:
              display.drawRect(i * SCREEN_WIDTH / 8, 0, SCREEN_WIDTH / 8, 64, SH110X_WHITE);
              display.fillRect(i * SCREEN_WIDTH / 8, 63, SCREEN_WIDTH / 8, -voltage[i] / 64, SH110X_WHITE);
              break;
          }
        }
        if (voltage[i] != oldvoltage[i]) {
          oldvoltage[i] = voltage[i];
          // display.fillCircle(X1ACT+XACT/2+i*XACT,YACT,RACT,SH110X_WHITE);
        } else {
          //            display.drawCircle(X1ACT+XACT/2+i*XACT,YACT,RACT,SH110X_WHITE);
        }
      }
      display.display();
    }
  } while (1 == 1);
}

void loop()
{

}