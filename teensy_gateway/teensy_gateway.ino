/*
 * teens_gateway
 * 
 * Simple Pi-in-the-Sky compatible LoRa receiver for displaying GPS info on a
 * Sparkfun Serial Graphic LCD.  Designed to work with trackers based on Dave
 * Ackerman's great work (http://www.daveakerman.com/).  His code can be found
 * at https://github.com/PiInTheSky and the core code this program is based on
 * at https://github.com/PiInTheSky/lora-gateway.
 * 
 * Designed for a PJRC Teensy-LC with a RFM95W (SX1276 chip) radio.
 * 
 * IO Connections
 *   D1  - Serial1 TX to Display
 *   D8  - ResetN
 *   D9  - DIO0 IRQ
 *   D10 - CSN
 *   D11 - MOSI
 *   D12 - MISO
 *   D13 - LED (receive packet indicator)
 *   D14 - SCK
 *   D15 - DIO5
 *   
 * IO Connections for second radio (currently not supported)
 *   D16 - CSN
 *   D17 - LED (receive packet indicator)
 *   D18 - DIO0
 *   D19 - DIO5
 * 
 * 
 * Display Format (128x64 pixel LCD Display with built-in 8x6 pixel characters)
 *             1         2
 *   0         0         0
 *  +----------------------+
 *  |XX.XXXXX, -XXX.XXXXX  |   -> GPS Coordinates
 *  |XXXXX M, XXXXXX F     |   -> Altitude
 *  |XXX C, XXX F          |   -> Temperature
 *  |Telem: NNNNN (TTTTs)  |   -> Number of Telemetry Packets (and time since last)
 *  |Image: NNNNN (TTTTs)  |   -> Number of SSDV Packets (and time since last)
 *  |FFF.FFFF MHz      -XXX|   -> Frequency and current RSSI
 *  |SNR: XXX  RSSI: -XXX  |   -> Current packet SNR and RSSI
 *  |FERR: XX.XXkHz     AFC|   -> Current Frequency Error and AFC enabled indicator
 *  +----------------------+
 *  
 * Uses a double-buffered character array to decouple generation of display information with
 * display update because the Sparkfun Display is pretty slow.  An upside is this makes it easy
 * to adapt this program to a different display.
 * 
 * The display brightness increases when packets are being received.
 * 
 * Data is also logged to the USB serial port.  This includes information about received packets
 * (controlled by the ENABLE_PACKET_LOGGING configuration constant) and the actual packets
 * (controlled by the ENABLE_TELEMETRY_LOGGING configuration constant) as well as program state
 * updates (e.g. frequency changes, etc).  Keypresses from the USB Serial are also monitored and
 * many of the commands that Dave's lora-gateway implemented are supported.
 * 
 * Copyright 2017 Dan Julio <dan@danjuliodesigns.com>
 * 
 * Revision History
 * ===============================================================================================
 *  1.0    Initial Release
 * 
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
#include <SparkFunSerial1GraphicLCD.h>
#include <SPI.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>


// ================================================================================================
// Radio system configuration (replaces config file)
//
#define CH0_ENABLE                 1
#define CH1_ENABLE                 0
#define CALLING_TIMEOUT            300
#define ENABLE_TELEMETRY_LOGGING   1
#define ENABLE_PACKET_LOGGING      1
#define LORA0_FREQ                 915.0
#define LORA1_FREQ                 915.0
#define LORA0_SPEED_MODE           8
#define LORA1_SPEED_MODE           8


// ================================================================================================
// Hardware interfaces
//
const int RSTN   = 8;
const int DIO0_0 = 9;
const int CSN0   = 10;
const int LED0   = 13;
const int SCLK   = 14;
const int DIO0_5 = 15;
const int CSN1   = 16;
const int LED1   = 17;
const int DIO1_0 = 18;
const int DIO1_5 = 19;


// ================================================================================================
// Necessary porting globals
//

// LED Blink timer
int LEDCounts[2];

// Main loop timing
int LoopPeriod, MSPerLoop;
unsigned long SecCount;


// ================================================================================================
// Main globals
//

// Display Instance
LCD LCD;

// LCD Display buffer and management
#define DISP_WIDTH      128
#define DISP_HEIGHT     64
#define DISP_LINE_CHARS (DISP_WIDTH/6)
#define DISP_LINES      (DISP_HEIGHT/8)
#define BUFFER_SIZE     (DISP_LINE_CHARS*DISP_LINES)
#define DISP_INACTIVE_B 25
#define DISP_ACTIVE_B   75

char DispPushBuffer[BUFFER_SIZE];
char DispPullBuffer[BUFFER_SIZE];
char DispCurBuffer[BUFFER_SIZE];
int DispPullIndex = 0;
bool DispActiveBrightness = false;

// Timer
IntervalTimer tickTimer;
volatile bool tick = false;


// ================================================================================================
// Arduino environment entry points
//
void setup() {
  LogInit();
  DispInit();
  RadioInit();

  LoopPeriod = 0;
  MSPerLoop = 10;
  SecCount = 0;
  
  ShowPacketCounts(0);
  ShowPacketCounts(1);

  tickTimer.begin(tickIsr, (int) (MSPerLoop * 1000));
}


void loop() {
  int ch;

  // Evaluate scheduled items
  if (tick) {
    tick = false;
    DispEval();
    LoopPeriod += MSPerLoop;
  }
  
  // Look for user adjustments
  if (Serial.available()) {
    ch = Serial.read();
    ProcessKeyPress(ch);
  }

  // Evalute the radio
  RadioMain();
}


void tickIsr() {
  tick = true;
}


// ================================================================================================
// Display routines
//
void DispInit() {
  LCD.clearScreen();
  delay(500);
  LCD.setBacklight(DISP_INACTIVE_B);

  for (int i=0; i<(BUFFER_SIZE); i++) {
    DispPushBuffer[i] = ' ';
    DispPullBuffer[i] = ' ';
    DispCurBuffer[i] = ' ';
  }
}


void DispString(int row, int col, char* s) {
  int len;

  if (row > (DISP_LINES-1)) row = (DISP_LINES-1);

  // Handle variable length strings on lines (e.g. get rid of previous characters if a new string is shorter)
  // This is a kludge because it only triggers for lines starting in column 0 and is hard-wired to handle
  // the special case of different strings at the ends of rows 5 and 7
  if (col == 0) {
    if ((row == 5) || (row == 7)) {
      len = 16;
    } else {
      len = DISP_LINE_CHARS;
    }
    while (len) {
      len--;
      DispPushBuffer[row*DISP_LINE_CHARS + len] = ' ';
    }
  }

  // Load the string
  len = strlen(s);
  while (len--) {
    if (col > (DISP_LINE_CHARS-1)) col = (DISP_LINE_CHARS-1);
  
    DispPushBuffer[row*DISP_LINE_CHARS + col++] = *s++;
  }
}


void DispEval() {
  char singleCharString[1];
  bool updatedChar = false;

  // Control brightness based on activity
  if ((LEDCounts[0] > 0) && !DispActiveBrightness) {
    LCD.setBacklight(DISP_ACTIVE_B);
    DispActiveBrightness = true;
  } else if ((LEDCounts[0] == 0) && DispActiveBrightness) {
    LCD.setBacklight(DISP_INACTIVE_B);
    DispActiveBrightness = false;
  }
  
  if (DispPullIndex >= BUFFER_SIZE) {
    // Copy the current active buffer into our (atomic) compare buffer
    for (int i=0; i<BUFFER_SIZE; i++) {
      DispPullBuffer[i] = DispPushBuffer[i];
    }
    DispPullIndex = 0;
  }

  // Check for changed characters - scan through array until a changed one is found or the 
  // entire array has been checked
  while (!updatedChar && (DispPullIndex < BUFFER_SIZE)) {
    if (DispPullBuffer[DispPullIndex] != DispCurBuffer[DispPullIndex]) {
      DispCurBuffer[DispPullIndex] = DispPullBuffer[DispPullIndex];
      LCD.setX((DispPullIndex % DISP_LINE_CHARS) * 6);
      LCD.setY((DispPullIndex / DISP_LINE_CHARS) * 8);
      singleCharString[0] = DispPullBuffer[DispPullIndex];
      LCD.printStr(singleCharString);
      updatedChar = true;
    }

    DispPullIndex++;
  }
}




// ================================================================================================
// Radio routines
//
void RadioInit() {
  SPI.setSCK(SCLK);
  SPI.begin();
  
  pinMode(DIO0_0, INPUT);
  pinMode(CSN0, OUTPUT);
  digitalWrite(CSN0, HIGH);
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);
  pinMode(DIO0_5, INPUT);
  pinMode(CSN1, OUTPUT);
  digitalWrite(CSN1, HIGH);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(DIO1_0, INPUT);
  pinMode(DIO1_5, INPUT);

  // Reset radio
  pinMode(RSTN, OUTPUT);
  digitalWrite(RSTN, LOW);
  delay(10);
  digitalWrite(RSTN, HIGH);

  // Initialize
  LEDCounts[0] = 0;
  LEDCounts[1] = 0;
  LoadConfigFile();
  setupRFM98(0);
  setupRFM98(1);
}


// ================================================================================================
// Serial logging routines
//
void LogInit() {
  Serial.begin(115200);
}


void LogString(char* s) {
  if (Serial) {
    Serial.print(s);
  }
}


void LogConstString(const char* s) {
  if (Serial) {
    Serial.print(s);
  }
}


