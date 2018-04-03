//
//  I'm publishing this source code under the MIT License (See: https://opensource.org/licenses/MIT)
//
//    Copyright 2018 Wayne Holder (https://sites.google.com/site/wayneholder/)
//
//    Portions Copyright Copyright (c) 2008-2011 Randall Bohn
//  
//    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//    documentation files (the "Software"), to deal in the Software without restriction, including without limitation
//    the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
//    to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  
//    The above copyright notice and this permission notice shall be included in all copies or substantial portions of
//    the Software.
//  
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
//    THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
//    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "OnePinSerial.h"

//  ATTiny85 Pinout
//
//                +-\/-+
//  RESET / PB5 1 |    | 8 Vcc
//   CLKI / PB3 2 |    | 7 PB2 / SCK
//   CLKO / PB4 3 |    | 6 PB1 / MISO
//          Gnd 4 |    | 5 PB0 / MOSI
//                +----+
//  
//  The following sources provided invaluable information:
//    http://www.ruemohr.org/docs/debugwire.html
//    https://github.com/dcwbrown/dwire-debug
//    Google "Understanding debugWIRE and the DWEN Fuse"
//    https://hackaday.io/project/20629-debugwire-debugger
//    https://github.com/jbtronics/WireDebugger/releases
//
//  This code supports experimentation with AVR In-System Programming Protocol and the debugWire protocol.
//  The In-System programming protocol uses the SPI interface and the RESET pin while debugWire requires
//  only the RESET pin to operate.  As currently configured, this code is designed to program and control
//  an ATTiny85, but can be reconfigured to control other AVR-Series 8 Bit Microcontrollers.  The code starts
//  up in In-System Programming mode, which has only a few single letter commands:
//
//    e            Erase Flash and EEPROM Memory
//    cxxxxxxxx   Send arbitrary 4 byte (xxxxxxxx) command sequence
//    i           Identify part type
//    f           Print current fuse settings for chip
//    +           Enable debugWire Fuse
//    -           Disable debugWire Fuse
//    p           Turn on power to chip (used to run code)
//    o           Switch off power to chip
//    b           Cycle Vcc and Send BREAK to engage debugWire Mode (debugWire Fuse must be enabled) 
//
//  Note: case does not matter, so typing f, or F is equivalent.
//
//  The In-System Programming Protocol uses a series of 4 byte commands which can be sent to the target using
//  the target's SPI interface while the RESET pin is active (LOW).  However, the first 4 byte command sent must
//  be the "Program Enable" command 0xAC 0x53 0xXX 0xYY, where the values of 0xXX and 0xYY do not matter.  This
//  command puts the chip into programming mode and echos back 0xZZ 0xAC 0x53 0xXX.  You can verify that the chip
//  is in programming mode by sending the command to read the chip's Vendor Code by sending 0x30 0x00 0x00 0x00.
//  This should echo back something like 0xYY 0x30 0xNN 0x1E, where 0x1E is the device code indicating an ATMEL
//  AVR-series microcontroller.  See the function enterProgramMode() for an example.  
//
//  Other 4 byte commands include:
//    Send                  Receive
//    0x30 0x00 0xNN 0x00   0xYY 0x30 0xNN 0xVV   Read mem address 0xNN and return value as 0xVV
//    0x30 0x00 0x00 0x00   0xYY 0x30 0xNN 0xVV   Read Vendor Code byte and return value as 0xVV 
//    0x30 0x00 0x01 0x00   0xYY 0x30 0xNN 0xVV   Read Part Family and Flash Size byte and return value as 0xVV
//    0x30 0x00 0x02 0x00   0xYY 0x30 0xNN 0xVV   Read Part Number byte and return value as 0xVV
//  
//  So, send a 4 byte sequence and get back a one byte response.  This code uses the function ispSend()
//  to make this simpler, such as;
//    ispSend()             Returns
//    0xF0 0x00 0x00 0x00   Polls the target's RDY/~BSY bit and returns 0xFF when busy, else 0xFE (doc error)
//    0x50 0x00 0x00 0x00   Read Low Fuses byte
//    0x58 0x08 0x00 0x00   Read High Fuses byte
//    0x50 0x08 0x00 0x00   Read Extended Fuses byte
//    0x58 0x00 0x00 0x00   Read Lock Bits
//    0xA0 0x00 <ad> 0x00   Reads byte from EEPROM at (<ad> & 0x3F) on selected page
//    0x28 <ms> <ls> 0x00   Reads High Byte from Program Memory <ms><ls>
//    0x20 <ms> <ls> 0x00   Reads Low Byte from Program Memory <ms><ls>
//  
//  You can use the cxxxxxxxx command to experiment with sending 4 byte commands while in the In-System Programming
//  mode.  For simplicity, the 'c' command only prints out the last byte of the 4 byte response.  So, for example,
//  to send the command to return the chip's device Id, type in the following and press Send:
//
//    c30000000
//
//  And the following will echo back:
//
//    Cmd: 30 00 00 00  = 1E
//
//  Similar commands support writing to Fuse bytes, etc:
//    ispSend()             Effect
//    0xAC 0xA0 0x00 <val>  Writes <val> into ATTiny85 Low Fuses byte
//    0xAC 0xA8 0x00 <val>  Writes <val> into ATTiny85 High Fuses byte
//    0xAC 0xA4 0x00 <val>  Writes <val> into ATTiny85 Extended Fuses byte
//    0xAC 0xE0 0x00 <val>  Writes ATTiny85 Lock Bits
//    0xC1 0x00 <pg> 0x00   Sets selected EEPROM Page to (<pg> & 0x03)
//    0xC0 0x00 <add> <val> Writes <val> to EEPROM address (<add> & 0x3F) on selected Page
//
//  IMPORTANT: Be very careful when changing the value of fuse bytes, as some combinations will put the chip in
//  a state where you can no longer program or communicate with it.  This can be especially tricky because the
//  enabled fuses are typically such that setting a bit to '1' disables the fuse and '0' enables it.
//
//  ATTiny series signature word and default fuses settings (see also printPartFromId())
//    ATTINY13   0x9007  // L: 0x6A, H: 0xFF              8 pin
//    ATTINY24   0x910B  // L: 0x62, H: 0xDF, E: 0xFF    14 pin
//    ATTINY44   0x9207  // L: 0x62, H: 0xDF, E: 0xFF    14 pin
//    ATTINY84   0x930C  // L: 0x62, H: 0xDF, E: 0xFF    14 pin
//    ATTINY25   0x9108  // L: 0x62, H: 0xDF, E: 0xFF     8 pin
//    ATTINY45   0x9206  // L: 0x62, H: 0xDF, E: 0xFF     8 pin
//    ATTINY85   0x930B  // L: 0x62, H: 0xDF, E: 0xFF     8 pin
//
//  Write to Flash Memory (Must first erase FLASH to all 0xFF values)
//
//  ispSend()             Effect
//  0xAC 0x80 0x00 0x00   Erase all Flash Program Memory and EEPROM Contents
//                        Note: wait nn ms then release RESET to end Erase Cycle
//  0x60 <ms> <ls> <val>  Writes <val> to Low Byte of Flash Address <ms><ls>  (must write LSB first)
//  0x68 <ms> <ls> <val>  Writes <val> to High Byte of Flash Address <ms><ls> (poll RDY
//
//  The debugWire Protocol
//
//  To use the debugWire protocol you must first enable debugWire Mode by using the In-System commands to 
//  clear the DWEN (DebugWIRE enabled) Fues.  On the ATTiny85, this means clearing bit 6 of the High Fuse
//  byte and cycling Vcc so the new fuese setting can take effect.  Note: once DWEN is enabled In-Syetem
//  Programming is no longer accessble via the conventional way (pulling RESET low) because debugWire is
//  now using the RESET pin.  Howwver, you can temporarily disable debugWire by sending a 0x06 byte on the
//  RESET line, then taking RESET LOW to enter in-System Programming Mode and clearing DWEN by setting bit
//  6 of the High Fuse byte.  This can be down with the following sequence of text commands:
//
//    1.  EXIT    Send disable debugWire command
//    2.  -       Disable DWEN bit (bit 6) in High Fuses byte on Tiny25/45/85
//    3.  F       Verify DWEN bit is set
//
//  IMPORTANT - You cannot program the chip with a programmer like the AVRISP mkII while the debugWire fuse
//  is enabled.
//  
//  DebugWire Command bytes
//    Basic commands
//      BREAK   Halts Target with a Break (Return 0xFF if stopped)
//      0x06    Disable debugWire (returns nothing)
//      0x07    Reset Target (returns 0x00 0x5 after 60 ms delay)
//      0x20    Starts Repeating Instructions (for example, 0xC2 0x01)
//      0x21    Steps Repeating Instruction once
//      0x23    Single Step Instruction Set by CMD_SET_INSTR
//      0x30    Start Normal Execution of Code (return nothing)
//      0x31    Single Step on target at PC (returns 0x00 0x55, where 0x00 comes right after 0x32; 0x55 after a delay)
//      0x32    GoStart Normal Execution using Instruction Set by CMD_SET_INSTR
//      0x33    Single Step Instruction (returns 0x00 0x55, where 0x00 comes right after 0x33; 0x55 after a delay)
//    Set Context
//      0x60    Set GO context  (No bp?)
//      0x61    Set run to cursor context (Run to hardware BP?)
//      0x63    Set step out context (Run to return instruction?)
//      0x64    Set up for single step using loaded instruction
//      0x66    Set up for read/write using repeating simulated instructions
//      0x79    Set step-in / autostep context or when resuming a sw bp (Execute a single instruction?)
//      0x7A    Set single step context
//    Read Control Register
//      0xF0      Returns PC as two bytes (MSB LSB)
//      0xF1      Returns HW Breakpoint Reg as two bytes (MSB LSB)
//      0xF2      Returns Current Instruction as two bytes (MSB LSB)
//      0xF3      Returns Device Signature as two bytes (MSB LSB)
//    Write Control Register
//      0xD0      Sets PC to following two bytes (MSB LSB)
//      0xD1      Sets HW Breakpoint Reg to following two bytes (MSB LSB)
//      0xD2      Set following two bytes as Instruction for CMD_SS_INSTR
//      0xD3      Not Used (can't write signature)
//    Results
//      0x00 0x55 Response to Asyn Serial BREAK and OK response for some commands
//
//    Two byte sequences for Hard-coded operations
//    Note: I'm still trying to figure out the details on how these sequences work...
//      0xC2 0x01   out DWDR,r0   out DWDR,r1,...   Start register read
//      0xC2 0x05   in r0,DWDR    in r0,DWDR,...    Start register write
//      0xC2 0x00   ld  r16,Z+    out DWDR,r16      Start data area read
//      0xC2 0x04   in r16,DWDR   st Z+,r16         Start data area write
//    Others...
//      0xC2 0x02   lpm r?,Z+     out DWDR,r?       ??
//      0xC2 0x03   lpm r?,Z+     out SWDR,r?       Flash Read??
//      0xC2 0x04   in r16,DWDR   st Z+,r16         Set data area write mode
//  
//    Send sequence of bytes using debugWire technique on RESET pin, for example:
//      CMD=66D00000D10020C20120    Read Registers 0 - 31
//      CMD=66D00000D10001C20521AA  Write 0xAA to Register 0
//
//    Dump 128 byte of Flash from Address 0x0000
//      CMD=66D0001ED10020C205200000D00000C202D1010020 (breaks out, as follows)
//        66
//        D0 00 1E    // PC = 001E
//        D1 00 20    // HWBP = 0020
//        C2 05       // Write Registers
//        20          // Go read/write
//        ll hh    
//        D0 00 00    // PC = 0000
//        C2 02       // Read Flash
//        D1 01 00    // HWBP = 0100
//        20          // Go read/write
//          
//    Example AVR Instructions for INST= and STEP commands
//        0000  NOP
//        9403  INC r0          1010 010d dddd 0011
//        9413  INC r1
//        9503  INC r16
//        940A  DEC r0          1010 010d dddd 1010
//        941A  DEC r1
//        E000  LDI, r16,0x00   1110 KKKK dddd KKKK (only regs 16-31)
//        E50A  LDI, r16,0x5A   1110 KKKK dddd KKKK
//        9598  BREAK
//
//    ATTiny85 Registers  I/O    SRAM
//        PORTB           0x18   0x38
//        DDRB            0017   0x37
//        PINB            0x16   0x36
//
//    Location of the DWDR Register (varies by processor)  ATTiny25/45/85 value verified, others from references
//        0x2E        ATTiny13
//        0x1F        ATTiny2313
//        0x22        ATTiny25/45/85
//        0x27        ATTiny24/44/84,441,841
//        0x31        ATmega48A,48PA,88A,88PA,8U2,168A,168PA,16U2,328P,328,32U2
//
//    Location of the DWEN fuse and fuse bit (varies by processor)
//        Fuse  Bit
//        High  0x04  ATTiny13/A
//        High  0x80  ATTiny2313/A
//        High  0x40  ATTiny25,45,85
//        High  0x40  ATTiny24/44/84,441,841
//        High  0x40  ATmega48A,48PA,88A,88PA,8U2,168A,168PA,16U2,328P,328,32U2
//
//    Location of fuse bits controlling clock speed
//        Fuse  Bits  (  R = divide by 8, 3:0 select clock, 0=ext, 1=
//        Low   ---R--10  ATTiny13
//        Low   R---3210  ATTiny2313
//        Low   R---3210  ATTiny25/45/85
//        Low   R---3210  ATTiny24/44/84
//        Low   R---3210  ATmega48A, etc. 
//
//    ATMega328P Fuse Settiings with Standard Arduino Bootloader - Low: FF, High: D8, Extd: FD
//    ATMega328P Fuse Settiings with Optiboot Bootloader -         Low: F7, High: DE, Extd: FD
//
//    Low byte:
//      7 = 1   Divide clock by 8
//      6 = 1   Clock output
//      5 = 1   Select start-up time bit 1
//      4 = 1   Select start-up time bit 0
//      3 = 1   Select Clock source bit 3 (F = Low Power Crystal Oscillator, 8-16 MHz range)
//      2 = 1   Select Clock source bit 2
//      1 = 1   Select Clock source bit 1
//      0 = 1   Select Clock source bit 0
//
//    High byte:
//      7 = 1   External Reset Disable
//      6 = 1   debugWIRE Enable
//      5 = 0   Enable Serial Program and Data Downloading
//      4 = 1   Watchdog Timer Always On
//      3 = 1   EEPROM memory is preserved through the Chip Erase
//      2 = 0   Select Boot Size bit 1 (see below)
//      1 = 0   Select Boot Size bit 0
//      0 = 0   Select Reset Vector (if enabled, sets IVSEL in MCUCR on RESET which moves int vectors to boot area)
//
//    MCUCR 0x35 (0x55) - IVSEL is bit 1
//
//    Bootloader Size:
//      0 = 0x3800 - 0x3FFF (2048 words)
//      1 = 0x3C00 - 0x3FFF (1024 words)
//      2 = 0x3E00 - 0x3FFF (512 words)
//      3 = 0x3F00 - 0x3FFF (256 words)
//
//    Extd byte:
//      7 = 1   Not used
//      6 = 1   Not used
//      5 = 1   Not used
//      4 = 1   Not used
//      3 = 1   Not used
//      2 = 1   Brown-out Detector trigger level bit 2
//      1 = 0   Brown-out Detector trigger level bit 1
//      0 = 1   Brown-out Detector trigger level bit 0

#define DEVELOPER 0

#define PMODE    8    // Input - HIGH = Program Mode, LOW = Debug Mode
#define VCC      9    // Target Pin 8 - Vcc
#define RESET   10    // Target Pin 1 - RESET, or SCI
#define MOSI    11    // Target Pin 5 - MOSI,  or SDI
#define MISO    12    // Target Pin 6 - MISO,  or SII
#define SCK     13    // Target Pin 7 - SCK,   or SDO

// Alternate Pin Definitions for High Voltage Programmer (for future expansion)
#define SCI     10    // Target Pin 2 - SCI
#define SII     11    // Target Pin 6 - SII
#define SDI     12    // Target Pin 5 - SDI
#define SDO     13    // Target Pin 7 - SDO

boolean       progMode = false;
boolean       debugWireOn = false;
byte          buf[256];               // Command and data buffer (also used by programmer)
byte          flashBuf[64];           // Flash read buffer for Diasm and FXxxxx and FBxxxx commands
unsigned int  flashStart;             // Base address of data in flashBuf[]
boolean       flashLoaded = false;    // True if flashBuf[] has valid data
boolean       hasDeviceInfo = false;  // True if Device Info has been read (dwdr, dwen, ckdiv8)
char          rpt[16];                // Repeat command buffer
byte          dwdr;                   // I/O addr of DWDR Register
byte          dwen;                   // DWEN bit mask in High Fuse byte
byte          ckdiv8;                 // CKDIV8 bit mask in Low Fuse BYte
unsigned int  ramBase;                // Base address of SRAM
unsigned int  ramSize;                // SRAM size in bytes
unsigned int  eeSize;                 // EEPROM size in bytes
byte          eecr;                   // EEPROM Control Register
byte          eedr;                   // EEPROM Data Register
byte          eearl;                  // EEPROM Address Register (low byte)
byte          eearh;                  // EEPROM Address Register (high byte)
unsigned int  flashSize;              // Flash Size in Bytes
unsigned int  pcSave;                 // Used by single STEP and RUN
unsigned int  bpSave;                 // Used by RUN
char          cursor = 0;             // Output line cursor used by disassembler to tab
boolean       breakWatch = false;     // Used by RUN commands
unsigned int  timeOutDelay;           // Timeout delay (based on baud rate)
boolean       runMode;                // If HIGH Debug Mode, else Program
boolean       reportTimeout = true;   // If true, report read timeout errors


OnePinSerial  debugWire(RESET);

#define DEBUG_BAUD    115200
#define PROGRAM_BAUD  19200

#define DWIRE_RATE    (1000000 / 128) // Set default baud rate (1 MHz / 128) Note: the 'b' command can change this

void setup () {
  pinMode(PMODE, INPUT);               // Mode Input
  digitalWrite(PMODE, HIGH);
  delay(1);
  if (runMode = digitalRead(PMODE)) {
    selectProgrammer();
  } else {
    selectDebugger();
  }
}

void selectDebugger() {
  powerOff();
  debugWire.begin(DWIRE_RATE);    
  setTimeoutDelay(DWIRE_RATE); 
  debugWire.enable(true);
  Serial.begin(DEBUG_BAUD);
  progMode = false;
  debugWireOn = false;
  flashLoaded = false;
  hasDeviceInfo = false;
  breakWatch = false;
  printMenu();
}

void powerOn () {
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  enableSpiPins();
  // Apply Vcc
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);
  // Bounce RESET
  digitalWrite(RESET, HIGH);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  digitalWrite(RESET, HIGH);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
}

void powerOff () {
  disableSpiPins();
  digitalWrite(VCC, LOW);
  pinMode(VCC, INPUT);
  delay(50);
  progMode = false;
}

void enableSpiPins () {
  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
}

void disableSpiPins () {
  pinMode(SCK, INPUT);
  digitalWrite(SCK, LOW);
  pinMode(MOSI, INPUT);
  digitalWrite(MOSI, LOW);
  pinMode(MISO, INPUT);
  digitalWrite(MISO, LOW);
}

byte transfer (byte val) {
  for (byte ii = 0; ii < 8; ++ii) {
    digitalWrite(MOSI, (val & 0x80) ? HIGH : LOW);
    digitalWrite(SCK, HIGH);
    delayMicroseconds(4);
    val = (val << 1) + digitalRead(MISO);
    digitalWrite(SCK, LOW); // slow pulse
    delayMicroseconds(4);
  }
  return val;
}

byte ispSend (byte c1, byte c2, byte c3, byte c4) {
  transfer(c1);
  transfer(c2);
  transfer(c3);
  return transfer(c4);
}

boolean enterProgramMode () {
  if (progMode) {
    return true;
  }
  byte timeout = 0;
  byte rsp;
  do {
    if (timeout > 0) {
      powerOff();
    }
    powerOn();
    ispSend(0xAC, 0x53, 0x00, 0x00);
    rsp = ispSend(0x30, 0x00, 0x00, 0x00);
  } while (rsp != 0x1E && ++timeout < 5);
  progMode = timeout < 5;
  if (!progMode) {
    println(F("Timeout: Chip may have DWEN bit enabled"));
  }
  return progMode;
}

// Read two byte chip signature into buf[]
void getChipId () {
  if (enterProgramMode()) {
    for (byte ii = 0; ii < 2; ii++) {
      buf[ii] = ispSend(0x30, 0x00, ii + 1, 0x00);
    }
  }
}

unsigned int identifyDevice () {
 if (enterProgramMode()) {
    print(F("SIG:   "));
    getChipId();
    printBufToHex8(2, false);
    printPartFromId(buf[0], buf[1]);
    return (buf[0] << 8) + buf[1];
  }
  return 0;
}

void busyWait () {
  while ((ispSend(0xF0, 0x00, 0x00, 0x00) & 1) == 1) {
    delay(10);
  }
}

/*
    Sig      Device   CKDIV8  DWEN
    0x920B - Tiny24     L:7   H:6
    0x9108 - Tiny25     L:7   H:6
    0x9205 - Mega48A    L:7   H:6
    0x9207 - Tiny44     L:7   H:6
    0x9206 - Tiny45     L:7   H:6
    0x920A - Mega48PA   L:7   H:6
    0x9215 - Tiny441    L:7   H:6
    0x930A - Mega88A    L:7   H:6
    0x930C - Tiny84     L:7   H:6
    0x930B - Tiny85     L:7   H:6
    0x930F - Mega88PA   L:7   H:6
    0x9315 - Tiny841    L:7   H:6
    0x9406 - Mega168A   L:7   H:6
    0x940B - Mega168PA  L:7   H:6
    0x950F - Mega328P   L:7   H:6   Arduino defaults: L: 0xFF. H: 0xDA, E: 0xFD
    0x9514 - Mega328    L:7   H:6
    
    0x900A - Tiny13     L:4   H:3
    0x910A - Tiny2313   L:7   H:7
    0x9389 - Mega8u2    L:7   H:7
    0x9489 - Mega16U2   L:7   H:7
    0x958A - Mega32U2   L:7   H:7
*/

void printPartFromId (byte sig1, byte sig2) {
  boolean isTiny = false;
  print(F("= "));
  dwen = 0x40;                      // DWEN bit default location (override, as needed)
  ckdiv8 = 0x80;                    // CKDIV8 bit default location (override, as needed)
  hasDeviceInfo = true;
  if (sig1 == 0x90) {            
    if (sig2 == 0x07) {             // 0x9007 - Tiny13
      println(F("Tiny13"));
      flashSize = 1024;
      ramBase = 0x60;
      ramSize = 64;
      eeSize = 64;
      dwdr = 0x2E;
      dwen = 0x08;
      ckdiv8 = 0x10;
      isTiny = true;
    }
  } else if (sig1 == 0x91) {
    if (sig2 == 0x0A) {             // 0x920A - Tiny2313
      println(F("Tiny2313"));  
      flashSize = 2048;
      ramBase = 0x60;
      ramSize = 128;
      eeSize = 128;
      dwdr = 0x1F;
      dwen = 0x80;
      isTiny = true;
    } else if (sig2 == 0x0B) {      // 0x920B - Tiny24
      println(F("Tiny24"));  
      flashSize = 2048;
      ramBase = 0x60;
      ramSize = 128;
      eeSize = 128;
      dwdr = 0x27;
      isTiny = true;
   } else if (sig2 == 0x08) {      // 0x9108 - Tiny25
      println(F("Tiny25"));  
      flashSize = 2048;
      ramBase = 0x60;
      ramSize = 128;
      eeSize = 128;
      dwdr = 0x22;
      isTiny = true;
    }
  } else if (sig1 == 0x92) {
    if (sig2 == 0x05) {             // 0x9295 - Mega48A
      println(F("Mega48A"));
      flashSize = 4096;
      ramBase = 0x100;
      ramSize = 512;
      eeSize = 256;
      dwdr = 0x31;
    } else if (sig2 == 0x07) {      // 0x9207 - Tiny44
      println(F("Tiny44"));  
      flashSize = 4096;
      ramBase = 0x60;
      ramSize = 256;
      eeSize = 256;
      dwdr = 0x27;
      isTiny = true;
    } else if (sig2 == 0x06) {      // 0x9206 - Tiny45
      println(F("Tiny45"));  
      flashSize = 4096;
      ramBase = 0x60;
      ramSize = 256;
      eeSize = 256;
      dwdr = 0x22;
      isTiny = true;
    } else if (sig2 == 0x0A) {      // 0x920A - Mega48PA
      println(F("Mega48PA"));  
      flashSize = 4096;
      ramBase = 0x100;
      ramSize = 512;
      eeSize = 256;
      dwdr = 0x31;
    } else if (sig2 == 0x15) {      // 0x9215 - Tiny441
      println(F("Tiny441"));  
      flashSize = 4096;
      ramBase = 0x100;
      ramSize = 256;
      eeSize = 256;
      dwdr = 0x27;
      isTiny = true;
    }
  } else if (sig1 == 0x93) {
    if (sig2 == 0x0A) {             // 0x930A - Mega88A
      println(F("Mega88A"));  
      flashSize = 8192;
      ramBase = 0x100;
      ramSize = 1024;
      eeSize = 512;
      dwdr = 0x31;
    } else if (sig2 == 0x0C) {      // 0x930C - Tiny84
      println(F("Tiny84"));  
      flashSize = 8192;
      ramBase = 0x60;
      ramSize = 512;
      eeSize = 512;
      dwdr = 0x27;
      isTiny = true;
    } else if (sig2 == 0x0B) {      // 0x930B - Tiny85
      println(F("Tiny85"));  
      flashSize = 8192;
      ramBase = 0x60;
      ramSize = 512;
      eeSize = 512;
      dwdr = 0x22;
      isTiny = true;
    } else if (sig2 == 0x0F) {      // 0x930F - Mega88PA
      println(F("Mega88PA"));  
      flashSize = 8192;
      ramBase = 0x100;
      ramSize = 1024;
      eeSize = 512;
      dwdr = 0x31;
    } else if (sig2 == 0x15) {      // 0x9315 - Tiny841
      println(F("Tiny841"));  
      flashSize = 8192;
      ramBase = 0x100;
      ramSize = 512;
      eeSize = 512;
      dwdr = 0x27;
      isTiny = true;
    } else if (sig2 == 0x89) {      // 0x9389 - Mega8u2
      println(F("Mega8u2"));  
      flashSize = 8192;
      ramBase = 0x100;
      ramSize = 512;
      eeSize = 512;
      dwdr = 0x31;
    }
  } else if (sig1 == 0x94) {
    if (sig2 == 0x06) {             // 0x9406 - Mega168A
      println(F("Mega168A"));  
      flashSize = 16384;
      ramBase = 0x100;
      ramSize = 1024;
      eeSize = 512;
      dwdr = 0x31;
    } else if (sig2 == 0x0B) {      // 0x9408 - Mega168PA
      println(F("Mega168PA"));  
      flashSize = 16384;
      ramBase = 0x100;
      ramSize = 1024;
      eeSize = 512;
      dwdr = 0x31;
    } else if (sig2 == 0x89) {      // 0x9489 - Mega16U2
      println(F("Mega16U2"));  
      flashSize = 16384;
      ramBase = 0x100;
      ramSize = 512;
      eeSize = 512;
      dwen = 0x80;
      dwdr = 0x31;
    }
  } else if (sig1 == 0x95) {
    if (sig2 == 0x0F) {             // 0x950F - Mega328P
      println(F("Mega328P"));  
      flashSize = 32768;
      ramBase = 0x100;
      ramSize = 2048;
      eeSize = 1024;
      dwdr = 0x31;
    } else if (sig2 == 0x14) {      // 0x9514 - Mega328
      println(F("Mega328"));  
      flashSize = 32768;
      ramBase = 0x100;
      ramSize = 2048;
      eeSize = 1024;
      dwdr = 0x31;
    } else if (sig2 == 0x8A) {      // 0x958A - Mega32U2
      println(F("Mega32U2"));  
      flashSize = 32768;
      ramBase = 0x100;
      ramSize = 1024;
      eeSize = 1024;
      dwen = 0x80;
      dwdr = 0x31;
    }
  } else {
    hasDeviceInfo = false;
  }
  // Set EEPROM Access Registers
  if (isTiny) {
    eecr  = 0x1C;
    eedr  = 0x1D;
    eearl = 0x1E;
    eearh = 0x1F;
  } else {
    eecr  = 0x1F;
    eedr  = 0x20;
    eearl = 0x21;
    eearh = 0x22;
  }
}

void sendBreak () {
  debugWire.sendBreak();
}

boolean doBreak () { 
  println(F("Cycling Vcc"));
  powerOff();
  digitalWrite(VCC, HIGH);
  digitalWrite(RESET, LOW);
  pinMode(RESET, INPUT);
  pinMode(VCC, OUTPUT);
  delay(100);
  // Mesaure debugWire Baud rate by sending two BREAK commands to measure both high-going and
  // low-going pulses in the 0x55 response byte
  debugWire.enable(false);
  unsigned long pulse = 0;
  uint8_t oldSREG = SREG;
  cli();                      // turn off interrupts for timing
  sendBreak();
  for (byte ii = 0; ii < 4; ii++) {
    pulse += pulseIn(RESET, HIGH, 20000);
  }
  delay(10);
  sendBreak();
  for (byte ii = 0; ii < 4; ii++) {
    pulse += pulseIn(RESET, LOW, 20000);
  }
  SREG = oldSREG;             // turn interrupts back on
  delay(10);
  unsigned long baud = 8000000L / pulse;
  cursor = 0;
  printCmd(F("Speed"));
  Serial.print(baud, DEC);
  Serial.println(F(" bps"));
  debugWire.enable(true);
  debugWire.begin(baud);                            // Set computed baud rate
  setTimeoutDelay(DWIRE_RATE);                      // Set timeout based on baud rate
  print(F("Sending BREAK: "));
  sendBreak();
  if (checkCmdOk()) {
    debugWireOn = true;
    println(F("debugWire Enabled"));
  } else {
    println();
  }
  // Discard any suprious characters, such as '\n'
  while (Serial.available()) {
    Serial.read();
  }
  return debugWireOn;
}

byte toHex (char cc) {
  if (cc >= '0' && cc <= '9') {
    return cc - '0';
  } else if (cc >= 'A' && cc <= 'F') {
    return cc - 'A' + 10;
  }
  return 0;
}

unsigned int convertHex (byte idx) {
  unsigned int val = 0;
  while (isHexDigit(buf[idx])) {
    val = (val << 4) + toHex(buf[idx++]);
  }
  return val;
}

byte after (char cc) {
  byte idx = 0;
  while (buf[idx++] != cc && idx < 32)
    ;
  return idx;
}

// Read hex characters in buf[] starting at <off> and convert to bytes in buf[] starting at index 0
byte convertToHex (byte off) {
  byte idx = 0;
  char cc;
  while ((cc = buf[off++]) != 0) {
    byte hdx = idx >> 1;
    buf[hdx] = (buf[hdx] << 4) + toHex(cc);
    idx++;
  }
  return (idx + 1) / 2;
}

byte readDecimal (byte off) {
  byte val = 0;
  char cc;
  while ((cc = buf[off++]) != 0 && cc >= '0' && cc <= '9') {
    val = (val * 10) + (byte) (cc - '0');
  }
  return val;
}

// Get '\n'-terminated string and return length
int getString () {
  int idx = 0;
  while (true) {
    if (Serial.available()) {
      char cc = toupper(Serial.read());
      if (cc == '\r') {
        // Ignore
      } else if (cc == '\n') {
        buf[idx] = 0;
        return idx;
      } else {
        buf[idx++] = cc;
      }
    }
  }
}

void printBufToHex8 (int count, boolean trailCrLF) {
  if (count > 16) {
    println();
  }
  for (byte ii = 0; ii < count; ii++) {
    printHex8(buf[ii]);
    if ((ii & 0x0F) == 0x0F) {
      println();
    } else {
      print(" ");
    }
  }
  if (trailCrLF && (count & 0x0F) != 0) {
    println();
  }
}

char toHexDigit (byte nib) {
  if (nib >= 0 & nib < 10) {
    return '0' + nib;
  } else {
    return 'A' + (nib - 10);
  }
}

boolean isDecDigit (char cc) {
  return cc >= '0' && cc <= '9';
}

boolean isHexDigit (char cc) {
  return isDecDigit(cc) || (cc >= 'A' && cc <= 'F');
}

boolean bufMatches (const __FlashStringHelper* ref) {
  byte jj = 0;
  for (byte ii = 0; ii < sizeof(buf); ii++) {
    byte cc = pgm_read_byte((char *) ref + jj++);
    char bb = buf[ii];
    if (cc == 0) {
      return bb == 0;
    }
    if ((cc == 'x' || cc == 'X') && isHexDigit(bb)) {         // 0-9 or A-F
      if (!isHexDigit(buf[ii + 1])) {
        while (pgm_read_byte((char *) ref + jj) == 'X') {
          jj++;
        }
      }
    } else if ((cc == 'd' || cc == 'D') && isDecDigit(bb)) {  // 0-9
      if (!isDecDigit(buf[ii + 1])) {
        while (pgm_read_byte((char *) ref + jj) == 'D') {
          jj++;
        }
      }
    } else if (cc == 'o' && bb >= '0' && bb <= '7') {         // 0-7
      continue;
    } else if (cc == 'b' && (bb == '0' || bb == '1')) {       // 0 or 1
      continue;
    } else if (cc != bb) {
      return false;
    }
  }
  return false;
}

boolean bufStartsWith (const __FlashStringHelper* ref) {
  for (byte ii = 0; ii < sizeof(buf); ii++) {
    byte cc = pgm_read_byte((char *) ref + ii);
    if (cc == 0) {
      return true;
    } else if (cc != buf[ii]) {
      return false;
    }
  }
  return false;
}

void sendCmd (byte* data, byte count) {
  debugWire.sendCmd(data, count);
}

byte getResponse (int expected) {
  return getResponse(&buf[0], expected);
}

byte getResponse (byte *data, int expected) {
  byte idx = 0;
  byte timeout = 0;
  do {
    if (debugWire.available()) {
      data[idx++] = debugWire.read();
      timeout = 0;
      if (expected > 0 && idx == expected) {
        return expected;
      }
    } else {
      delayMicroseconds(timeOutDelay);
      timeout++;
    }
  } while (timeout < 50 && idx < sizeof(buf));
  if (reportTimeout) {
    print(F("Timeout: received: "));
    printDec(idx);
    print(F(" expected: "));
    printDec(expected);
    println();
  }
  return idx;
}

void setTimeoutDelay (unsigned int rate) {
  timeOutDelay = F_CPU / rate;
}

unsigned int getWordResponse () {
  byte tmp[2];
  getResponse(&tmp[0], 2);
  return ((unsigned int) tmp[0] << 8) + tmp[1];
}

boolean checkCmdOk () {
  byte tmp[2];
  byte rsp = getResponse(&tmp[0], 1);
  if (rsp == 1 && tmp[0] == 0x55) {
    println("Ok");
    return true;
  } else {
    return false;
  }
}

boolean checkCmdOk2 () {
  byte tmp[2];
  if (getResponse(&tmp[0], 2) == 2 && tmp[0] == 0x00 && tmp[1] == 0x55) {
    println("Ok");
    return true;
  } else {
    println("Err");
    return false;
  }
}

//  The functions used to read read and write registers, SRAM and flash memory use "in reg,addr" and "out addr,reg" instructions 
//  to trnasfers data over debugWire via the DWDR register.  However, because the location of the DWDR register can vary from device
//  to device, the necessary "in" and "out" instructions need to be build dynamically using the following 4 functions:
// 
//         -- --                            In:  1011 0aar rrrr aaaa
//      D2 B4 02 23 xx   - in r0,DWDR (xx)       1011 0100 0000 0010  a=100010(22), r=00000(00) // How it's used
//         -- --                            Out: 1011 1aar rrrr aaaa
//      D2 BC 02 23 <xx> - out DWDR,r0           1011 1100 0000 0010  a=100010(22), r=00000(00) // How it's used
//
//  Note: 0xD2 sets next two bytes as instruction which 0x23 then executes.  So, in first example, the sequence D2 B4 02 23 xx
//  copies the value xx into the r0 register via the DWDR register.  The second example does the reverse and returns the value
//  in r0 as <xx> by sending it to the DWDR register.

// Build high byte of opcode for "out addr, reg" instruction
byte outHigh (byte add, byte reg) {
  // out addr,reg: 1011 1aar rrrr aaaa
  return 0xB8 + ((reg & 0x10) >> 4) + ((add & 0x30) >> 3);
}

// Build low byte of opcode for "out addr, reg" instruction
byte outLow (byte add, byte reg) {
  // out addr,reg: 1011 1aar rrrr aaaa
  return (reg << 4) + (add & 0x0F);
}

// Build high byte of opcode for "in reg,addr" instruction
byte inHigh  (byte add, byte reg) {
  // in reg,addr:  1011 0aar rrrr aaaa
  return 0xB0 + ((reg & 0x10) >> 4) + ((add & 0x30) >> 3);
}

// Build low byte of opcode for "in reg,addr" instruction
byte inLow  (byte add, byte reg) {
  // in reg,addr:  1011 0aar rrrr aaaa
  return (reg << 4) + (add & 0x0F);
}

// Set register <reg> by building and executing an "out <reg>,DWDR" instruction via the CMD_SET_INSTR register
void writeRegister (byte reg, byte val) {
  byte wrReg[] = {0x64,                                               // Set up for single step using loaded instruction
                  0xD2, inHigh(dwdr, reg), inLow(dwdr, reg), 0x23,    // Build "in reg,DWDR" instruction
                  val};                                               // Write value to register via DWDR
  sendCmd(wrReg,  sizeof(wrReg));
}

// Read register <reg> by building and executing an "out DWDR,<reg>" instruction via the CMD_SET_INSTR register
byte readRegister (byte reg) {
  byte rdReg[] = {0x64,                                               // Set up for single step using loaded instruction
                  0xD2, outHigh(dwdr, reg), outLow(dwdr, reg),        // Build "out DWDR, reg" instruction
                  0x23};                                              // Execute loaded instruction
  sendCmd(rdReg,  sizeof(rdReg));
  getResponse(1);                                                     // Get value sent as response
  return buf[0];;
}

// Read registers 0-31 into buf[]
boolean  readAllRegisters () {
  byte rdRegs[] = {0x66,                                              // Set up for read/write using repeating simulated instructions
                  0xD0, 0x00, 0x00,                                   // Set Start Reg number (r0)
                  0xD1, 0x00, 0x20,                                   // Set End Reg number (r31) + 1
                  0xC2, 0x01,                                         // Set repeating copy to via DWDR to registers
                  0x20};                                              // Go
  sendCmd(rdRegs,  sizeof(rdRegs));
  return getResponse(32) == 32;
}

// Set or clear bit in I/O address 0x00 - 0x1F by generating and executing an "sbi" or "cbi" instruction
void setClrIOBit (byte addr, byte bit, boolean set) {
  // Generate an "sbi/cbi addr,bit" instruction
  byte cmd[] = {0x64, 0xD2, set ? 0x9A : 0x98, ((addr & 0x1F) << 3) + (bit & 0x7), 0x23};  // 1001 10x0 aaaa abbb
  sendCmd(cmd,  sizeof(cmd));
}

// Write one byte to SRAM address space using an SRAM-based value for <addr>, not an I/O address
void writeSRamByte (unsigned int addr, byte val) {
  byte r30 = readRegister(30);        // Save r30 (Z Reg low)
  byte r31 = readRegister(31);        // Save r31 (Z Reg high)
  byte wrSRam[] = {0x66,                                              // Set up for read/write using repeating simulated instructions
                   0xD0, 0x00, 0x1E,                                  // Set Start Reg number (r30)
                   0xD1, 0x00, 0x20,                                  // Set End Reg number (r31) + 1
                   0xC2, 0x05,                                        // Set repeating copy to registers via DWDR
                   0x20,                                              // Go
                   addr & 0xFF, addr >> 8,                            // r31:r30 (Z) = addr
                   0xD0, 0x00, 0x01,
                   0xD1, 0x00, 0x03,
                   0xC2, 0x04,                                        // Set simulated "in r?,DWDR; st Z+,r?" insrtuctions
                   0x20,                                              // Go
                   val};
  sendCmd(wrSRam, sizeof(wrSRam));
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
}

// Read one byte from SRAM address space using an SRAM-based value for <addr>, not an I/O address
byte readSRamByte (unsigned int addr) {
  byte r30 = readRegister(30);        // Save r30 (Z Reg low)
  byte r31 = readRegister(31);        // Save r31 (Z Reg high)
  byte rdSRam[] = {0x66,                                              // Set up for read/write using repeating simulated instructions
                   0xD0, 0x00, 0x1E,                                  // Set Start Reg number (r30)
                   0xD1, 0x00, 0x20,                                  // Set End Reg number (r31) + 1
                   0xC2, 0x05,                                        // Set repeating copy to registers via DWDR
                   0x20,                                              // Go
                   addr & 0xFF, addr >> 8,                            // r31:r30 (Z) = addr
                   0xD0, 0x00, 0x00,                                  // 
                   0xD1, 0x00, 0x02,                                  // 
                   0xC2, 0x00,                                        // Set simulated "ld r?,Z+; out DWDR,r?" insrtuctions
                   0x20};                                             // Go
  sendCmd(rdSRam, sizeof(rdSRam));
  getResponse(1);
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
  return buf[0];
}

// Read <len> bytes from SRAM address space into buf[] using an SRAM-based value for <addr>, not an I/O address
// Note: can't read addresses that correspond to  r28-31 (Y & Z Regs) because Z is used for transfer (not sure why Y is clobbered) 
boolean readSRamBytes (unsigned int addr, byte len) {
  unsigned int len2 = len * 2;
  byte r28 = readRegister(28);        // Save r28 (Y Reg low)
  byte r29 = readRegister(29);        // Save r29 (Y Reg high)
  byte r30 = readRegister(30);        // Save r30 (Z Reg low)
  byte r31 = readRegister(31);        // Save r31 (Z Reg high)
  byte rsp;
  reportTimeout = false;
  for (byte ii = 0; ii < 4; ii++) {
    byte rdSRam[] = {0x66,                                            // Set up for read/write using repeating simulated instructions
                     0xD0, 0x00, 0x1E,                                // Set Start Reg number (r30)
                     0xD1, 0x00, 0x20,                                // Set End Reg number (r31) + 1
                     0xC2, 0x05,                                      // Set repeating copy to registers via DWDR
                     0x20,                                            // Go
                     addr & 0xFF, addr >> 8,                          // r31:r30 (Z) = addr
                     0xD0, 0x00, 0x00,                                // 
                     0xD1, len2 >> 8, len2,                           // Set repeat count = len * 2
                     0xC2, 0x00,                                      // Set simulated "ld r?,Z+; out DWDR,r?" instructions
                     0x20};                                           // Go
    sendCmd(rdSRam, sizeof(rdSRam));
    rsp = getResponse(len);
    if (rsp == len) {
      break;
    } else {
      // Wait and retry read
      delay(5);
    }
  }
  reportTimeout = true;
  writeRegister(28, r28);             // Restore r28
  writeRegister(29, r29);             // Restore r29
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
  return rsp == len;
}

//   EEPROM Notes: This section contains code to read and write from EEPROM.  This is accomplished by setting parameters
//    into registers 28 - r31 and then using the 0xD2 command to send and execure a series of instruction opcodes on the
//    target device. 
// 
//   EEPROM Register Locations for ATTiny25/45/85, ATTiny24/44/84, ATTiny13, Tiny2313, Tiny441/841
//     EECR    0x1C EEPROM Control Register
//     EEDR    0x1D EEPROM Data Register
//     EEARL   0x1E EEPROM Address Register (low byte)
//     EEARH   0x1F EEPROM Address Register (high byte)
// 
//   EEPROM Register Locations for ATMega328, ATMega32U2/16U2/32U2, etc.
//     EECR    0x1F EEPROM Control Register
//     EEDR    0x20 EEPROM Data Register
//     EEARL   0x21 EEPROM Address Register (low byte)
//     EEARH   0x22 EEPROM Address Register (high byte)

// 
//   Read one byte from EEPROM
//   

byte readEepromByte (unsigned int addr) {
  byte setRegs[] = {0x66,                                               // Set up for read/write using repeating simulated instructions
                    0xD0, 0x00, 0x1C,                                   // Set Start Reg number (r28(
                    0xD1, 0x00, 0x20,                                   // Set End Reg number (r31) + 1
                    0xC2, 0x05,                                         // Set repeating copy to registers via DWDR
                    0x20,                                               // Go
                    0x01, 0x01, addr & 0xFF, addr >> 8};                // Data written into registers r28-r31
  byte doRead[]  = {0x64,                                               // Set up for single step using loaded instruction
                    0xD2, outHigh(eearh, 31), outLow(eearh, 31), 0x23,  // out EEARH,r31  EEARH = ah  EEPROM Address MSB
                    0xD2, outHigh(eearl, 30), outLow(eearl, 30), 0x23,  // out EEARL,r30  EEARL = al  EEPROMad Address LSB
                    0xD2, outHigh(eecr, 28), outLow(eecr, 28), 0x23,    // out EECR,r28   EERE = 01 (EEPROM Read Enable)
                    0xD2, inHigh(eedr, 29), inLow(eedr, 29), 0x23,      // in  r29,EEDR   Read data from EEDR
                    0xD2, outHigh(dwdr, 29), outLow(dwdr, 29), 0x23};   // out DWDR,r29   Send data back via DWDR reg
  byte r28 = readRegister(28);        // Save r28
  byte r29 = readRegister(29);        // Save r29
  byte r30 = readRegister(30);        // Save r30
  byte r31 = readRegister(31);        // Save r31
  sendCmd(setRegs, sizeof(setRegs));
  sendCmd(doRead, sizeof(doRead));
  getResponse(1);                                                       // Read data from EEPROM location
  writeRegister(28, r28);             // Restore r28
  writeRegister(29, r29);             // Restore r29
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
  return buf[0];
}

//   
//   Write one byte to EEPROM
//   

void writeEepromByte (unsigned int addr, byte val) {
  byte r28 = readRegister(28);        // Save r28
  byte r29 = readRegister(29);        // Save r29 
  byte r30 = readRegister(30);        // Save r30
  byte r31 = readRegister(31);        // Save r31
  byte setRegs[] = {0x66,                                                 // Set up for read/write using repeating simulated instructions
                    0xD0, 0x00, 0x1C,                                     // Set Start Reg number (r30)
                    0xD1, 0x00, 0x20,                                     // Set End Reg number (r31) + 1
                    0xC2, 0x05,                                           // Set repeating copy to registers via DWDR
                    0x20,                                                 // Go
                    0x04, 0x02, addr & 0xFF, addr >> 8};                  // Data written into registers r28-r31
  byte doWrite[] = {0x64,                                                 // Set up for single step using loaded instruction
                    0xD2, outHigh(eearh, 31), outLow(eearh, 31), 0x23,    // out EEARH,r31  EEARH = ah  EEPROM Address MSB
                    0xD2, outHigh(eearl, 30), outLow(eearl, 30), 0x23,    // out EEARL,r30  EEARL = al  EEPROM Address LSB
                    0xD2, inHigh(dwdr, 30), inLow(dwdr, 30), 0x23,        // in  r30,DWDR   Get data to write via DWDR
                    val,                                                  // Data written to EEPROM location
                    0xD2, outHigh(eedr, 30), outLow(eedr, 30), 0x23,      // out EEDR,r30   EEDR = data
                    0xD2, outHigh(eecr, 28), outLow(eecr, 28), 0x23,      // out EECR,r28   EECR = 04 (EEPROM Master Program Enable)
                    0xD2, outHigh(eecr, 29), outLow(eecr, 29), 0x23};     // out EECR,r29   EECR = 02 (EEPROM Program Enable)
  sendCmd(setRegs, sizeof(setRegs));
  sendCmd(doWrite, sizeof(doWrite));
  writeRegister(28, r28);             // Restore r28
  writeRegister(29, r29);             // Restore r29
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
}

//
//  Read 128 bytes from flash memory area at <addr> into data[] buffer
//
int readFlashPage (char *data, unsigned int len, unsigned int addr) {
  byte r28 = readRegister(28);        // Save r28 (Y Reg low)
  byte r29 = readRegister(29);        // Save r29 (Y Reg high)
  byte r30 = readRegister(30);        // Save r30 (Z Reg low)
  byte r31 = readRegister(31);        // Save r31 (Z Reg high)
  // Read sizeof(flashBuf) bytes form flash page at <addr>
  byte rsp;
  reportTimeout = false;
  unsigned int lenx2 = len * 2;
  for (byte ii = 0; ii < 4; ii++) {
    byte rdFlash[] = {0x66,                                               // Set up for read/write using repeating simulated instructions
                      0xD0, 0x00, 0x1E,                                   // Set Start Reg number (r30)
                      0xD1, 0x00, 0x20,                                   // Set End Reg number (r31) + 1
                      0xC2, 0x05,                                         // Set repeating copy to registers via DWDR
                      0x20,                                               // Go
                      addr & 0xFF, addr >> 8,                             // r31:r30 (Z) = addr
                      0xD0, 0x00, 0x00,                                   // Set start = 0
                      0xD1, lenx2 >> 8,lenx2,                             // Set end = repeat count = sizeof(flashBuf) * 2
                      0xC2, 0x02,                                         // Set simulated "lpm r?,Z+; out DWDR,r?" instructions
                      0x20};                                              // Go
    sendCmd(rdFlash, sizeof(rdFlash));
    rsp = getResponse(data, len);                                         // Read len bytes
     if (rsp ==len) {
      break;
    } else {
      // Wait and retry read
      delay(5);
    }
  }
  reportTimeout = true;
  flashStart = addr;
  writeRegister(28, r28);             // Restore r28
  writeRegister(29, r29);             // Restore r29
  writeRegister(30, r30);             // Restore r30
  writeRegister(31, r31);             // Restore r31
  return rsp;
}

void printCommErr (byte read, byte expected) {
  print(F("\ndebugWire Communication Error: read "));
  printDec(read);
  print(F(" expected "));
  printDec(expected);
  println();
}

unsigned int getFlashWord (unsigned int addr) {
  if (!flashLoaded || addr < flashStart || addr >= flashStart + sizeof(flashBuf)) {
    byte len = readFlashPage(&flashBuf[0], sizeof(flashBuf), flashStart = addr);
    if (len != sizeof(flashBuf)) {
      printCommErr(len, sizeof(flashBuf));
    }
    flashLoaded = true;
  }
  int idx = addr - flashStart;
  return (flashBuf[idx + 1] << 8) + flashBuf[idx];
}

void printCmd (const __FlashStringHelper* cmd) {
  print(cmd);
  write(':');
  tabTo(8);
}

void printCmd () {
  write(':');
  tabTo(8);
}

void echoCmd () {
  print((char*) buf);
  write(':');
  tabTo(8);
}

void echoSetCmd (byte len) {
  for (byte ii = 0; ii < len; ii++) {
    write(buf[ii]);
  }
  print(":=");
  tabTo(8);
}

unsigned int getPc () {
  sendCmd((const byte[]) {0xF0}, 1);
  unsigned int pc = getWordResponse();
  return (pc - 1) * 2;
}

unsigned int getBp () {
  sendCmd((const byte[]) {0xF1}, 1);
  return (getWordResponse() - 1) * 2;
}

unsigned int getOpcode () {
  sendCmd((const byte[]) {0xF1}, 1);
  return getWordResponse();
}

void setPc (unsigned int pc) {
  pc = pc / 2;
  byte cmd[] = {0xD0, pc >> 8, pc & 0xFF};
  sendCmd(cmd, sizeof(cmd));
}

void setBp (unsigned int bp) {
  bp = bp / 2;
  byte cmd[] = {0xD1, bp >> 8, bp & 0xFF};
  sendCmd(cmd, sizeof(cmd));
}

void printDebugCommands () {
    print(F(
      "Debugging Commands:\n"
      "  HELP          Print this menu\n"
      "  REGS          Print All Registers 0-31\n"
      "  Rdd           Print Value of Reg dd (dd is a decimal value from 0 - 31)\n"
      "  Rdd=xx        Set Reg dd to New Value xx (dd is a decimal value from 0 - 31)\n"
      "  IOxx          Print Value of I/O space location xx\n"
      "  IOxx=yy       Set I/O space location xx to new value yy\n"
      "  IOxx.d=b      Change bit d (0-7) in I/O location xx to value b (1 or 0)\n"
      "  SRAMxxxx      Read and Print 32 bytes from SRAM address xxxx\n"
      "  SBxxxx        Print Byte Value of SRAM location xxxx\n"
      "  SBxxxx=yy     Set SRAM location xxxx to new byte value yy\n"
      "  SWxxxx        Print Word Value of SRAM location xxxx\n"
      "  SWxxxx=yyyy   Set SRAM location xxxx to new word value yyyy\n"
      "  EBxxxx        Print Byte Value of EEPROM location xxxx\n"
      "  EBxxxx=yy     Set EEPROM location xxxx to new byte value yy\n"
      "  EWxxxx        Print Word Value of EEPROM location xxxx\n"
      "  EWxxxx=yyyy   Set EEPROM location xxxx to new word value yyyy\n"
      "  CMD=xxxx      Send sequence of bytes xxxx... and show response\n"
      "  FWxxxx        Print 32 Word Values (64 bytes) from Flash addr xxxx\n"
      "  FBxxxx        Print 64 Byte Values from Flash addr xxxx and decode ASCII\n"
      "  LISTxxxx      Disassemble 16 words (32 bytes) from Flash addr xxxx\n"
      "  RUN           Start Execution at Current Value of PC (use BREAK to stop)\n"
      "  RUNxxxx       Start Execution at xxxx (use BREAK to stop)\n"
      "  RUNxxxx yyyy  Start Execution at xxxx with a Breakpoint set at yyyy\n"
      "  RUN xxxx      Start Execution at Current Value of PC with breakpoint at xxxx\n"
      "  BREAK         Send Async BREAK to Target (stops execution)\n"
      "  STEP          Single Step One Instruction at Current PC\n"
      "  RESET         Reset Target\n"
      "  EXIT          Exit from debugWire mode back to In-System\n"
      "  PC            Read and Print Program Counter\n"
      "  PC=xxxx       Set Program Counter to xxxx\n"
      "  SIG           Read and Print Device Signature\n"
#if DEVELOPER
      "Developer Commands:\n"
      "  CMD=xxxx      Send Sequence of Bytes xxxx... and show response\n"
      "  BP            Read and Print Breakpoint Register\n"
      "  BP=xxxx       Set Breakpoint Register to xxxxe\n"
      "  EXEC=xxxx     Execute Current Instruction opcode xxxxe\n"
      "  RAMSET        Init first 32 bytes of SRAMe\n"
      "  Dxxxx         Disassemble single word instruction opcode xxxxe\n"
      "  Dxxxx yyyy    Disassemble two word instruction opcode xxxx + yyyye\n"
#endif
  ));
}

void setRepeatCmd (const __FlashStringHelper* cmd, unsigned int addr) {
  strcpy_P(rpt, (char*) cmd);
  byte idx = (byte) strlen_P((char*) cmd);
  rpt[idx++] = toHexDigit((addr >> 12) & 0xF);
  rpt[idx++] = toHexDigit((addr >> 8) & 0xF);
  rpt[idx++] = toHexDigit((addr >> 4) & 0xF);
  rpt[idx++] = toHexDigit(addr & 0xF);
  rpt[idx] = 0;
}

void loop () {
  boolean mode = digitalRead(PMODE);
  if (runMode != mode) {
    runMode = mode;
    if (runMode) {
      selectProgrammer();
    } else {
      selectDebugger();
    }
  }
  if (mode) {
    programmer();
  } else {
    debugger();
  }
}

void debugger () {
  if (debugWireOn) {
    // This section implement the debugWire commands after being enabled by the break ('b') command
    if (Serial.available()) {
      if (getString() == 0 && rpt[0] != 0) {              // Check for repeated command
        strcpy(buf, rpt);
      }
      rpt[0] = 0;
      if (bufMatches(F("BREAK"))) {                       // BREAK - Send Async BREAK Target
        echoCmd();
        sendBreak();
        if (checkCmdOk()) {
          dAsm(pcSave = getPc(), 1);
          setBp(0);
          strcpy(rpt, "STEP");
        }
        breakWatch = false;
        return;
      } else if (breakWatch) {
        // Code is running, wait for BREAK or breakpoint
        return;
      }
      if (bufMatches(F("HELP"))) {                        // HELP - Prints menu of debug commands
        printDebugCommands();
      } else if (bufMatches(F("PC"))) {                   // PC - Read and Print Internal Value of Program Counter
        echoCmd();
        printHex16(pcSave);
        println();
        dAsm(pcSave, 1);
      } else if (bufMatches(F("PC=xXXX"))) {              // PC=xXXX - Set Program Counter to xXXX
        echoSetCmd(2);
        setPc(pcSave = convertHex(3));
        printHex16(pcSave);
        println();
        dAsm(pcSave, 1);
      } else if (bufMatches(F("RESET"))) {                // RESET - Reset Target
        echoCmd();
        sendCmd((const byte[]) {0x07}, 1);
        if (checkCmdOk2()) {
          setPc(pcSave = 0);
          // Preset registers 0-31
          for (byte ii = 0; ii < 32; ii++) {
            writeRegister(ii, ii);
          }
          dAsm(pcSave, 1);
        }
      } else if (bufMatches(F("SIG"))) {                  // SIG - Read and Print Device Signature
        echoCmd();
        sendCmd((const byte[]) {0xF3}, 1);
        printBufToHex8(getResponse(2), false);
        printPartFromId(buf[0], buf[1]);
      } else if (bufMatches(F("EXIT"))) {                 // EXIT - Exit from debugWire mode back to In-System
        echoCmd();
        sendCmd((const byte[]) {0x06}, 1);
        debugWireOn = false;
        enableSpiPins();
        println(F("debugWire temporarily Disabled"));
      } else if (bufMatches(F("REGS"))) {                 // REGS - Print Registers labelled as 0-31
        if (readAllRegisters()) {
          for (byte ii = 0; ii < 32; ii++) {
            print(ii < 10 ? F(" r") : F("r") );
            printDec(ii);
            write(':');
            printHex8(buf[ii]);
            if ((ii & 7) == 7) {
              println();
            } else {
              print(F(", "));
            }
          }
        } else {
          println(F("debugWire Communication Error"));
        }
      } else if (bufMatches(F("RdD"))) {                  // RdD - Print Reg dD, where dD is a 1, or 2 digit, decimal value
        echoCmd();
        byte reg = readDecimal(1);
        if (reg < 32) {
          printHex8(readRegister(reg));
          if (reg < 31) {
            // Set repeat command to show next register
            byte idx = 0;
            reg++;
            rpt[idx++] = 'R';
            if (reg >= 10) {
              rpt[idx++] = (reg / 10) + '0';
            }
            rpt[idx++] = (reg % 10) + '0';
            rpt[idx] = 0;
          }
        } else {
          print(F("Invalid register"));
        }
        println();
      } else if (bufMatches(F("RdD=xX"))) {               // RdD=xX - Set Reg dD to new value, where dD is a 1, or 2 digit, decimal value
        echoSetCmd(after('=') - 1);
        byte reg = readDecimal(1);
        if (reg < 32) {
          byte val = convertHex(after('='));
          writeRegister(reg, val);
          printHex8(val);
        } else {
          print(F("Invalid register"));
        }
        println();
      } else if (bufMatches(F("IOxX"))) {                 // IOxX - Print I/O space location xX, where xx is address for "in", or "out"
        // Set repeat command for I/O input polling
        strcpy(rpt, buf);
        echoCmd();
        unsigned int addr = convertHex(2);
        if (addr < 0x40) {
          printHex8(readSRamByte(addr + 0x20));
        } else {
          print(F("Invalid I/O Address, Range is 0x00 - 0x3F"));        
        }
        println();
      } else if (bufMatches(F("IOxX=xX"))) {              // IOxX=xX - Set I/O space location xx to new value
        echoSetCmd(after('=') - 1);
        unsigned int addr = convertHex(2);
        byte val = convertHex(after('='));
        if (addr < 0x40) {
          writeSRamByte(addr + 0x20, val);
          printHex8(val);
        } else {
          print(F("Invalid I/O Address, Range is 0x00 - 0x3F"));        
        }
        println();
      } else if (bufMatches(F("IOxX.o=b"))) {             // IOxX.0=b change bit 'o' in I/O location xx to 'b'
        echoSetCmd(after('=') - 1);
        byte addr = convertHex(2);
        if (addr < 0x20) {
          byte bit = readDecimal(after('.'));
          char ss = buf[after('=')];
          if (bit <= 7 && (ss == '1' || ss == '0')) {
            setClrIOBit(addr, bit, ss == '1');
            write(ss);
          } else {
            print(F("Invalid bit # or value"));        
          }
        } else {
          print(F("Invalid I/O Address, Range is 0x00 - 0x1F"));
        }
        println();
      } else if (bufMatches(F("SRAMxXXX"))) {             // SRAMxXXX - Read and Print 32 bytes from SRAM address xXXX
        unsigned int addr = convertHex(4);
        if (addr >= 0x20) {
          byte len = 32;
          if (readSRamBytes(addr, len)) {
           for (byte ii = 0; ii < len; ii++) {
              if ((ii & 0x0F) == 0) {
                write('S');
                printHex16(addr + ii);
                printCmd();
              }
              printHex8(buf[ii]);
              if ((ii & 0x0F) == 0x0F) {
                println();
              } else {
                write(' ');
              }
            }
          } else {
            println(F("Read Err"));
          }
        } else {
          write('S');
          printHex16(addr);
          printCmd();
          print(F("Invalid Address, Must be >= 0x20"));
          println();
        }
        setRepeatCmd(F("SRAM"), addr + 32);
      } else if (bufMatches(F("SBxXXX"))) {               // SBxXXX - Print byte value of SRAM location xXXX
        unsigned int addr = convertHex(2);
        write('S');
        printHex16(addr);
        printCmd();
        if (addr >= 0x20) {
          printHex8(readSRamByte(addr));
        } else {
          print(F("Invalid Address, Must be >= 0x20"));
        }
        println();
        setRepeatCmd(F("SB"), addr + 1);
      } else if (bufMatches(F("SBxXXX=xX"))) {            // SBxXXX=xX - Set SRAM location xXXX to new byte value xX
        unsigned int addr = convertHex(2);
        write('S');
        printHex16(addr);
        printCmd(F("="));
        if (addr >= 0x20) {
          byte val = convertHex(after('='));
          writeSRamByte(addr, val);
          printHex8(val);
        } else {
          print(F("Invalid Address, Must be >= 0x20"));
        }
        println();
      } else if (bufMatches(F("SWxXXX"))) {               // SWxXXX - Print word value of SRAM location xXXX
        unsigned int addr = convertHex(2);
        write('S');
        printHex16(addr);
        printCmd();
        if (addr >= 0x20) {
         // 16 Bit word bytes are in LSB:MSB Order
          printHex8(readSRamByte(addr + 1));
          printHex8(readSRamByte(addr));
        } else {
          print(F("Invalid Address, Must be >= 0x20"));
        }
        println();
        setRepeatCmd(F("SW"), addr + 2);
      } else if (bufMatches(F("SWxXXX=xXXX"))) {          // SWxXXX=xXXX - Set SRAM location xXXX to new word value xX
        unsigned int addr = convertHex(2);
        unsigned int data =  convertHex(after('='));
        write('S');
        printHex16(addr);
        printCmd(F("="));
        if (addr >= 0x20) {
          writeSRamByte(addr, data & 0xFF);
          writeSRamByte(addr + 1, data >> 8);
          printHex16(data);
        } else {
          print(F("Invalid Address, Must be >= 0x20"));
        }
        println();
      } else if (bufMatches(F("EBxXXX"))) {               // EBxXXX - Print value of EEPROM location xXXX
        unsigned int addr = convertHex(2);
        write('E');
        printHex16(addr);
        printCmd();
        if (addr < eeSize) {
          printHex8(readEepromByte(addr));
        } else {
          print(F("Invalid EEPROM Address"));        
        }
        println();     
        setRepeatCmd(F("EB"), addr + 1);
      } else if (bufMatches(F("EBxXXX=xX"))) {            // EBxXXX=xX - Set EEPROM location xXXX to new value xX
        unsigned int addr = convertHex(2);
        write('E');
        printHex16(addr);
        printCmd(F("="));
        if (addr < eeSize) {
          byte val = convertHex(after('='));
          writeEepromByte(addr, val);
          printHex8(val);
        } else {
          print(F("Invalid EEPROM Address"));        
        }
        println();
      } else if (bufMatches(F("EWxXXX"))) {               // EWxXXX - Print word value of EEPROM location xXXX
        unsigned int addr = convertHex(2);
        write('E');
        printHex16(addr);
        printCmd();
        if (addr < eeSize) {
         // 16 Bit word bytes are in LSB:MSB Order
         unsigned int data = ((unsigned int) readEepromByte(addr + 1) << 8) + readEepromByte(addr);
         printHex16(data);
        } else {
          print(F("Invalid EEPROM Address"));        
        }
        println();
        setRepeatCmd(F("EW"), addr + 2);
      } else if (bufMatches(F("EWxXXX=xXXX"))) {          // EWxXXX=xXXX - Set EEPROM location xXXX to new word value xX
        unsigned int addr = convertHex(2);
        unsigned int data =  convertHex(after('='));
        write('E');
        printHex16(addr);
        printCmd(F("="));
        if (addr < eeSize) {
          writeEepromByte(addr, data & 0xFF);
          writeEepromByte(addr + 1, data >> 8);
          printHex16(data);
        } else {
          print(F("Invalid EEPROM Address"));        
        }
        println();
      } else if (bufMatches(F("FBxXXX"))) {               // FBxXXX - Print 64 bytes from Flash addr xXXX
        unsigned int addr = convertHex(2);
        unsigned int count = readFlashPage(&buf[0], sizeof(flashBuf), addr);
        if (count == sizeof(flashBuf)) {
          for (byte ii = 0; ii < count; ii++) {
            if ((ii & 0x0F) == 0) {
              write('F');
              printHex16(addr + ii);
              printCmd();
            }
            printHex8(buf[ii]);
            if ((ii & 0x0F) == 0x0F) {
              // Print ASCII equivalents for characters <= 0x7F and >= 0x20
              byte idx = ii & 0xF0;
              print(F("    "));
              for (byte jj = 0; jj < 16; jj++) {
                char cc = buf[idx + jj];
                write(cc >= 0x20 && cc <= 0x7F ? cc : '.');
              }
              println();
            } else {
              write(' ');
            }
          }
          setRepeatCmd(F("FB"), addr + 128);
        } else {
          printCommErr(count, sizeof(flashBuf));
        }
      } else if (bufMatches(F("FWxXXX"))) {               // FWxXXX - Print 32 words (64 bytes) from Flash addr xXXX
        unsigned int addr = convertHex(2);
        unsigned int count = readFlashPage(&buf[0], sizeof(flashBuf), addr);
        if (count == sizeof(flashBuf)) {
          for (byte ii = 0; ii < count; ii += 2) {
            if ((ii & 0x0E) == 0) {
              write('F');
              printHex16(addr + ii);
              printCmd();
            }
            // 16 Bit Opcode is LSB:MSB Order
            printHex8(buf[ii + 1]);
            printHex8(buf[ii]);
            if ((ii & 0x0E) == 0x0E) {
              println();
            } else {
              write(' ');
            }
          }
          setRepeatCmd(F("FW"), addr + 128);
        } else {
          printCommErr(count, sizeof(flashBuf));
        }
      } else if (bufMatches(F("LxXXX"))) {                // LxXXX - Disassemble 16 words (32 bytes) from Flash addr xXXX
        unsigned int addr = convertHex(1);
        dAsm(addr, 16);
        setRepeatCmd(F("L"), addr + 32);
      } else if (bufMatches(F("STEP"))) {                 // STEP - Single Step One Instruction at Current PC
        echoCmd();
        unsigned int opcode = getFlashWord(pcSave);
        if (opcode == 0x9598) {
          // Skip over "break" instruction
          println(F("Skipping break"));
          dAsm(pcSave += 2, 1);
        } else {
          if (true) {
            readSRamByte((unsigned int) eecr + 0x2C);     // Reading EECR register seems to fix unexpected EE_RDY interrupt issue
            unsigned int pc = pcSave / 2;
            byte cmd[] = {0xD0, pc >> 8, pc, 0x31};
            sendCmd(cmd, sizeof(cmd));
          } else {
            setPc(pcSave);
            sendCmd((const byte[]) {0x31}, 1);
            //sendCmd((const byte[]) {0x60, 0x31}, 2);
          }
          if (checkCmdOk2()) {
            dAsm(pcSave = getPc(), 1);
          }
        }
        strcpy(rpt, "STEP");
      } else if (bufMatches(F("RUNxXXX xXXXX"))) {         // RUNxXXX xXXXX - Start execution at loc xXXX with breakpoint at 2nd xXXX
        echoSetCmd(3);
        unsigned int pc = convertHex(3);
        unsigned int bp = convertHex(after(' '));
        print(F("PC:"));
        printHex16(pc);
        print(F(" BP:"));
        printHex16(bp);
        // Set Run To Cursor (BP) Mode (timers enabled)
        pcSave = pc;
        bpSave = bp;
        pc /= 2;
        bp /= 2;
        // Needs 0x41/0x61 or break doesn't happen
        byte cmd[] = {0x61, 0xD1, bp >> 8, bp, 0xD0, pc >> 8, pc, 0x30};
        sendCmd(cmd, sizeof(cmd));
        println();
        breakWatch = true;
      } else if (bufMatches(F("RUN xXXX"))) {             // RUN xXXX - Start execution at current PC with breakpoint at xXXX
        echoSetCmd(3);
        unsigned int bp = convertHex(4);
        print(F("PC:"));
        printHex16(pcSave);
        print(F(" BP:"));
        printHex16(bp);
        setBp(bpSave = bp);
        // Set Run To Cursor (BP) Mode (timers enabled)
        bpSave = bp;
        bp /= 2;
        unsigned int pc = pcSave / 2;
        // Needs 0x41/0x61 or break doesn't happen
        byte cmd[] = {0x61, 0xD1, bp >> 8, bp, 0xD0, pc >> 8, pc, 0x30};
        sendCmd(cmd, sizeof(cmd));
        println();
        breakWatch = true;
      } else if (bufMatches(F("RUNxXXX"))) {              // RUNxXXX - Start execution on Target at xXXX
        echoSetCmd(3);
        unsigned int pc = convertHex(3);
        print(F("PC:"));
        printHex16(pc);
        // Set Run without breakpoint (timers enabled)
        pcSave = pc;
        pc /= 2;
        byte cmd[] = {0x60, 0xD0, pc >> 8, pc, 0x30};
        sendCmd(cmd, sizeof(cmd));
        println(F(" Running"));
        breakWatch = true;
      } else if (bufMatches(F("RUN"))) {                  // RUN code from current PC
        echoCmd();
        unsigned int opcode = getFlashWord(pcSave);
        // Skip over "break" instruction, if needed
        pcSave = opcode == 0x9598 ? pcSave + 2 : pcSave;
        // Set Run without breakpoint (timers enabled)
        unsigned int pc = pcSave / 2;
        byte cmd[] = {0x60, 0xD0, pc >> 8, pc, 0x30};
        sendCmd(cmd, sizeof(cmd));
        println(F("Running"));
        breakWatch = true;
#if 0
      // Still trying to make this work...
      } else if (bufMatches(F("RETURN"))) {               // RETURN - RUN code from current PC and break on return
        echoCmd();
        unsigned int opcode = getFlashWord(pcSave);
        // Skip over "break" instruction, if needed
        pcSave = opcode == 0x9598 ? pcSave + 2 : pcSave;
        // Set Run without breakpoint (timers enabled)
        unsigned int pc = pcSave / 2;
        byte cmd[] = {0x63, 0xD0, pc >> 8, pc, 0x30};
        sendCmd(cmd, sizeof(cmd));
        println(F("Running"));
        breakWatch = true;
#endif
#if DEVELOPER
      } else if (bufMatches(F("REGS=xX"))) {              // REGS=xx set all registers to value of xx
        echoSetCmd(4);
        byte val = convertHex(5);
        printHex8(val);
        for (byte ii = 0; ii < 32; ii++) {
          writeRegister(ii, val);
        }
        println();
      } else if (bufMatches(F("BP"))) {                   // BP - Read and Print Breakpoint Register
        echoCmd();
        printHex16(getBp());
        println();
      } else if (bufMatches(F("BP=xXXX"))) {              // BP=xXXX - Set Breakpoint Register to xXXX
        unsigned int bp = convertHex(3);
        echoSetCmd(2);
        setBp(bp);
        printHex16(bp);
        println();
      } else if (bufStartsWith(F("CMD="))) {              // CMD=xx - Send sequence of bytes and show response
        echoSetCmd(3);
        byte cnt = convertToHex(4);
        for (byte ii = 0; ii < cnt; ii++) {
          printHex8(buf[ii]);
          print(" ");
        }
        sendCmd(&buf[0], cnt);
        byte rsp = getResponse(0);
        printCmd(F("RSP"));
        printBufToHex8(rsp, true);
      } else if (bufMatches(F("EXEC=xxxx"))) {            // EXEC=xxxx - Execute Instruction opcode xxxx
        echoSetCmd(4);
        unsigned int opcode = convertHex(5);
        byte cmd[] = {0xD2, opcode >> 8, opcode & 0xFF};
        sendCmd(cmd, sizeof(cmd));
        sendCmd((const byte[]) {0x64, 0x23}, 2);
        printHex16(opcode);
        print("  ");
        cursor = 13;
        dAsm2Byte(0, opcode);
        println();
      } else if (bufMatches(F("RAMSET"))) {               // RAMSET - Init first 23 bytes of SRAM
        echoCmd();
        // Preset first 32 bytes of SRAM
        for (byte ii = 0; ii < 32; ii++) {
          writeSRamByte(ramBase + ii, ii);
          write('.');
        }
        println();
      } else if (bufMatches(F("Dxxxx xxxx"))) {           // Dxxxx xxxx - Disassemble two word instruction opcode xxxx + xxxx
        // Used during development of the disasembler
        println((char *) buf);
        unsigned int tmp = convertHex(1);
        flashBuf[0] = tmp;
        flashBuf[1] = tmp >> 8;
        tmp = convertHex(6);
        flashBuf[2] = tmp;
        flashBuf[3] = tmp >> 8;
        flashLoaded = true;
        flashStart = 0;
        dAsm(0, 1);
        flashLoaded = false;
      } else if (bufMatches(F("Dxxxx"))) {                // Dxxxx - Disassemble single word instruction opcode xxxx
        // Used during development of the disasembler
        println((char *) buf);
        unsigned int tmp = convertHex(1);
        flashBuf[0] = tmp;
        flashBuf[1] = tmp >> 8;
        flashLoaded = true;
        flashStart = 0;
        dAsm(0, 1);
        flashLoaded = false;
#endif
#if 0
        /*
         * Unfortunately, enabling access to the Bootloader via debugWire commands doesn't seem to work
         */
      } else if (bufMatches(F("BOOT"))) {                 // BOOT - Display status of IVSEL bit in MCUCR Register (0x35)
        echoCmd();
        byte r16 = readRegister(16);            // Save r16
        byte cmd[] = {0x64,
                      0xD2, inHigh(0x3, 16), inLow(0x35, 16), 0x23,       // Build "in r16, 0x35" instruction
                      0xD2, outHigh(dwdr, 16), outLow(dwdr, 16), 0x23};   // Build "out DWDR, reg" instruction
        sendCmd(cmd,  sizeof(cmd));
        getResponse(1);
        writeRegister(16, r16);                 // Restore r16
        print(F("MCUCR = "));
        printHex8(buf[0]);
        println();
      } else if (bufMatches(F("BOOT ON"))) {              // BOOT ON - Enable Access to Bootloader
        echoCmd();
        println();
        byte r16 = readRegister(16);            // Save r16
        byte cmd[] = {0x64,
                      0xD2, 0xE0, 0x01, 0x23,   // ldi r16, 0x01 - MCUCR = (1 << IVCE);
                      0xD2, 0xBF, 0x05, 0x23,   // out 0x35, r16
                      0xD2, 0xE0, 0x02, 0x23,   // ldi r16, 0x02 - MCUCR = (1 << IVSEL);
                      0xD2, 0xBF, 0x05, 0x23};  // out 0x35, r16
        sendCmd(cmd,  sizeof(cmd));
        writeRegister(16, r16);                 // Restore r16
      } else if (bufMatches(F("BOOT OFF"))) {             // BOOT OFF - Disable Access to Bootloader
        echoCmd();
         println();
       byte r16 = readRegister(16);            // Save r16
        byte cmd[] = {0x64,
                      0xD2, 0xE0, 0x01, 0x23,   // ldi r16, 0x01 - MCUCR = (1 << IVCE);
                      0xD2, 0xBF, 0x05, 0x23,   // out 0x35, r16
                      0xD2, 0xE0, 0x00, 0x23,   // ldi r16, 0x00 - MCUCR = 0;
                      0xD2, 0xBF, 0x05, 0x23};  // out 0x35, r16
        sendCmd(cmd,  sizeof(cmd));
        writeRegister(16, r16);                 // Restore r16
#endif
      } else {
        print((char *) buf);
        println(F(" ?"));
      }
    } else if (breakWatch) {
      if (debugWire.available()) {
        byte cc = debugWire.read();
        if (cc == 0x55) {
          println(F("BREAKPOINT"));
          dAsm(pcSave = getPc(), 1);
          strcpy(rpt, "STEP");
          breakWatch = false;
        }
      }
    }
  } else {
    // This section implements the In-System Programming Commands in a way that's compatible with using it to
    // program AVR Chips from ATTinyIDE
    if (Serial.available()) {
      char cc = toupper(Serial.read());
      switch (cc) {
#if 0
        case 'E':                                           // Erase Flash and EEPROM Memory
          if (enterProgramMode()) {
            ispSend(0xAC, 0x80, 0x00, 0x00);
            busyWait();
            println(F("Erased"));
          }
          break;
#endif
          
#if DEVELOPER
        case 'C':                                           // Send arbitrary 4 byte command sequence
          if (enterProgramMode()) {
            getString();
            if (convertToHex(0) == 4) {
              print(F("Cmd:   "));
              printBufToHex8 (4, false);
              print(F(" = "));
              ispSend(buf[0], buf[1], buf[2], buf[3]);
              println();
            } else {
              println(F("Err: 4 bytes needed"));
            }
          } 
          break;
#endif
        case 'F':                                             // Identify Device and Print current fuse settings
          if (!hasDeviceInfo) {
            identifyDevice();
          }
          if (hasDeviceInfo && enterProgramMode()) {
            byte lowFuse, highFuse;
            print(F("Low: "));
            printHex8(lowFuse = ispSend(0x50, 0x00, 0x00, 0x00));
            print(F(", High: "));
            printHex8(highFuse = ispSend(0x58, 0x08, 0x00, 0x00));
            print(F(", Extd: "));
            printHex8(ispSend(0x50, 0x08, 0x00, 0x00));
            print(F(" - CHDIV8 "));
            print((ckdiv8 & lowFuse) != 0 ? F("Disabled") : F("Enabled") );
            print(F(", DWEN "));
            println((dwen & highFuse) != 0 ? F("Disabled") : F("Enabled") );
          }
          break;
          
        case '8':                                             // Enable CKDIV8 (divide clock by 8)
        case '1':                                             // Disable CKDIV8
          if (!hasDeviceInfo) {
            identifyDevice();
          }
          if (ckdiv8 != 0 && enterProgramMode()) {
             byte lowFuse = ispSend(0x50, 0x00, 0x00, 0x00);
             byte newFuse;
             boolean enable = cc == '8';
            if (enable) {
              // Enable CKDIV8 fuse by setting bit LOW
              newFuse = lowFuse & ~ckdiv8;
            } else {
              // Disable CKDIV8 fuse by setting bit HIGHW
              newFuse = lowFuse + ckdiv8;
            }
            if (newFuse != lowFuse) {
              ispSend(0xAC, 0xA0, 0x00, newFuse);
              print(F("CKDIV8 Fuse "));
              println(enable ? F("Enabled") : F("Disabled"));
            } else {
              print(F("CKDIV8 Fuse Already "));
              println(enable ? F("Enabled") : F("Disabled"));
            }
          } else {
            println(F("Unable to change CKDIV8 fuse"));
          }
          break;
          
        case '+':                                             // Enable debugWire Fuse
        case '-':                                             // Disable debugWire Fuse
          if (!hasDeviceInfo) {
            identifyDevice();
          }
          if (dwen != 0 && enterProgramMode()) {
            byte highFuse = ispSend(0x58, 0x08, 0x00, 0x00);
            byte newFuse;
            boolean enable = cc == '+';
            if (enable) {
              // Enable debugWire DWEN fuse by setting bit LOW
              newFuse = highFuse & ~dwen;
            } else {
              // Disable debugWire DWEN fuse by setting bit HIGH
              newFuse = highFuse + dwen;
            }
            if (newFuse != highFuse) {
              ispSend(0xAC, 0xA8, 0x00, newFuse);
              print(F("DWEN Fuse "));
              println(enable ? F("Enabled") : F("Disabled"));
            } else {
              print(F("DWEN Fuse Already "));
              println(enable ? F("Enabled") : F("Disabled"));
            }
          } else {
            println(F("Unable to change DWEN fuse"));
          }
          break;
          
#if DEVELOPER
        case 'P':                                             // Turn on power to chip (used to run code)
          powerOff();
          digitalWrite(VCC, HIGH);
          digitalWrite(RESET, LOW);
          pinMode(RESET, INPUT);
          pinMode(VCC, OUTPUT);
          println(F("VCC On"));
          break;
          
        case 'O':                                             // Switch off power to chip
          powerOff();
          println(F("VCC off"));
          break;
#endif
      
        case 'B':                                             // Cycle Vcc and Send BREAK to engage debugWire Mode
          if (doBreak()) {
            disableSpiPins();
            printCmd(F("SIG"));
            sendCmd((const byte[]) {0xF3}, 1);
            printBufToHex8(getResponse(2), false);
            printPartFromId(buf[0], buf[1]);
            print(F("Flash:  "));
            Serial.print(flashSize, DEC);
            println(F(" bytes"));
            print(F("SRAM:   "));
            Serial.print(ramSize, DEC);
            println(F(" bytes"));
            print(F("SRBase: 0x"));
            printHex16(ramBase);
            println();
            print(F("EEPROM: "));
            Serial.print(eeSize, DEC);
            println(F(" bytes"));
#if DEVELOPER
            print(F("DWEN:   0x"));
            printHex8(dwen);
            println();
            print(F("CKDIV8: 0x"));
            printHex8(ckdiv8);
            println();
#endif
            pcSave = 0;
          }
          break;

        default:
          printMenu();
          break;
      }
      // Gobble up any unused characters
      while (Serial.available()) {
        Serial.read();
      }
    }
  }
}

void printMenu () {
  println(F("Commands:"));
  println(F(" F - Identify Device & Print Fuses"));
  println(F(" + - Enable debugWire DWEN Fuse"));
  println(F(" - - Disable debugWire DWEN Fuse"));
  println(F(" 8 - Enable CKDIV8 (divide clock by 8)"));
  println(F(" 1 - Disable CKDIV8"));
  println(F(" B - Engage Debugger"));
#if 0
  println(F(" E - Erase Flash & EEPROM"));
#endif
#if DEVELOPER
  println(F(" C - Send 4 Byte ISP Command"));
  println(F(" P - Vcc On"));
  println(F(" O - Vcc Off"));
#endif 
}

   //
   // The next section implements a basic disassembler for the AVR Instruction Set which is available in debugWire mode
   // using thecommand  "Lxxxx", where xxxx is a 16 bit hexadecimal address.
   // Note: this dissaambler was written quickly and crudely so there may be errors, or omissions in its output
   //

#define XX  1
#define YY  2
#define ZZ  3

byte    srDst, srSrc;
boolean showStatus, skipWord;
boolean showSrc, showDst, srcPair, dstPair;
byte    dstReg, srcReg;

void dAsm (unsigned int addr, byte count) {
  for (byte ii = 0; ii < count; ii++) {
    cursor = 0;
    showStatus = skipWord = false;
    showDst = showSrc = false;
    srcPair = dstPair = false;
    byte idx = ii * 2;
    unsigned int word2;
    // 16 Bit Opcode is LSB:MSB Order
    unsigned int opcode = getFlashWord(addr + idx);
    printHex16(addr + idx);
    printCmd();
    printHex16(opcode);
    tabTo(14);
    if ((opcode & ~0x1F0) == 0x9000) {                        // lds (4 byte instruction)
      printInst(F("lds"));
      printDstReg((opcode & 0x1F0) >> 4);
      print(F(",0x"));
      word2 = getFlashWord(addr + idx + 2);
      printHex16(word2);
      skipWord = true;
    } else if ((opcode & ~0x1F0) == 0x9200) {                 // sts (4 byte instruction)
      printInst(F("sts"));
      print(F("0x"));
      word2 = getFlashWord(addr + idx + 2);
      printHex16(word2);
      printSrcReg((opcode & 0x1F0) >> 4);
      skipWord = true;
    } else if ((opcode & 0x0FE0E) == 0x940C) {                // jmp (4 byte instruction)
      printInst(F("jmp"));
      print(F("0x"));
      word2 = getFlashWord(addr + idx + 2);
      // 22 bit address
      long add22 = (opcode & 0x1F0) << 13;
      add22 += (opcode & 1) << 16;
      add22 += word2;
      printHex24(add22 * 2);
      skipWord = true;
    } else if ((opcode & 0x0FE0E) == 0x940E) {                // call (4 byte instruction)
      printInst(F("call"));
      print(F("0x"));
      word2 = getFlashWord(addr + idx + 2);
      // 22 bit address
      long add22 = (opcode & 0x1F0) << 13;
      add22 += (opcode & 1) << 16;
      add22 += word2;
      printHex24(add22 * 2);
      skipWord = true;
    } else {
     dAsm2Byte(addr + idx, opcode);
    }
   // If single stepping, show register and status information
    if (count == 1) {
      printRegsUsed();
      if (showStatus) {
        printStatus();
      }
    }
    if (skipWord) {
      // Print 2nd line to show extra word used by 2 word instructions
      println();
      printHex16(addr + idx + 2);
      write(':');
      tabTo(8);
      printHex16(word2);
      ii++;
    }
    println();
  }
}

  /*
   * Specials case instructions: implement?
   *    ELPM  95D8
   *    LPM   95C8
   *    
   * Special case, implied operand display?
   *    SPM   Z+,r1:r0
   */

// Disassemble 2 byte Instructions
void dAsm2Byte (unsigned int addr, unsigned int opcode) {
  srDst = srSrc = 0;
  if (dAsmNoArgs(opcode)) {                                         // clc, clh, etc.
  } else if (dAsmLogic(opcode)) {                                   // 
    printDstReg((opcode & 0x1F0) >> 4);
  } else if ((srDst = dAsmXYZStore(opcode)) != 0) {                 //
    printSrcReg((opcode & 0x1F0) >> 4);
  } else if (dAsmBranch(opcode)) {                                  // Branch instruction
    int delta = ((opcode & 0x200 ? (int) (opcode | 0xFC00) >> 3 : (opcode & 0x3F8) >> 3) + 1) * 2;
    printHex16(addr + delta);
    showStatus = true;
  } else if (dAsmArith(opcode)) {
    printDstReg((opcode & 0x1F0) >> 4);
    printSrcReg(((opcode & 0x200) >> 5) + (opcode & 0x0F));
  } else if (dAsmBitOps(opcode)) {                                  // 
    printDstReg((opcode & 0x1F0) >> 4);
    print(",");
    printDec(opcode & 0x07);
  } else if (dAsmLddYZQ(opcode)) {                                  // ldd rn,Y+q, ldd rn,Z+q
    // Handled in function
  } else if (dAsmStdYZQ(opcode)) {                                  // std Y+q,rn, std Z+q,rn
    // Handled in function
  } else if (dAsmXYZLoad(opcode)) {                                 // 
    // Handled in function
  } else if (dAsmRelCallJmp(opcode)) {
    int delta = ((opcode & 0x800 ? (int) (opcode | 0xF000) : (int) (opcode & 0xFFF)) + 1) * 2;
    printHex16(addr + delta);
  } else if ((opcode & ~0x7FF) == 0xB000) {                          // in rn,0xnn
    printInst(F("in"));
    printDstReg((opcode & 0x1F0) >> 4);
    print(F(",0x"));
    printHex8(((opcode & 0x600) >> 5) + (opcode & 0x0F));
  } else if ((opcode & ~0x7FF) == 0xB800) {                           // out 0xnn,rn
    printInst(F("out"));
    print(F("0x"));
    printHex8(((opcode & 0x600) >> 5) + (opcode & 0x0F));
    printSrcReg((opcode & 0x1F0) >> 4);
  } else if (dAsmByteImd(opcode)) {                                   // cpi, sbci, subi, ori, andi or ldi
    printDstReg(((opcode & 0xF0) >> 4) + 16);
    print(F(",0x"));
    printHex8(((opcode & 0xF00) >> 4) + (opcode & 0x0F));
  } else if ((opcode & 0x7FF) == 0xA000) {                            // lds
    printInst(F("lds"));
    printDstReg(((opcode & 0xF0) >> 4) + 16);
    print(F(",0x"));
    printHex8(((opcode & 0x700) >> 4) + (opcode & 0x0F) + 0x40);
  } else if ((opcode & 0x7FF) == 0xA800) {                            // sts
    printInst(F("sts"));
    print(F("0x"));
    printHex8(((opcode & 0x700) >> 4) + (opcode & 0x0F) + 0x40);
    printSrcReg(((opcode & 0xF0) >> 4) + 16);
  } else if (dAsmSetClr(opcode)) {                                    // bclr or bset
    print(" ");
    printDec((opcode & 0x70) >> 4);
  } else if (dAsmBitOps2(opcode)) {                                   // cbi, sbi, sbic or sbis
    print(F("0x"));
    printHex8((opcode & 0xF8) >> 3);
    print(",");
    printDec( opcode & 0x07);
  } else if (dAsmWordImd(opcode)) {                                   // adiw or sbiw
    printDstPair(((opcode & 0x30) >> 4) * 2 + 24);
    print(F(",0x"));
    printHex8(((opcode & 0xC0) >> 4) + (opcode & 0x0F));
  } else if (dAsmMul(opcode)) {                                       // mulsu, fmul, fmuls or fmulsu
    printDstReg(((opcode & 7) >> 4) + 16);
    printSrcReg((opcode & 0x07) + 16);
  } else if ((opcode & ~0xFF) == 0x0100) {                            // movw r17:16,r1:r0
    printInst(F("movw"));
    printDstPair(((opcode & 0xF0) >> 4) * 2);
    print(",");
    printSrcPair((opcode & 0x0F) * 2);
  } else if ((opcode & ~0xFF) == 0x0200) {                            // muls r21,r20
    printInst(F("muls"));
    printDstReg(((opcode & 0xF0) >> 4) + 16);
    printSrcReg((opcode & 0x0F) + 16);
  }
}

  // Print functions that track cursor position (to support tabbing)

void print (char *txt) {
  cursor += strlen(txt);
  Serial.print(txt);
}

void print (const __FlashStringHelper* txt) {
  cursor += strlen_P((char *) txt);
  Serial.print(txt);
}

void println (char *txt) {
  Serial.println(txt);
  cursor = 0;
}

void println (const __FlashStringHelper* txt) {
  Serial.println(txt);
  cursor = 0;
}

void println () {
  Serial.println();
  cursor = 0;
}

void write (char cc) {
  Serial.write(cc);
  cursor++;
}

void tabTo (byte pos) {
  while (cursor < pos) {
    Serial.write(' ');
    cursor++;
  }
}

void printHex (byte val) {
  Serial.print(val, HEX);
  cursor += val < 0x10 ? 1 : 2;
}

void printHex8 (byte val) {
  if (val < 0x10) {
    Serial.print("0");
  }
  Serial.print(val, HEX);
  cursor += 2;
}

void printHex16 (unsigned int val) {
  printHex8(val >> 8);
  printHex8(val);
}

void printHex24 (unsigned long val) {
  printHex8(val >> 16);
  printHex8(val >> 8);
  printHex8(val);
}

void printDec (byte val) {
  Serial.print(val, DEC);
  if (val < 10) {
    cursor++;
  } else if (val < 100) {
    cursor += 2;
  } else {
    cursor += 3;
  }
}

void printStatus () {
  tabTo(36);
  print(F("; "));
  byte stat = readSRamByte(0x3F + 0x20);
  write((stat & 0x80) != 0 ? 'I' : '-');
  write((stat & 0x40) != 0 ? 'T' : '-');
  write((stat & 0x20) != 0 ? 'H' : '-');
  write((stat & 0x10) != 0 ? 'S' : '-');
  write((stat & 0x08) != 0 ? 'V' : '-');
  write((stat & 0x04) != 0 ? 'N' : '-');
  write((stat & 0x02) != 0 ? 'Z' : '-');
  write((stat & 0x01) != 0 ? 'C' : '-');
}

void printSpecialReg (byte sr) {
  switch (sr) {
    case XX:
      printHex8(readRegister(27));
      printHex8(readRegister(26));
      break;
    case YY:
      printHex8(readRegister(29));
      printHex8(readRegister(28));
      break;
    case ZZ:
      printHex8(readRegister(31));
      printHex8(readRegister(30));
      break;
  }
}

void printRegsUsed () {
  if (showSrc || showDst || srSrc != 0 || srDst != 0) {
    tabTo(36);
    print(F("; "));
    if (srDst != 0) {
      printSpecialReg(srDst);
    } else if (showDst) {
      if (dstPair) {
        printHex8(readRegister(dstReg + 1));
        write(':');
        printHex8(readRegister(dstReg));
      } else {
        printHex8(readRegister(dstReg));
      }
    }
    if (showSrc || srSrc != 0) {
      if (showDst || srDst != 0) {
        print(F(", "));
      }
      if (showSrc) {
        if (srcPair) {
          printHex8(readRegister(srcReg + 1));
          write(':');
          printHex8(readRegister(srcReg));
        } else {
          printHex8(readRegister(srcReg));
        }
      } else {
        printSpecialReg(srSrc);
      }
    }
  }
}

void printInst (const __FlashStringHelper* str) {
  print(str);
  tabTo(20);
}

void printDstReg (byte reg) {
  print(F("r"));
  printDec(dstReg = reg);
  showDst = true;
}

void printSrcReg (byte reg) {
  print(F(",r"));
  printDec(srcReg = reg);
  showSrc = true;
}

void printDstPair (byte reg) {
  print(F("r"));
  printDec(reg + 1);
  print(F(":r"));
  printDec(dstReg = reg);
  showDst = true;
  dstPair = true;
}

void printSrcPair (byte reg) {
  print(F("r"));
  printDec(reg + 1);
  print(F(":r"));
  printDec(srcReg = reg);
  showSrc = true;
  srcPair = true;
}

boolean dAsmNoArgs (unsigned int opcode) {
  switch (opcode) {
    case 0x9598: print(F("break"));   return true;
    case 0x9488: print(F("clc"));     return true;
    case 0x94d8: print(F("clh"));     return true;
    case 0x94f8: print(F("cli"));     return true;
    case 0x94a8: print(F("cln"));     return true;
    case 0x94c8: print(F("cls"));     return true;
    case 0x94e8: print(F("clt"));     return true;
    case 0x94b8: print(F("clv"));     return true;
    case 0x9498: print(F("clz"));     return true;
    case 0x9519: print(F("eicall"));  return true;
    case 0x9419: print(F("eijmp"));   return true;
    case 0x95d8: print(F("elpm"));    return true;
    case 0x9509: print(F("icall"));   return true;
    case 0x9409: print(F("ijmp"));    return true;
    case 0x95c8: print(F("lpm"));     return true;
    case 0x0000: print(F("nop"));     return true;
    case 0x9508: print(F("ret"));     return true;
    case 0x9518: print(F("reti"));    return true;
    case 0x9408: print(F("sec"));     return true;
    case 0x9458: print(F("seh"));     return true;
    case 0x9478: print(F("sei"));     return true;
    case 0x9428: print(F("sen"));     return true;
    case 0x9448: print(F("ses"));     return true;
    case 0x9468: print(F("set"));     return true;
    case 0x9438: print(F("sev"));     return true;
    case 0x9418: print(F("sez"));     return true;
    case 0x9588: print(F("sleep"));    return true;
    case 0x95e8: print(F("spm"));     return true;
    case 0x95f8: print(F("spm"));     return true;
    case 0x95a8: print(F("wdr"));     return true;
  }
  return false;
}

boolean dAsmSetClr (unsigned int opcode) {
  switch (opcode & ~0x0070) {
    case 0x9488: printInst(F("bclr")); return true;
    case 0x9408: printInst(F("bset")); return true;
  }
  return false;
}

boolean dAsmLogic (unsigned int opcode) {
  switch (opcode & ~0x01F0) {
    case 0x900f: printInst(F("pop"));   return true;
    case 0x920f: printInst(F("push"));  return true;
    case 0x9400: printInst(F("com"));   return true;
    case 0x9401: printInst(F("neg"));   return true;
    case 0x9402: printInst(F("swap"));  return true;
    case 0x9403: printInst(F("inc"));   return true;
    case 0x9405: printInst(F("asr"));   return true;
    case 0x9406: printInst(F("lsr"));   return true;
    case 0x9407: printInst(F("ror"));   return true;
    case 0x940a: printInst(F("dec"));   return true;
  }
  return false;
}

byte dAsmXYZStore (unsigned int opcode) {
  switch (opcode & ~0x01F0) {
    case 0x9204: printInst(F("xch")); print(F("Z"));  return ZZ;
    case 0x9205: printInst(F("las")); print(F("Z"));  return ZZ;
    case 0x9206: printInst(F("lac")); print(F("Z"));  return ZZ;
    case 0x9207: printInst(F("lat")); print(F("Z"));  return ZZ;
    case 0x920c: printInst(F("st"));  print(F("X"));  return XX;
    case 0x920d: printInst(F("st"));  print(F("X+")); return XX;
    case 0x920e: printInst(F("st"));  print(F("-X")); return XX;
    case 0x8208: printInst(F("st"));  print(F("Y"));  return YY;
    case 0x9209: printInst(F("st"));  print(F("Y+")); return YY;
    case 0x920a: printInst(F("st"));  print(F("-Y")); return YY;
    case 0x8200: printInst(F("st"));  print(F("Z"));  return ZZ;
    case 0x9201: printInst(F("st"));  print(F("Z+")); return ZZ;
    case 0x9202: printInst(F("st"));  print(F("-Z")); return ZZ;
  }
  return 0;
}

boolean dAsmXYZLoad (unsigned int opcode) {
  char src[4];
  switch (opcode & ~0x1F0) {
    case 0x9004: printInst(F("lpm"));  strcpy_P(src, (char*) F("Z"));  srSrc = ZZ; break;
    case 0x9005: printInst(F("lpm"));  strcpy_P(src, (char*) F("Z+")); srSrc = ZZ; break;
    case 0x9006: printInst(F("elpm")); strcpy_P(src, (char*) F("Z"));  srSrc = ZZ; break;
    case 0x9007: printInst(F("elpm")); strcpy_P(src, (char*) F("Z+")); srSrc = ZZ; break;
    case 0x900c: printInst(F("ld"));   strcpy_P(src, (char*) F("X"));  srSrc = XX; break;
    case 0x900d: printInst(F("ld"));   strcpy_P(src, (char*) F("X+")); srSrc = XX; break;
    case 0x900e: printInst(F("ld"));   strcpy_P(src, (char*) F("-X")); srSrc = XX; break;
    case 0x8008: printInst(F("ld"));   strcpy_P(src, (char*) F("Y"));  srSrc = YY; break;
    case 0x9009: printInst(F("ld"));   strcpy_P(src, (char*) F("Y+")); srSrc = YY; break;
    case 0x900a: printInst(F("ld"));   strcpy_P(src, (char*) F("-Y")); srSrc = YY; break;
    case 0x8000: printInst(F("ld"));   strcpy_P(src, (char*) F("Z"));  srSrc = ZZ; break;
    case 0x9001: printInst(F("ld"));   strcpy_P(src, (char*) F("Z+")); srSrc = ZZ; break;
    case 0x9002: printInst(F("ld"));   strcpy_P(src, (char*) F("-Z")); srSrc = ZZ; break;;
    default: return false;
  }
  printDstReg((opcode & 0x1F0) >> 4);
  print(",");
  print(src);
  return true;
}

boolean dAsmLddYZQ (unsigned int opcode) {
  char src[4];
  switch (opcode & ~0x2DF7) {
    case 0x8008: printInst(F("ldd"));  strcpy_P(src, (char*) F("Y+"));  srSrc = YY; break;
    case 0x8000: printInst(F("ldd"));  strcpy_P(src, (char*) F("Z+"));  srSrc = ZZ; break;
    default: return false;
  }
  byte qq = ((opcode & 0x2000) >> 8) + ((opcode & 0x0C00) >> 7) + (opcode & 0x07);
  printDstReg((opcode & 0x1F0) >> 4);
  print(",");
  print(src);
  printDec(qq);
  return true;
}

boolean dAsmStdYZQ (unsigned int opcode) {
  switch (opcode & ~0x2DF7) {
    case 0x8208: printInst(F("std"));  print(F("Y+"));  srSrc = YY; break;
    case 0x8200: printInst(F("std"));  print(F("Z+"));  srSrc = ZZ; break;
    default: return false;
  }
  byte qq = ((opcode & 0x2000) >> 8) + ((opcode & 0x0C00) >> 7) + (opcode & 0x07);
  printDec(qq);
  print(",");
  printDstReg((opcode & 0x1F0) >> 4);
  return true;
}
boolean dAsmRelCallJmp (unsigned int opcode) {
  switch (opcode & ~0xFFF) {
    case 0xd000: printInst(F("rcall")); return true;
    case 0xc000: printInst(F("rjmp"));  return true;
  }
  return false;
}

boolean dAsmBitOps2  (unsigned int opcode) {
  switch (opcode & ~0xFF) {
    case 0x9800: printInst(F("cbi"));   return true;
    case 0x9a00: printInst(F("sbi"));   return true;
    case 0x9900: printInst(F("sbic"));  return true;
    case 0x9b00: printInst(F("sbis"));  return true;
  }
  return false;
}

boolean dAsmByteImd (unsigned int opcode) {
  switch (opcode & ~0xFFF) {
    case 0x3000: printInst(F("cpi"));   return true;
    case 0x4000: printInst(F("sbci"));  return true;
    case 0x5000: printInst(F("subi"));  return true;
    case 0x6000: printInst(F("ori"));   return true;
    case 0x7000: printInst(F("andi"));  return true;
    case 0xE000: printInst(F("ldi"));   return true;
  }
  return false;
}

boolean dAsmArith (unsigned int opcode) {
  switch (opcode & ~0x3FF) {
    case 0x1c00: printInst(F("adc"));   return true;
    case 0x0c00: printInst(F("add"));   return true;
    case 0x2000: printInst(F("and"));   return true;
    case 0x1400: printInst(F("cp "));   return true;
    case 0x0400: printInst(F("cpc"));   return true;
    case 0x1000: printInst(F("cpse"));  return true;
    case 0x2400: printInst(F("eor"));   return true;
    case 0x2c00: printInst(F("mov"));   return true;
    case 0x9c00: printInst(F("mul"));   return true;
    case 0x2800: printInst(F("or"));    return true;
    case 0x0800: printInst(F("sbc"));   return true;
    case 0x1800: printInst(F("sub"));   return true;
  }
  return false;
}

boolean dAsmWordImd (unsigned int opcode) {
  switch (opcode & ~0xFF) {
    case 0x9600: printInst(F("adiw")); return true;
    case 0x9700: printInst(F("sbiw")); return true;
  }
  return false;
}

boolean dAsmBitOps (unsigned int opcode) {
  switch (opcode & ~0x1F7) {
    case 0xf800: printInst(F("bld"));   return true;
    case 0xfa00: printInst(F("bst"));   return true;
    case 0xfc00: printInst(F("sbrc"));  return true;
    case 0xfe00: printInst(F("sbrs"));  return true;
  }
  return false;
}

boolean dAsmBranch (unsigned int opcode) {
  switch (opcode & ~0x3F8) {
    case 0xf000: printInst(F("brcs")); return true;   // 1111 00kk kkkk k000
    case 0xf001: printInst(F("breq")); return true;   // 1111 00kk kkkk k001
    case 0xf002: printInst(F("brmi")); return true;   // 1111 00kk kkkk k010
    case 0xf003: printInst(F("brvs")); return true;   // 1111 00kk kkkk k011
    case 0xf004: printInst(F("brlt")); return true;   // 1111 00kk kkkk k100
    case 0xf005: printInst(F("brhs")); return true;   // 1111 00kk kkkk k101
    case 0xf006: printInst(F("brts")); return true;   // 1111 00kk kkkk k110
    case 0xf007: printInst(F("brie")); return true;   // 1111 00kk kkkk k111
    case 0xf400: printInst(F("brcc")); return true;   // 1111 01kk kkkk k000
    case 0xf401: printInst(F("brne")); return true;   // 1111 01kk kkkk k001
    case 0xf402: printInst(F("brpl")); return true;   // 1111 01kk kkkk k010
    case 0xf403: printInst(F("brvc")); return true;   // 1111 01kk kkkk k011
    case 0xf404: printInst(F("brge")); return true;   // 1111 01kk kkkk k100
    case 0xf405: printInst(F("brhc")); return true;   // 1111 01kk kkkk k101
    case 0xf406: printInst(F("brtc")); return true;   // 1111 01kk kkkk k110
    case 0xf407: printInst(F("brid")); return true;   // 1111 01kk kkkk k111
  }
  return false;
}

boolean dAsmMul (unsigned int opcode) {
  switch (opcode & ~0x077) {
    case 0x0300: printInst(F("mulsu"));   return true;
    case 0x0308: printInst(F("fmul"));    return true;
    case 0x0380: printInst(F("fmuls"));   return true;
    case 0x0388: printInst(F("fmulsu"));  return true;
  }
  return false;
}

//
//  In-System Programmer (adapted from ArduinoISP sketch by Randall Bohn)
//
//  My changes to Mr Bohn's original code include removing and renaming functions that were redundant with
//  my code, converting other functions to cases in a master switch() statement, removing code that served
//  no purpose, and making some structural changes to reduce the size of the code.
//  
//  Portions of the code after this point were originally Copyright (c) 2008-2011 Randall Bohn
//    If you require a license, see:  http://www.opensource.org/licenses/bsd-license.php
//

// STK Definitions
#define STK_OK      0x10    // DLE
#define STK_FAILED  0x11    // DC1
#define STK_UNKNOWN 0x12    // DC2
#define STK_INSYNC  0x14    // DC4
#define STK_NOSYNC  0x15    // NAK
#define CRC_EOP     0x20    // Ok, it is a space...

// Code definitions
#define EECHUNK   (32)

// Variables used by In-System Programmer code

uint16_t      pagesize;
uint16_t      eepromsize;
bool          rst_active_high;
int           pmode = 0;
unsigned int  here;           // Address for reading and writing, set by 'U' command
unsigned int  hMask;          // Pagesize mask for 'here" address

void programmer (void) {
  if (Serial.available()) {
    avrisp();
  }
}

void selectProgrammer () {
  pmode = 0;
  digitalWrite(VCC, HIGH);    // Enable VCC
  pinMode(VCC, OUTPUT);
  delay(100);
  debugWire.enable(false);
  disableSpiPins();          // Disable SPI pins
  Serial.begin(PROGRAM_BAUD);
}

uint8_t getch () {
  while (!Serial.available())
    ;
  return Serial.read();
}

void fill (int n) {
  for (int x = 0; x < n; x++) {
    buf[x] = getch();
  }
}

void empty_reply () {
  if (CRC_EOP == getch()) {
    Serial.write(STK_INSYNC);
    Serial.write(STK_OK);
  } else {
    Serial.write(STK_NOSYNC);
  }
}

void breply (uint8_t b) {
  if (CRC_EOP == getch()) {
    Serial.write(STK_INSYNC);
    Serial.write(b);
    Serial.write(STK_OK);
  } else {
    Serial.write(STK_NOSYNC);
  }
}

////////////////////////////////////
//      Command Dispatcher
////////////////////////////////////

void avrisp () {
  uint8_t ch = getch();
  switch (ch) {
    case 0x30:                                  // '0' - 0x30 Get Synchronization (Sign On)
      empty_reply();
      break;
      
    case 0x31:                                  // '1' - 0x31 Check if Starterkit Present
      if (getch() == CRC_EOP) {
        Serial.write(STK_INSYNC);
        Serial.print("AVR ISP");
        Serial.write(STK_OK);
      } else {
        Serial.write(STK_NOSYNC);
      }
      break;

    case 0x41:                                  // 'A' - 0x41 Get Parameter Value
      switch (getch()) {
        case 0x80:
          breply(0x02); // HWVER
          break;
        case 0x81:
          breply(0x01); // SWMAJ
          break;
        case 0x82:
          breply(0x12); // SWMIN
          break;
        case 0x93:
          breply('S');  // Serial programmer
          break;
        default:
          breply(0);
      }
      break;
      
    case 0x42:                                  // 'B' - 0x42 Set Device Programming Parameters
      fill(20);
      // AVR devices have active low reset, AT89Sx are active high
      rst_active_high = (buf[0] >= 0xE0);
      pagesize   = (buf[12] << 8) + buf[13];
      // Setup page mask for 'here' address variable
      if (pagesize == 32) {
        hMask = 0xFFFFFFF0;
      } else if (pagesize == 64) {
        hMask = 0xFFFFFFE0;
      } else if (pagesize == 128) {
        hMask = 0xFFFFFFC0;
      } else if (pagesize == 256) {
        hMask = 0xFFFFFF80;
      } else {
        hMask = 0xFFFFFFFF;
      }
      eepromsize = (buf[14] << 8) + buf[15];
      empty_reply();
      break;
      
    case 0x45:                                  // 'E' - 0x45 Set Extended Device Programming Parameters (ignored)
      fill(5);
      empty_reply();
      break;
      
    case 0x50:                                  // 'P' - 0x50 Enter Program Mode
      if (!pmode) {
        delay(100);
        // Reset target before driving SCK or MOSI
        digitalWrite(RESET, rst_active_high ? HIGH : LOW);  // Reset Enabled
        pinMode(RESET, OUTPUT);
        enableSpiPins();
        // See AVR datasheets, chapter "Serial_PRG Programming Algorithm":
        // Pulse RESET after SCK is low:
        digitalWrite(SCK, LOW);
        delay(20); // discharge SCK, value arbitrarily chosen
        digitalWrite(RESET, !rst_active_high ? HIGH : LOW); // Reset Disabled
        // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
        // speeds above 20 KHz
        delayMicroseconds(100);
        digitalWrite(RESET, rst_active_high ? HIGH : LOW);  // Reset Enabled
        // Send the enable programming command:
        delay(50); // datasheet: must be > 20 msec
        ispSend(0xAC, 0x53, 0x00, 0x00);
        pmode = 1;
      }
      empty_reply();
      break;
      
    case 0x51:                                  // 'Q' - 0x51 Leave Program Mode
      // We're about to take the target out of reset so configure SPI pins as input
      pinMode(MOSI, INPUT);
      pinMode(SCK, INPUT);
      digitalWrite(RESET, !rst_active_high ? HIGH : LOW); // Reset Disabled
      pinMode(RESET, INPUT);
      pmode = 0;
      empty_reply();
      break;

    case  0x55:                                 // 'U' - 0x55 Load Address (word)
      here = getch();
      here += 256 * getch();
      empty_reply();
      break;

    case 0x56:                                  // 'V' - 0x56 Universal Command
      fill(4);
      breply(ispSend(buf[0], buf[1], buf[2], buf[3]));
      break;

    case 0x60:                                  // '`' - 0x60 Program Flash Memory
      getch(); // low addr
      getch(); // high addr
      empty_reply();
      break;
      
    case 0x61:                                  // 'a' - 0x61 Program Data Memory
      getch(); // data
      empty_reply();
      break;

    case 0x64: {                                // 'd' - 0x64 Program Page (Flash or EEPROM)
      char result = STK_FAILED;
      unsigned int length = 256 * getch();
      length += getch();
      char memtype = getch();
      // flash memory @here, (length) bytes
      if (memtype == 'F') {
        fill(length);
        if (CRC_EOP == getch()) {
          Serial.write(STK_INSYNC);
          int ii = 0;
          unsigned int page = here & hMask;
          while (ii < length) {
            if (page != (here & hMask)) {
              ispSend(0x4C, (page >> 8) & 0xFF, page & 0xFF, 0);  // commit(page);
              page = here & hMask;
            }
            ispSend(0x40 + 8 * LOW, here >> 8 & 0xFF, here & 0xFF, buf[ii++]);
            ispSend(0x40 + 8 * HIGH, here >> 8 & 0xFF, here & 0xFF, buf[ii++]);
            here++;
          }
          ispSend(0x4C, (page >> 8) & 0xFF, page & 0xFF, 0);      // commit(page);
          Serial.write(STK_OK);
          break;
        } else {
          Serial.write(STK_NOSYNC);
        }
        break;
      } else if (memtype == 'E') {
        // here is a word address, get the byte address
        unsigned int start = here * 2;
        unsigned int remaining = length;
        if (length > eepromsize) {
          result = STK_FAILED;
        } else {
          while (remaining > EECHUNK) {
            // write (length) bytes, (start) is a byte address
            // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
            fill(length);
            for (unsigned int ii = 0; ii < EECHUNK; ii++) {
              unsigned int addr = start + ii;
              ispSend(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buf[ii]);
              delay(45);
            }
            start += EECHUNK;
            remaining -= EECHUNK;
          }
          // write (length) bytes, (start) is a byte address
          // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
          fill(length);
          for (unsigned int ii = 0; ii < remaining; ii++) {
            unsigned int addr = start + ii;
            ispSend(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buf[ii]);
            delay(45);
          }
          result = STK_OK;
        }
        if (CRC_EOP == getch()) {
          Serial.write(STK_INSYNC);
          Serial.write(result);
        } else {
          Serial.write(STK_NOSYNC);
        }
        break;
      }
      Serial.write(STK_FAILED);
    } break;

    case 0x74: {                                // 't' - 0x74 Read Page (Flash or EEPROM)
      char result = STK_FAILED;
      int length = 256 * getch();
      length += getch();
      char memtype = getch();
      if (CRC_EOP != getch()) {
        Serial.write(STK_NOSYNC);
        break;
      }
      Serial.write(STK_INSYNC);
      if (memtype == 'F') {
        for (int ii = 0; ii < length; ii += 2) {
          Serial.write(ispSend(0x20 + LOW * 8, (here >> 8) & 0xFF, here & 0xFF, 0));   // low
          Serial.write(ispSend(0x20 + HIGH * 8, (here >> 8) & 0xFF, here & 0xFF, 0));  // high
          here++;
        }
        result = STK_OK;
      } else if (memtype == 'E') {
        // here again we have a word address
        int start = here * 2;
        for (int ii = 0; ii < length; ii++) {
          int addr = start + ii;
          Serial.write(ispSend(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF));
        }
        result = STK_OK;
      }
      Serial.write(result);
    } break;

    case 0x75:                                  // 'u' - 0x75 Read Signature Bytes
      if (CRC_EOP != getch()) {
        Serial.write(STK_NOSYNC);
        break;
      }
      Serial.write(STK_INSYNC);
      Serial.write(ispSend(0x30, 0x00, 0x00, 0x00)); // high
      Serial.write(ispSend(0x30, 0x00, 0x01, 0x00)); // middle
      Serial.write(ispSend(0x30, 0x00, 0x02, 0x00)); // low
      Serial.write(STK_OK);
      break;

    case 0x20:                                  // ' ' - 0x20 CRC_EOP
      // expecting a command, not CRC_EOP, this is how we can get back in sync
      Serial.write(STK_NOSYNC);
      break;

    default:                                    // anything else we will return STK_UNKNOWN
      if (CRC_EOP == getch()) {
        Serial.write(STK_UNKNOWN);
      } else {
        Serial.write(STK_NOSYNC);
      }
  }
}

/*
  AVR opcodes (taken from a post to avrfreaks by Jeremy Brandon)
  
  Most are single 16 bit words; four marked * have a second word
  to define an address or address extension (kkkk kkkk kkkk kkkk)
  
    d  bits that specify an Rd (0..31 or 16..31 or 16..23 or 24..30)
    r  bits that specify an Rr ( - ditto - )
    k  bits that specify a constant or an address
    q  bits that specify an offset
    -  bit that specifies pre-decrement mode: 0=no, 1=yes
    +  bit that specifies post-decrement mode: 0=no, 1=yes
    x  bit of any value
    s  bits that specify a status bit (0..7)
    A  bits that specify i/o memory
    b  bits that define a bit (0..7)
  
    0000 0000 0000 0000  nop
    0000 0001 dddd rrrr  movw
    0000 0010 dddd rrrr  muls
    0000 0000 0ddd 0rrr  mulsu
    0000 0000 0ddd 1rrr  fmul
    0000 0000 1ddd 0rrr  fmuls
    0000 0000 1ddd 1rrr  fmulsu
    0000 01rd dddd rrrr  cpc
    0000 10rd dddd rrrr  sbc
    0000 11rd dddd rrrr  add
    0001 00rd dddd rrrr  cpse
    0001 01rd dddd rrrr  cp
    0001 10rd dddd rrrr  sub
    0001 11rd dddd rrrr  adc
    0001 00rd dddd rrrr  and
    0001 01rd dddd rrrr  eor
    0001 10rd dddd rrrr  or
    0001 11rd dddd rrrr  mov
    0011 kkkk dddd kkkk  cpi
    0100 kkkk dddd kkkk  sbci
    0101 kkkk dddd kkkk  subi
    0110 kkkk dddd kkkk  ori
    0111 kkkk dddd kkkk  andi
    1000 000d dddd 0000  ld Z
    1000 000d dddd 1000  ld Y
    10q0 qq0d dddd 0qqq  ldd Z
    10q0 qq0d dddd 1qqq  ldd Y
    10q0 qq1d dddd 0qqq  std Z
    10q0 qq1d dddd 1qqq  std Y
    1001 000d dddd 0000  lds *
    1001 000d dddd 00-+  ld –Z+
    1001 000d dddd 010+  lpm Z
    1001 000d dddd 011+  elpm Z
    1001 000d dddd 10-+  ld –Y+
    1001 000d dddd 11-+  ld X
    1001 000d dddd 1111  pop
    1001 001r rrrr 0000  sts *
    1001 001r rrrr 00-+  st –Z+
    1001 001r rrrr 01xx  ???
    1001 001r rrrr 10-+  st –Y+
    1001 001r rrrr 11-+  st X
    1001 001d dddd 1111  push
    1001 010d dddd 0000  com
    1001 010d dddd 0001  neg
    1001 010d dddd 0010  swap
    1001 010d dddd 0011  inc
    1001 010d dddd 0100  ???
    1001 010d dddd 0101  asr
    1001 010d dddd 0110  lsr
    1001 010d dddd 0111  ror
    1001 010d dddd 1010  dec
    1001 0100 0sss 1000  bset
    1001 0100 1sss 1000  bclr
    1001 0100 0000 1001  ijmp
    1001 0100 0001 1001  eijmp
    1001 0101 0000 1000  ret
    1001 0101 0000 1001  icall
    1001 0101 0001 1000  reti
    1001 0101 0001 1001  eicall
    1001 0101 1000 1000  sleep
    1001 0101 1001 1000  break
    1001 0101 1010 1000  wdr
    1001 0101 1100 1000  lpm
    1001 0101 1101 1000  elpm
    1001 0101 1110 1000  spm
    1001 0101 1111 1000  espm
    1001 010k kkkk 110k  jmp *
    1001 010k kkkk 111k  call *
    1001 0110 kkdd kkkk  adiw
    1001 0111 kkdd kkkk  sbiw
    1001 1000 aaaa abbb  cbi
    1001 1001 aaaa abbb  sbic
    1001 1010 aaaa abbb  sbi
    1001 1011 aaaa abbb  sbis
    1001 11rd dddd rrrr  mul
    1011 0aad dddd aaaa  in
    1011 1aar rrrr aaaa  out
    1100 kkkk kkkk kkkk  rjmp
    1101 kkkk kkkk kkkk  rcall
    1110 kkkk dddd kkkk  ldi
    1111 00kk kkkk ksss  brbs
    1111 01kk kkkk ksss  brbc
    1111 100d dddd 0bbb  bld
    1111 101d dddd 0bbb  bst
    1111 110r rrrr 0bbb  sbrc
    1111 111r rrrr 0bbb  sbrs
*/
