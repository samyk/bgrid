/*

write flash (erases eeprom!):
  avrdude -P/dev/tty.usbmodemFB0001 -b19200 -v -p attiny84 -C /Users/samy/.platformio/packages/tool-avrdude/avrdude.conf -c stk500v1 -b 19200 -e -D -U flash:w:.pioenvs/attiny84/firmware.hex:i

read eeprom:
  avrdude -P/dev/tty.usbmodemFB0001 -b19200 -v -p attiny84 -C /Users/samy/.platformio/packages/tool-avrdude/avrdude.conf -c stk500v1 -b 19200 -D -U eeprom:r:eeprom.bin:r

write eeprom:
  avrdude -P/dev/tty.usbmodemFB0001 -b19200 -v -p attiny84 -C /Users/samy/.platformio/packages/tool-avrdude/avrdude.conf -c stk500v1 -b 19200 -D -U eeprom:w:testeeprom.bin:r

*/

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define TINY
#endif

//#define SINGLE

#ifdef SINGLE
crap
#include <RF24Tiny.h>
#else
#include <RF24.h>
#endif
#include <EEPROM.h>

#ifdef CORE_TEENSY
#include "Adafruit_NeoPixel.h"
#else
#include "light_ws2812.c"
#include <avr/wdt.h>
#endif

#ifdef SERIAL_ENABLE
#define _SS_MAX_RX_BUFF 2 // RX buffer size
#include <SoftwareSerial.h>
SoftwareSerial Ser(RX, 10);
#endif

//#include "WS2812.h"

//#define sp(x) Ser.write(x)
#define sp(x)


//#define WS_PIN 1 // 1 maps to PB0 - not sure why
#define WS_PIN 1 // PIN_B0
#define LEDS 32 // 32 -- TODO, get/set from EEPROM

#define MAX_BYTES 32
#define RGB_SIZE 3
byte buf[MAX_BYTES];
int i; // reusable int


/*
attiny84, no single
DATA:    [=====     ]  52.7% (used 270 bytes from 512 bytes)
PROGRAM: [======    ]  56.6% (used 4640 bytes from 8192 bytes)

attiny44, no single
DATA:    [==========]  105.5% (used 270 bytes from 256 bytes)
PROGRAM: [==========]  113.3% (used 4640 bytes from 4096 bytes)
Error: The program size (4640 bytes) is greater than maximum allowed (4096 bytes)

attiny24, SINGLE
DATA:    [======    ]  60.2% (used 77 bytes from 128 bytes)
PROGRAM: [==========]  198.7% (used 4070 bytes from 2048 bytes)
*/

#if defined WS2812_H_
WS2812 LED(LEDS);
#elif defined LIGHT_WS2812_H_
// XXX pin defined in ws2812_config.h
#ifdef SINGLE
uint32_t onergb;
#else
struct cRGB rgb[LEDS];
#endif
//struct cRGB *rgb;
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);
#endif

// EEPROM (1 byte each):
// [magic B0] [version 00] [serial ID] [flags]
// flags (8 bits): [test LEDs]
#define EEPROM_MAGIC_ADDR   0x00
#define EEPROM_VERSION_ADDR 0x01
// rewritable
#define EEPROM_WRITABLE     0x10 // can write from here up
#define EEPROM_SERIAL_ADDR  0x10
#define EEPROM_FLAGS_ADDR   0x12
#define EEPROM_FLAGS_TESTLEDS_BIT 7
#define EEPROM_MAGIC 0xB0

#ifdef CORE_TEENSY
#define wdt_reset()
#define wdt_enable()
#define pf(...) printf(__VA_ARGS__)
#define dbg(str) Serial.print(str)
#define dbgh(str, type) Serial.print(str, type)
#define dbgln(str) Serial.println(str)
#define out(x) Serial.println(x)
#else
#define pf(...)
#define dbg(str) void()
#define dbgh(str, type) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)
#endif

#define NRF_CE 7
#define NRF_CSN 8
#ifdef SINGLE
RF24Tiny radio; // CE, CSN
#else
RF24 radio(NRF_CE, NRF_CSN); // CE, CSN
#endif

byte address[] = "2Node";

unsigned long last = 0;
byte eeprom_set = 0;
uint8_t serial = 0;

#define RX 3
#define TX 10
#define DBG_PIN TX

#ifdef SINGLE
#define setColor(tcolor) do { onergb = tcolor; show(); } while(0)
#else
#define setColor(tcolor) setColorReal(tcolor)
#endif
#define setColorFromBuf(x) do { \
  last = millis(); \
  setColor(((unsigned long)buf[x] << 16) | ((unsigned long)buf[x+1] << 8) | (buf[x+2])); \
} while(0)

void setup()
{
  byte mcusr = 0;

#ifdef TINY
  mcusr = MCUSR;
  MCUSR = 0;

  // MUST BE THE FIRST THING IN SETUP...works after capturing MCUSR though
  wdt_enable(WDTO_8S);
#endif

  // pin D 10 -> PA0 (bit=_BV(0), port=PA(1), reg=DDRA, out=PORTA)
  //pinMode(DBG_PIN, OUTPUT);

  getEEPROM();
  wdt_reset(); // let WDT know we're good

  setupLEDs();
  wdt_reset(); // let WDT know we're good

  // if we were NOT reset by watchdog timer
  if (!(mcusr & _BV(WDRF)))
  {
    // eeprom not set
    if (!eeprom_set)
      testLEDs(2);
    // "turn on leds"
    else if (EEPROM.read(EEPROM_FLAGS_ADDR) & (1 << EEPROM_FLAGS_TESTLEDS_BIT))
      testLEDs(1);
    else
      testLEDs(4);
  }
  // reset by watchdog, do NOTHING!
  else { }

  wdt_reset(); // let WDT know we're good

  dbgln("pre");
  setupRadio();
  dbgln("post");
  wdt_reset(); // let WDT know we're good

  // don't use loop() to avoid serial stuff
  while (1) rx();
}

void getEEPROM()
{
  serial = 0;

  // do we have valid eeprom?
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC)
    eeprom_set = 1;

  // get 2 byte "unique" ID even if eeprom not set, this way 65535 is our ID for unset ones
  //if (eeprom_set)
  serial = (EEPROM.read(EEPROM_SERIAL_ADDR) << 8) | EEPROM.read(EEPROM_SERIAL_ADDR+1);
  serial &= 0xFF; // oops, i think we only support 8 bits elsewhere
}

void testLEDs(byte nums)
{
  uint32_t color = 0xFFFFFF;
  for (i = 0; i < nums; i++)
  {
    //setColor(color);
    if (i == 0)
      setColor(0x0F0000);
    if (i == 1)
      setColor(0x000F00);
    if (i == 2)
      setColor(0x00000F);
    if (i == 3)
      setColor(0x0F0F0F);
    delay(200);
    color >>= 4;
  }
  setColor(0x000000);
}

void setupRadio()
{
  dbgln("a1");
  //radio.powerDown();
  dbgln("a2");
  delay(500);

  dbgln("a2.0");
  radio.begin();
  dbgln("a3");

  //radio.disableCRC()
  //radio.setCRCLength(RF24_CRC_16); // XXX haven't tested
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(false);
  radio.setAddressWidth(5);
  radio.setDataRate(RF24_250KBPS); // RF24_2MBPS
  radio.setChannel(125);

  dbgln("a40");
  radio.openReadingPipe(1, (uint8_t*)(&address)); // XXX

#define MAX_RGB_PER_PACKET 9
  if (serial == 0xFF) serial = 0;
  address[0] = serial / MAX_RGB_PER_PACKET;
//  radio.openReadingPipe(1, (uint8_t*)(&address)); // XXX
  radio.openReadingPipe(2, (uint8_t*)(&address)); // XXX
  dbgln("a41");

  radio.startListening();
  dbgln("a42");
  //radio.printDetails();
}

void setupLEDs()
{
#if defined WS2812_H_
  LED.setOutput(WS_PIN);
#elif defined LIGHT_WS2812_H_
  //DDRB |= _BV(WS_PIN); // _BV(ws2812_pin);
  //pinMode(WS_PIN, OUTPUT); // i think this happens in the code itself
#else
  strip.begin();
  strip.show();
#endif
}

void rx()
{
  // TODO fadeout (but cancel if interrupt)
  // TODO validate interrupts work
  if (millis() - last > 3000)
  {
    setColor(0);
    last = millis();
  }

  while (radio.available())
  {
    radio.read(&buf, MAX_BYTES);

    // valid packets?
    if (buf[0] == '*' || buf[0] == 'U' || buf[0] == 'S' || buf[0] == '_')
      handle_packet(); // handle packet, let WDT know we're good

    // clear buf
    memset(buf, 0, sizeof buf);
  }

}

// reset our watchdog
void handle_packet()
{
  wdt_reset(); // let WDT know we're good

  // all balloons should turn this color
  if (buf[0] == '*')
    setColorFromBuf(1);

  // some number of balloons turn this color if (serial % buf[1] == buf[2])
  else if (buf[0] == 'U')
  {
    if ((serial % buf[1]) == buf[2])
      setColorFromBuf(3);
  }

  // get bytes based off of "serial" data
  // 'S' [starting ID] [# of IDs] ([RGB ...] for # of IDs)
  else if (buf[0] == 'S')
  {
    // make sure not out of bounds
    if (3 + 3 * buf[2] + 2 > MAX_BYTES)
      return;

    // is our serial number in here?
    if (serial >= buf[1] && serial < buf[1] + buf[2])
    {
      // make sure not out of bounds
      if (3 + 3 * (serial - buf[1]) + 2 > MAX_BYTES)
        return;

      setColorFromBuf(3 + 3 * (serial - buf[1])); // 3 byte header + skip RGBs not relevant
    }
    else if (buf[1] == 0xFF) // all
    {
      setColorFromBuf(3);
    }
  }

  // set EEPROM bytes/bits without affecting neighboring bits
  // '_' [ID to affect (2 bytes)] [# of settings (1 byte)] [bytes=1, bits=0 (1 bit), reset (1 bit), reserved (6 bits)]
  // repeating (for # of flags): [addr (2 bytes)] [byte / bit (3 bits), on/off (1 bit), reserved (4 bits)]
  // TODO transmit flags
//#define DISABLE_EEPROM_WRITES
#ifndef DISABLE_EEPROM_WRITES
  else if (buf[0] == '_')
  {
#define _ID 1
#define _NUM 3
#define _OPTS 4
#define _ADDR 5
#define _DATA 7

#define _DATA_LEN 3

#define _OPTS_BYTE 7
#define _OPTS_RESET 6

    // make sure we're working on right id
    uint16_t id = (buf[_ID] << 8) | buf[_ID+1];
    id &= 0xFF;
    if (id != 0 && id != serial)
      return;

    byte numflags = buf[_NUM];
    byte bytes = (buf[_OPTS] >> _OPTS_BYTE)  & 1;
    byte reset = (buf[_OPTS] >> _OPTS_RESET) & 1;

    // make sure not too much data
    if (5 + numflags * _DATA_LEN > MAX_BYTES)
      return;

    // go through each addr we want to update
    for (i = 0; i < numflags; i++)
    {
      uint16_t addr = (buf[_ADDR + i * _DATA_LEN] << 8) | buf[_ADDR+1 + i * _DATA_LEN];
      if (addr < EEPROM_WRITABLE)
        return;

      byte curbits = EEPROM.read(addr);
      // handle writing a byte
      if (bytes)
      {
        // write only if changed
        if (curbits != buf[_DATA + i * _DATA_LEN])
          EEPROM.write(addr, buf[_DATA + i * _DATA_LEN]);
      }

      // write single bit
      else
      {
        byte bitmask = 1 << (buf[_DATA + i * _DATA_LEN] >> 5);
        byte onoff = (buf[_DATA + i * _DATA_LEN] >> 4) & 1;

        // different value, let's change it
        if (((curbits & bitmask) > 0) != onoff)
        {
          // turn on
          if (onoff)
            curbits |= (1 << bitmask);
          // turn off
          else
            curbits &= ~(1 << bitmask);
          EEPROM.write(addr, curbits);
        }
      }
    }

    if (reset)
      softReset();

  } // end of _
#endif
}


void softReset()
{
#ifdef TINY
  asm volatile ("rjmp 0");
#endif
}

// set all LEDs to a single color
void setColorReal(unsigned long color)
{
//digitalWrite(DBG_PIN, HIGH);
  for (i = 0; i < LEDS; i++)
    setPixelColor(i, color);
  show();
//digitalWrite(DBG_PIN, LOW);
}

void show()
{
#if defined WS2812_H_
  LED.sync();
#elif defined LIGHT_WS2812_H_
  //ws2812_sendarray((uint8_t *)rgb, LEDS*3);
  #ifdef SINGLE
 //ws2812_setsingleleds_pin(onergb, LEDS, WS_PIN);
  ws2812_sendsingle_mask(onergb, LEDS*3, WS_PIN);
  _delay_us(ws2812_resettime);
  #else
  ws2812_setleds_pin(rgb, LEDS, WS_PIN);
  #endif
#else
  strip.show();
#endif
}

void setPixelColor(int ind, unsigned long color)
{
#if defined WS2812_H_
  cRGB val;
  val.r = (color >> 16) & 0xFF;
  val.g = (color >>  8) & 0xFF;
  val.b = (color >>  0) & 0xFF;
  LED.set_crgb_at(ind, val);
#elif defined LIGHT_WS2812_H_
  #ifdef SINGLE
  onergb = color;
  #else
  // can we just assign color directly?
  rgb[ind].r = (color >> 16) & 0xFF;
  rgb[ind].g = (color >>  8) & 0xFF;
  rgb[ind].b = (color >>  0) & 0xFF;
  #endif
#else
  strip.setPixelColor(ind, color);
#endif
}

void loop() { }

/*
avr-g++ -o "/Users/samy/Code/balloon/mcu-code/84a2/84a2.ino.cpp" -x c++ -fpreprocessed -dD -E "/var/folders/70/8z3l03311gg8yxfmjb6dk_yc0000gn/T/tmpf6748L"
avr-g++ -o .pioenvs/attiny44/src/84a2.ino.cpp.o -c -fno-exceptions -fno-threadsafe-statics -fpermissive -std=gnu++11 -Os -Wall -ffunction-sections -fdata-sections -flto -mmcu=attiny44 -DPLATFORMIO=30600 -DARDUINO_AVR_ATTINYX4 -DF_CPU=8000000L -DARDUINO_ARCH_AVR -DARDUINO=10805 -I. -I/Users/samy/.platformio/packages/framework-arduinoavr/libraries/__cores__/tiny/SPI -I/Users/samy/Code/arduino/libraries/RF24 -I/Users/samy/Code/arduino/libraries/RF24/utility -I/Users/samy/.platformio/packages/framework-arduinoavr/cores/tiny -I/Users/samy/.platformio/packages/framework-arduinoavr/variants/tinyX4 84a2.ino.cpp
avr-g++ -o .pioenvs/attiny44/firmware.elf -Os -mmcu=attiny44 -Wl,--gc-sections -flto -fuse-linker-plugin .pioenvs/attiny44/src/84a2.ino.cpp.o .pioenvs/attiny44/src/light_ws2812.c.o -L.pioenvs/attiny44 -Wl,--start-group .pioenvs/attiny44/liba68/libSPI.a .pioenvs/attiny44/lib74a/libRF24.a .pioenvs/attiny44/libFrameworkArduinoVariant.a .pioenvs/attiny44/libFrameworkArduino.a -lm -Wl,--end-group
MethodWrapper(["checkprogsize"], [".pioenvs/attiny44/firmware.elf"])
avr-objcopy -O ihex -R .eeprom .pioenvs/attiny44/firmware.elf .pioenvs/attiny44/firmware.hex
*/

/*
assembly output
avr-g++ -o blah.cpp -x c++ -fpreprocessed -dD -E 84a2.ino
avr-g++ -S -o test2.s -c -fno-exceptions -fno-threadsafe-statics -fpermissive  -Os -Wall -ffunction-sections -fdata-sections  -mmcu=attiny44 -DPLATFORMIO=30600 -DARDUINO_AVR_ATTINYX4 -DF_CPU=8000000L -DARDUINO_ARCH_AVR -DARDUINO=10805 -I. -I/Users/samy/.platformio/packages/framework-arduinoavr/libraries/__cores__/tiny/SPI -I/Users/samy/Code/arduino/libraries/RF24 -I/Users/samy/Code/arduino/libraries/RF24/utility -I/Users/samy/.platformio/packages/framework-arduinoavr/cores/tiny -I/Users/samy/.platformio/packages/framework-arduinoavr/variants/tinyX4 blah.cpp
*/

// SPI is defined in USICR ifdef in ~/.platformio/packages/framework-arduinoavr/libraries/__cores__/tiny/SPI/SPI.cpp

// pinout /Users/samy/.platformio//packages/framework-arduinoavr/variants/tinyX4/pins_arduino.h

// avrdude: safemode: Fuses OK (E:FF, H:DF, L:62) [was]
// avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2) [now]

/*
CSN goes LOW during SPI transfer, CE is always HIGH

WIRING: (note miso/mosi are swapped on mcu)
- D7 (CE) -> 10/PA3
- D8 (CSN) -> 11/PA2
- MOSI -> 8/PA5
- SCK -> 9/PA4
- MISO ? -> 7/PA6
- WS2812 -> 2/PB0
- IRQ -> 5/PB2

//                           +-\/-+
//                     VCC  1|    |14  GND
//  ws         (D  0)  PB0  2|    |13  PA0  (D 10)        AREF
//             (D  1)  PB1  3|    |12  PA1  (D  9)
//  reset      (D 11)  PB3  4|    |11  PA2  (D  8)
//  PWM  INT0  (D  2)  PB2  5|    |10  PA3  (D  7)
//  PWM        (D  3)  PA7  6|    |9   PA4  (D  6)  clk
//  PWM  mosi  (D  4)  PA6  7|    |8   PA5  (D  5)  miso  PWM

ISSUE investigation:
 - issue was address was being treated improperly, also be careful with rgb mem
 */
