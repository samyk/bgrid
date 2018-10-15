/*
TODO:
 - add unique id
 - figure out why ws2812s not outputting
 */

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define TINY
#endif

#include <RF24.h>
#include <EEPROM.h>
#ifdef CORE_TEENSY
#include "Adafruit_NeoPixel.h"
#else
#include "light_ws2812.c"
#include <avr/wdt.h>
#endif
//#include "WS2812.h"

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

//#define WS_PIN 1 // 1 maps to PB0 - not sure why
#define WS_PIN 1 // PIN_B0
#define LEDS 24 // 24 // XXX need to actually support 24, limited by nrf rx

//#define RXBYTES (LEDS*3 + 1 > 32 ? 32 : LEDS*3 + 1)// 32 is MAX nRF can tx/rx
#define RXBYTES 32
byte buf[RXBYTES];

// XXX make LEDs ramp to all 24 or whatever

#if defined WS2812_H_
WS2812 LED(LEDS);
#elif defined LIGHT_WS2812_H_
// XXX pin defined in ws2812_config.h
struct cRGB rgb[LEDS];
//struct cRGB *rgb;
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);
#endif

#ifdef CORE_TEENSY
#define dbg(str) Serial.print(str)
#define dbgh(str, type) Serial.print(str, type)
#define dbgln(str) Serial.println(str)
#define out(x) Serial.println(x)
#else
#define dbg(str) void()
#define dbgh(str, type) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)
#endif

RF24 radio(7, 8); // CE, CSN

//byte addresses[][6] = {"1Node","2Node"};
byte address[] = "2Node";

unsigned long last = 0;
uint8_t serial;

void setup()
{
  wdt_enable(WDTO_8S);

#ifdef CORE_TEENSY
  Serial.begin(115200);
  dbgln("starting");
#endif

  setupLEDs();
  testLEDs();
  setupRadio();
  getEEPROM();

  // don't use loop() to avoid serial stuff
  while (1) rx();
}

// EEPROM (1 byte each):
// [magic B0] [version 00] [serial ID]
#define EEPROM_MAGIC_ADDR   0x00
#define EEPROM_VERSION_ADDR 0x01
#define EEPROM_SERIAL_ADDR  0x02
#define EEPROM_MAGIC 0xB0
void getEEPROM()
{
  serial = 0;

  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC)
    serial = EEPROM.read(EEPROM_SERIAL_ADDR);
}

void testLEDs()
{
  setColor(0xff0000);
  delay(300);
  setColor(0x00ff00);
  delay(300);
  setColor(0x0000ff);
  delay(300);
  setColor(0x000000);
}

void setupRadio()
{
  radio.begin();

  //radio.disableCRC()
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(false); // XXX WTF - WANT THIS OFF!! but doesn't seem to work
  radio.setAddressWidth(5);
  radio.setDataRate(RF24_250KBPS); // RF24_2MBPS
  radio.setChannel(125); // XXX

  radio.openReadingPipe(1, (uint8_t*)(&address));

  radio.startListening();
  //radio.printDetails();
}

void setupLEDs()
{
#if defined WS2812_H_
  LED.setOutput(WS_PIN);
#elif defined LIGHT_WS2812_H_
  //DDRB |= _BV(WS_PIN); // _BV(ws2812_pin);
  pinMode(WS_PIN, OUTPUT);
#else
  strip.begin();
  strip.show();
#endif
}

#define setColorFromBuf(x) do { \
  unsigned long color = ((unsigned long)buf[x] << 16) | ((unsigned long)buf[x+1] << 8) | (buf[x+2]); \
  setColor(color); \
} while(0)

void loop() { }
void rx()
{
  if (millis() - last > 1000)
    setColor(0);

  if (radio.available())
  {
    // get payload (how many bytes do we get?)
    while (radio.available())
    {
      radio.read(&buf, RXBYTES);

      wdt_reset(); // let WDT know we're good

      for (int i = 0; i < RXBYTES; i++)
        dbgh(buf[i], HEX);
      dbgln("");
    }


    last = millis();

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
      if (serial >= buf[1] && serial < buf[1] + buf[2])
        setColorFromBuf(3 + 3 * (serial - buf[1])); // 3 byte header + skip RGBs not relevant
    }
  }
}


void setColor(unsigned long color)
{
  for (int i = 0; i < LEDS; i++)
    setPixelColor(i, color);
  show();
}

void show()
{
#if defined WS2812_H_
  LED.sync();
#elif defined LIGHT_WS2812_H_
  //ws2812_sendarray((uint8_t *)rgb, LEDS*3);
  ws2812_setleds_pin(rgb, LEDS, WS_PIN);
#else
  strip.show();
#endif
}

void setPixelColor(int i, unsigned long color)
{
#if defined WS2812_H_
  cRGB val;
  val.r = (color >> 16) & 0xFF;
  val.g = (color >>  8) & 0xFF;
  val.b = (color >>  0) & 0xFF;
  LED.set_crgb_at(i, val);
#elif defined LIGHT_WS2812_H_
  rgb[i].r = (color >> 16) & 0xFF;
  rgb[i].g = (color >>  8) & 0xFF;
  rgb[i].b = (color >>  0) & 0xFF;
#else
  strip.setPixelColor(i, color);
#endif
}

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

