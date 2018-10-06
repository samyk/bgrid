#include <RF24.h>
#include "Adafruit_NeoPixel.h"
//#include "WS2812.h"
//#include "light_ws2812.c"

// avrdude: safemode: Fuses OK (E:FF, H:DF, L:62) [was]
// avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2) [now]

/*
CSN goes LOW during SPI transfer
CE is always HIGH

WIRING:
- D7 (CE) -> 10/PA3
- D8 (CSN) -> 11/PA2
- MOSI -> 8/PA5
- SCK -> 9/PA4
- MISO ? -> 7/PA6
- WS2812 -> 2/PB0

//                           +-\/-+
//                     VCC  1|    |14  GND
//             (D  0)  PB0  2|    |13  PA0  (D 10)        AREF
//             (D  1)  PB1  3|    |12  PA1  (D  9)
//             (D 11)  PB3  4|    |11  PA2  (D  8)
//  PWM  INT0  (D  2)  PB2  5|    |10  PA3  (D  7)
//  PWM        (D  3)  PA7  6|    |9   PA4  (D  6)
//  PWM        (D  4)  PA6  7|    |8   PA5  (D  5)        PWM

ISSUE investigation:
 - nrf powered from 3.3 bench supply w/cap
 - mosi + sck + csn/ce + miso must be working as registers can be read
 - transmitter works as secondary nrf works
 - chip is fine as i swapped it out with working one
 - status register normally looks good (0x17FF -> 0x0E11)
 - CE good
 - MOSI good
 - MISO good
 - SCK good
 - WS good
 - CSN good

DIFFERENCES:
 - teensy 10MHz, attiny 1MHz SPI
 - teensy 3.3v, attiny was 5v now 3.3v, still issues


TODO:
 - add unique id
 */

#define WS_PIN 0 // 1 maps to PB0 - not sure why
#define LEDS 24

#if defined WS2812_H_
WS2812 LED(LEDS);
#elif defined LIGHT_WS2812_H_
// XXX pin defined in ws2812_config.h
struct cRGB rgb[LEDS];
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);
#endif

#define dbg(str) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)

RF24 radio(7, 8); // CE, CSN

//byte addresses[][6] = {"1Node","2Node"};
byte address[] = "2Node";

unsigned long last = 0;
uint32_t serial;

void setup()
{
  delay(1000);
  dbgln("starting");

  setupLEDs();
  setupRadio();
  testLEDs();
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
  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  radio.begin();

  //radio.setPALevel(RF24_PA_HIGH);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS); // RF24_2MBPS
  radio.setChannel(125); // XXX

  //radio.openWritingPipe(address+1);
  //radio.openReadingPipe(1,addresses[1]);
  radio.openReadingPipe(1, address);

  radio.startListening();
  //radio.printDetails();
}

void setupLEDs()
{
#if defined WS2812_H_
  LED.setOutput(WS_PIN);
#elif defined LIGHT_WS2812_H_
  DDRB |= _BV(WS_PIN); // _BV(ws2812_pin);
#else
  strip.begin();
  strip.show();
#endif
}

void loop()
{
// XXX readd once RF working
//  if (millis() - last > 1000)
//    setColor(0);

#define RXBYTES 128
  byte buf[RXBYTES];

  //delay(200);
  if (radio.available())
  {
  setColor(0x00ff00);
  delay(100);
  setColor(0x000000);
    while (radio.available())
    {
  setColor(0xff0000);
  delay(100);
  setColor(0x000000);
      radio.read(&buf, RXBYTES); // Get the payload
    }

    last = millis();

    // all balloons should turn this color
    if (buf[0] == '*')
    {
#define colorFromBuf(x) unsigned long color = ((unsigned long)buf[x] << 16) | ((unsigned long)buf[x+1] << 8) | (buf[x+2]);
      //unsigned long color = ((unsigned long)buf[1] << 16) | ((unsigned long)buf[2] << 8) | (buf[3]);
      colorFromBuf(1);
      setColor(color);
    }

    // some number of balloons turn this color if uniqid % buf[1] == buf[2]
    else if (buf[0] == 'U')
    {
      if ((serial % buf[1]) == buf[2])
      {
        //unsigned long color = ((unsigned long)buf[3] << 16) | ((unsigned long)buf[4] << 8) | (buf[5]);
        colorFromBuf(3);
        setColor(color);
      }
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
