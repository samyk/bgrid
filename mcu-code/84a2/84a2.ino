#include <RF24.h>
//#include "WS2812.h"
#include "light_ws2812.c"
//#include "Adafruit_NeoPixel.h"

// SPI is defined in USICR ifdef in ~/.platformio/packages/framework-arduinoavr/libraries/__cores__/tiny/SPI/SPI.cpp

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
 - teensy 10MHz, attiny 1MHz SPI - changing teensy SPI to < 1MHz doesn't fix
 - teensy 3.3v, attiny 5v - using 3.3V on attiny doesn't fix
 - 5us instead of 


TODO:
 - add unique id
 */

#define WS_PIN 1 // 1 maps to PB0 - not sure why
#define LEDS 24 // 24 // XXX need to actually support 24, limited by nrf rx

/*
#define DOUBLE // if defined, half LED space is used and LEDs are doubled up
#ifdef DOUBLE
#define RXBYTES ((LEDS/2) * 3 + 1)
#else
#define RXBYTES (LEDS * 3 + 1)
#endif
byte buf[RXBYTES];
*/
//#define RXBYTES (LEDS*3 + 1)

//#define RXBYTES (LEDS*3 + 1 > 32 ? 32 : LEDS*3 + 1)// 32 is MAX nRF can tx/rx
#define RXBYTES 4
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

#define dbg(str) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)

RF24 radio(7, 8); // CE, CSN

//byte addresses[][6] = {"1Node","2Node"};
byte address[] = "2Node";
//uint64_t address = "2Node";

unsigned long last = 0;
uint32_t serial;
#define LED_POWER_PIN 1

void setup()
{
  delay(1000);
  dbgln("starting");

  pinMode(LED_POWER_PIN, OUTPUT);
  digitalWrite(LED_POWER_PIN, HIGH); // XXX go low when wanting to turn off

  setupLEDs();
  /*
  while (1) {
    setColor(0x00ff00);
    delay(1);
  }
  */
  setupRadio();
  testLEDs();

  // don't use loop() to avoid serial stuff
  while (1) rx();
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
  DDRB |= _BV(WS_PIN); // _BV(ws2812_pin);
#else
  strip.begin();
  strip.show();
#endif
}

#define colorFromBuf(x) unsigned long color = ((unsigned long)buf[x] << 16) | ((unsigned long)buf[x+1] << 8) | (buf[x+2]);
void loop() { }
void rx()
{
// XXX readd once RF working
  if (millis() - last > 1000)
    setColor(0);

//delay(500);
  //delay(200);
  if (radio.available())
  {
//  setColor(0x00ff00); delay(100); setColor(0x000000);
    while (radio.available())
    {
  //setColor(0xff0000); delay(100); setColor(0x000000);delay(100);
      //radio.read(&rgb, RXBYTES); // Get the payload
      radio.read(&buf, RXBYTES); // Get the payload
//      setPixelColor(i, buf[0]);
//      show();
    }


    last = millis();
    // all balloons should turn this color
    if (buf[0] == '*')
    {
  //setColor(0x00ff00); delay(500); setColor(0x000000);
      //rgb = (cRGB*)(buf + 1); show();
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
c
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
