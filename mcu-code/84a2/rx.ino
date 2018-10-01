#include <RF24.h>
//#include <Adafruit_NeoPixel.h>
#include "WS2812.h"

// 4384,149 vs 3850,145
// 3826,

#define WS_PIN 0
#define LEDS 1

#ifdef WS2812_H_
WS2812 LED(LEDS);
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);
#endif


#define dbg(str) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)

// radio(CE, CSN)
RF24 radio(7, 8);

byte addresses[][6] = {"1Node","2Node"};

bool role = 0; // 0 = pong back , 1 = ping out
unsigned long last = 0;
uint32_t serial;

void setup() {
  Serial.begin(115200);
  //while (!Serial) ; // wait for arduino serial monitor
  delay(2000); // WNG - println's below started working after I put in this delay
  dbgln("RF24/examples/GettingStarted");

  // XXX flash a unique serial and read it out again
  //serial = getTeensySerial();

#ifdef WS2812_H_
  LED.setOutput(WS_PIN);
#else
  strip.begin();
  strip.show();
#endif

  dbgln("*** PRESS 'T' to begin transmitting to the other node");
  radio.begin();

  //radio.setPALevel(RF24_PA_HIGH);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS); // RF24_2MBPS
  radio.setChannel(125); // XXX

  if(role){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();
  radio.printDetails(); 
}

void loop()
{
  if (millis() - last > 1000)
    setColor(0);

#define RXBYTES 128
  byte buf[RXBYTES];

  if (radio.available())
  {
    //out(radio.available());
    while (radio.available())
    {
      radio.read(&buf, RXBYTES); // Get the payload
      /*
      out(buf[0]);
      out(buf[1]);
      out(buf[2]);
      out(buf[3]);
      */
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
#ifdef WS2812_H_
    LED.sync();
#else
    strip.show();
#endif
}

void setPixelColor(int i, unsigned long color)
{
#ifdef WS2812_H_
  cRGB val;
  val.r = (color >> 16) & 0xFF;
  val.g = (color >>  8) & 0xFF;
  val.b = (color >>  0) & 0xFF;
  LED.set_crgb_at(i, val);
#else
  strip.setPixelColor(i, color);
#endif
}
