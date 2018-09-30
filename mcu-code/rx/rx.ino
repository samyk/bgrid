#include <SPI.h>
#include <math.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

#define WS_PIN 0
#define LEDS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);

#define dbg(str) void()
#define dbgln(str) void()
#define out(x) Serial.println(x)

// radio(CE, CSN)
RF24 radio(7, 8);

byte addresses[][6] = {"1Node","2Node"};

bool role = 0; // 0 = pong back , 1 = ping out
unsigned long last = 0;
unsigned long pw;
uint32_t serial;

void setup() {
  Serial.begin(115200);
  pw = (unsigned long)pow(2, 24);
  //while (!Serial) ; // wait for arduino serial monitor
  delay(2000); // WNG - println's below started working after I put in this delay
  dbgln("RF24/examples/GettingStarted");

  serial = getTeensySerial();

  strip.begin();
  strip.show();

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
  {
    for (int i = 0; i < strip.numPixels(); i++)
      strip.setPixelColor(i, 0);
    strip.show();
  }

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
      unsigned long color = ((unsigned long)buf[1] << 16) | ((unsigned long)buf[2] << 8) | (buf[3]);
      setColor(color);
    }

    // some number of balloons turn this color if uniqid % buf[1] == buf[2]
    else if (buf[0] == 'U')
    {
      if ((serial % buf[1]) == buf[2])
      {
        unsigned long color = ((unsigned long)buf[3] << 16) | ((unsigned long)buf[4] << 8) | (buf[5]);
        setColor(color);
      }
    }
  }
}


void setColor(unsigned long color)
{
  for (int i = 0; i < strip.numPixels(); i++)
    strip.setPixelColor(i, color);
  strip.show();
}


#define MY_SYSREGISTERFILE	((uint8_t *)0x40041000) // System Register File

static uint32_t getTeensySerial(void) {
	uint32_t num;
	__disable_irq();
	#if defined(HAS_KINETIS_FLASH_FTFA) || defined(HAS_KINETIS_FLASH_FTFL)
		FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
		FTFL_FCCOB0 = 0x41;
		FTFL_FCCOB1 = 15;
		FTFL_FSTAT = FTFL_FSTAT_CCIF;
		while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
		num = *(uint32_t *)&FTFL_FCCOB7;
	#elif defined(HAS_KINETIS_FLASH_FTFE)
		kinetis_hsrun_disable();
		FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
		*(uint32_t *)&FTFL_FCCOB3 = 0x41070000;
		FTFL_FSTAT = FTFL_FSTAT_CCIF;
		while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
		num = *(uint32_t *)&FTFL_FCCOBB;
		kinetis_hsrun_enable();
	#endif
	__enable_irq();
	return num;
	}

