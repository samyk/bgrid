#include <SPI.h>
#include <math.h>
#include <RF24.h>

#define MAX_BYTES 32
#define MAX_SER_BYTES 0xFF
#define RGB_SIZE 3

//#define DEBUG
#ifdef DEBUG
#define pf(...) printf(__VA_ARGS__)
#else
#define pf(...)
#endif

#define dbg(str) void()
#define dbgln(str) void()
#define out(x) Serial.print(x)
#define outh(x) Serial.print(x, HEX)
#define outln(x) Serial.println(x)

// radio(CE, CSN)
RF24 radio(7, 8);

byte addresses[][6] = {"1Node","2Node"};

byte multicast = 1; // turns off ack request
unsigned long last = 0;

void setup() {
  delay(2000);

  radio.begin();
  //radio.setPALevel(RF24_PA_HIGH);
  //radio.setCRCLength(RF24_CRC_16); // XXX haven't tested
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setAutoAck(false);
  radio.setAddressWidth(5);
  radio.setDataRate(RF24_250KBPS); // RF24_2MBPS
  radio.setChannel(125); // XXX

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  //radio.startListening();
  //radio.printDetails();
}

byte mem[MAX_BYTES];
byte sermem[MAX_SER_BYTES];

#define BAD_DATA do { return; } while(0)

void readPix()
{
#define MAX_RGB_PER_PACKET 9
#define RGB_PACKET_SIZE (MAX_RGB_PER_PACKET * RGB_SIZE)

  int startid = 0;
  mem[0] = 'S';
  mem[1] = 0;
  mem[2] = MAX_RGB_PER_PACKET;
  for (int j = 0; j < RGB_PACKET_SIZE; j++)
    mem[3+j] = 0xFF;

  addresses[1][0] = (startid) / MAX_RGB_PER_PACKET;
  radio.openWritingPipe(addresses[1]);
  radio.write(&mem, RGB_PACKET_SIZE+3, 1);
}

void loop()
{
  readPix();
}
