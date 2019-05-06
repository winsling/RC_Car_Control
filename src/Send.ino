// Simple RFM12B sender program, with ACK and optional encryption
// It initializes the RFM12B radio with optional encryption and passes through any valid messages to the serial port
// felix@lowpowerlab.com

#include <RFM12B.h>
#include <avr/sleep.h>

// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID        2  //network ID used for this unit
#define NETWORKID    99  //the network ID we are on
#define GATEWAYID     1  //the node ID we're sending to
#define ACK_TIME     50  // # of ms to wait for an ack
#define SERIAL_BAUD  115200

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";

int interPacketDelay = 50; //wait this many ms between sending packets
char input = 0;
int SpdPin = 0;
int StePin = 1;
int Ste = 0;
int Spd = 0;

struct command_type {
  int Speed;
  int SteeringAngle;
};

union SerializedData_type {
  command_type command;
  char command_serial[14];
} SerializedData;

// Need an instance of the Radio Module
RFM12B radio;
byte sendSize=0;
char payload[] = "StAng20 Spd40";
bool requestACK=false;
String Command;
void setup()
{
  Serial.begin(SERIAL_BAUD);
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);
  radio.Sleep(); //sleep right away to save power
  Serial.println("Transmitting...\n\n");
}

void loop()
{
  SerializedData.command.SteeringAngle = map(analogRead(StePin),0,1023,0,180);
  SerializedData.command.Speed = map(analogRead(SpdPin),0,1023,70,96);
  interPacketDelay = 200;

  Serial.print("Sending:");

  
  //Command = String(Spd)+" "+String(Ste);
  //Command.toCharArray(payload, 14);

  //requestACK = !(sendSize % 3); //request ACK every 3rd xmission
  requestACK = 1;
  radio.Wakeup();
  radio.Send(GATEWAYID, SerializedData.command_serial, 14, requestACK);
  if (requestACK)
  {
    Serial.print(" - waiting for ACK...");
    if (waitForAck()) Serial.println("ok!");
    else Serial.println("nothing...");
  }
  radio.Sleep();
  
  sendSize = (sendSize + 1) % 13;
  Serial.println();
  delay(interPacketDelay);
}

// wait a few milliseconds for proper ACK, return true if received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}