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
int StatusLEDPin = 3;
int FrontLightSwitchPin = 4;
int ENPOSwitchPin = 5;
int Ste = 0;
int Spd = 0;

struct command_type {
  int Speed;
  int SteeringAngle;
  bool FrontLight;
  bool ENPO;

};

union SerializedData_type {
  command_type command;
  char command_serial[8];
} SerializedData;

// Need an instance of the Radio Module
RFM12B radio;
bool requestACK=false;

void setup()
{


  Serial.begin(SERIAL_BAUD);
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);
  // radio.Sleep(); //sleep right away to save power
  radio.Wakeup();
  pinMode(StatusLEDPin,OUTPUT);
  pinMode(FrontLightSwitchPin,INPUT);
  pinMode(ENPOSwitchPin,INPUT_PULLUP);
  }




void rfm_handling()
{
  unsigned long t1,t2;
  int Steer_Temp=analogRead(StePin);

  if ((Steer_Temp > 500) && (Steer_Temp < 523)) Steer_Temp = 511; // Filter Poti noise

  t1=millis();
  
  SerializedData.command.SteeringAngle = map(Steer_Temp,0,1023,0,180);
  SerializedData.command.Speed = map(analogRead(SpdPin),0,1023,70,96);
  SerializedData.command.FrontLight = digitalRead(FrontLightSwitchPin) > 0;
  SerializedData.command.ENPO = digitalRead(ENPOSwitchPin) > 0;
  
  requestACK = 1;
  //radio.Wakeup(); // removed for speed up ?
  radio.Send(GATEWAYID, SerializedData.command_serial, 8, requestACK);

  if (requestACK)
    {
      if (waitForAck()) digitalWrite(StatusLEDPin,HIGH);
      else digitalWrite(StatusLEDPin,LOW);
    }
  //radio.Sleep();
  t2=millis();
  //Serial.println(t2-t1);
  Serial.println(SerializedData.command.ENPO);
}

void loop()
{
  rfm_handling();
}

// wait a few milliseconds for proper ACK, return true if received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}
