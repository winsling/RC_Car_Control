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
int StatusPin = 1;
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
bool requestACK=false;

// Es gilt prescale / cpufreq * count = deltaT 
// => count = deltaT * cpufreq / prescale count = 0.2 * 16000000 / 256 = 12500
// => maxcount - initcount = deltaT * cpufreq / prescale 
// => initcount = maxcount - deltaT * cpufreq / prescale initcount = 65536 - 12500 = 53036
// Beispielsrechnung: Alle 0,5 Sekunden soll ein Timer-Overflow-Interrupt stattfinden.
// Wir verwenden einen 16-Bit-Timer: bits = 16 => maxcount = 216 = 65536.
// Wir benötigen einen Timer Overflow pro halbe Sekunde. deltaT = 0,5 sec = 1 / timerfreq
// Die Taktfrequenz des Arduino-Board beträgt cpufreq = 16 MHz = 16.000.000 Hz
// Als Prescale-Wert liegt prescale = 256 vor.
// Der Timer startet statt mit 0 mit folgendem Anfangszählerstand initcount = 65.536 - 8.000.000/256 = 34.286


void setup()
{


  // Timer 1
  noInterrupts();           // Alle Interrupts temporär abschalten
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 53036;            // Timer nach obiger Rechnung vorbelegen
  TCCR1B |= (1 << CS12);    // 256 als Prescale-Wert spezifizieren
  TIMSK1 |= (1 << TOIE1);   // Timer Overflow Interrupt aktivieren
  interrupts();             // alle Interrupts scharf schalten

  
  Serial.begin(SERIAL_BAUD);
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);
  radio.Sleep(); //sleep right away to save power
  Serial.println("Transmitting...\n\n");
}

ISR(TIMER1_OVF_vect)        
{
  TCNT1 = 53036;             // Zähler erneut vorbelegen
  rfm_handling();
}



void rfm_handling()
{
  SerializedData.command.SteeringAngle = map(analogRead(StePin),0,1023,0,180);
  SerializedData.command.Speed = map(analogRead(SpdPin),0,1023,70,96);
  
  requestACK = 1;
  radio.Wakeup();
  radio.Send(GATEWAYID, SerializedData.command_serial, 14, requestACK);
  if (requestACK)
  {

    if (waitForAck()) {

      digitalWrite(StatusPin,HIGH);
    }
    else {

      digitalWrite(StatusPin,LOW);
    }
  }
  radio.Sleep();

}

void loop()
{
}

// wait a few milliseconds for proper ACK, return true if received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}
