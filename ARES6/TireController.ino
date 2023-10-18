#include "variant.h"
#include <due_can.h>

typedef union {
  uint32_t uint32_data;
  uint8_t uint8_data[4];
} msg_union;

msg_union msg = {0};

//Leave defined if you use native port, comment if using programming port
//#define Serial SerialUSB

#include "CytronMotorDriver.h"

//Driver Tire
CytronMD tireRF(PWM_DIR, 2, 10); //0x200
CytronMD tireRB(PWM_DIR, 3, 11); //0x201
CytronMD tireLF(PWM_DIR, 4, 12); //0x202
CytronMD tireLB(PWM_DIR, 5, 13); //0x203

//Driver Steer
CytronMD steerRF(PWM_DIR, 6, 30); //0x210
CytronMD steerRB(PWM_DIR, 7, 31);//0x211
CytronMD steerLF(PWM_DIR, 8, 32); //0x212
CytronMD steerLB(PWM_DIR, 9, 33); //0x213

//Encoder Tire
const int enTireRF = A0; //0x300
const int enTireRB = A1; //0x301
const int enTireLF = A2; //0x302
const int enTireLB = A3; //0x303

//Encoder Steer
#include <Encoder.h>
Encoder enSteerRF(22, 23); //0x310
Encoder enSteerRB(24, 25); //0x311
Encoder enSteerLF(26, 27); //0x312
Encoder enSteerLB(28, 29); //0x313

#include <DueTimer.h>
bool timer_100Hz = false;

void setup()
{

  Serial.begin(115200);

  // Initialize CAN0 and CAN1, Set the proper baud rates here
  Can0.begin(CAN_BPS_1000K); //This value should be uniform for all programs.

  Can0.watchFor();

  pinMode(enTireRF, INPUT);
  pinMode(enTireRB, INPUT);
  pinMode(enTireLF, INPUT);
  pinMode(enTireLB, INPUT);

  //Timer Configuration
  Timer3.attachInterrupt(ISR_100Hz);
  Timer3.start(10000); 
}

void send_CAN(uint32_t id, uint32_t data)
{
  msg.uint32_data = data;
  CAN_FRAME outgoing;
  outgoing.id = id;
  outgoing.extended = false;
  outgoing.priority = 3; //0-15 lower is higher priority

  outgoing.length = 8;

  outgoing.data.bytes[0] = msg.uint8_data[0];
  outgoing.data.bytes[1] = msg.uint8_data[1];
  outgoing.data.bytes[2] = msg.uint8_data[2];
  outgoing.data.bytes[3] = msg.uint8_data[3];
  
  Can0.sendFrame(outgoing);
}

void ISR_100Hz() {
  if (timer_100Hz) {
    //Serial.println("100Hz overrun");
  } else {
    timer_100Hz = true;
  }
}

void loop() {
  CAN_FRAME incoming;
  static unsigned long lastTime = 0;

  if (Can0.available() > 0) {
    Can0.read(incoming);

    /* //for debug
    Serial.println("received data via CAN0");
    Serial.println(incoming.id, HEX);
    for (int i = 0; i < 8; i++) {
      Serial.println(incoming.data.byte[i], HEX);
    }
    */

    long unsigned int rxId = incoming.id;
    msg.uint8_data[0] = incoming.data.bytes[0];
    msg.uint8_data[1] = incoming.data.bytes[1];
    msg.uint8_data[2] = incoming.data.bytes[2];
    msg.uint8_data[3] = incoming.data.bytes[3];

    switch (rxId) {
      case 0x200:
        tireRF.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x201:
        tireRB.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x202:
        tireLF.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x203:
        tireLB.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x210:
        steerRF.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x211:
        steerRB.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x212:
        steerLF.setSpeed((int)msg.uint32_data - 256);
        break;
      case 0x213:
        steerLB.setSpeed((int)msg.uint32_data - 256);
        break;  
    }

    
  }

  if (timer_100Hz) {
    timer_100Hz = false;
    
    send_CAN(0x310, analogRead(enTireRF));
    send_CAN(0x311, analogRead(enTireRB));
    send_CAN(0x312, analogRead(enTireLF));
    send_CAN(0x313, analogRead(enTireLB));
    
    send_CAN(0x310, abs(enSteerRF.read()));
    send_CAN(0x311, abs(enSteerRB.read()));
    send_CAN(0x312, abs(enSteerLF.read()));
    send_CAN(0x313, abs(enSteerLB.read()));
  }
}