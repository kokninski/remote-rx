#include <Arduino.h>
// nrf24_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_NRF24 class. RH_NRF24 class does not provide for addressing or
// reliability, so you should only use RH_NRF24  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf24_client
// Tested on Uno with Sparkfun NRF25L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <SPI.h>
#include <RH_NRF24.h>
#include <Servo.h>

#define SERVO1 5
#define SERVO2 6
#define SERVO3 7
#define SERVO4 8

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Singleton instance of the radio driver
RH_NRF24 nrf24(10, 9); // CE, CSN
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini

struct DataFrame{
  uint16_t ch1_x;
  uint16_t ch1_y;
  uint16_t ch2_x;
  uint16_t ch2_y;

  bool but1;
  bool but2;
  bool but3;
  bool but4;
};
unsigned long prev_millis = 0;

void initPins(void){
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);

  servo1.writeMicroseconds(map(2048,0,4096,1200,1800));
  servo2.writeMicroseconds(map(2048,0,4096,1200,1800));
  servo3.writeMicroseconds(map(2048,0,4096,500,2500));
  servo4.writeMicroseconds(map(2048,2048,4096,700,2500));
  
}
void setup() 
{
  Serial.begin(9600);
  while (!Serial) 
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
  Serial.println("Beginning");
  nrf24.printRegisters();
  initPins();
}

void loop()
{
  if (nrf24.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
    //  NRF24::printBuffer("request: ", buf, len);
      // Serial.print("got request (length ");
      // Serial.print(len);
      // Serial.println(").");
      
      DataFrame *data = (DataFrame*)buf;

      char b [50];
      // sprintf(b, "CH1X: %i CH1Y: %i\nCH2X: %i CH2Y: %i\n", \
          data->ch1_x, data->ch1_y, data->ch2_x, data->ch2_y);
      unsigned long dt = millis()-prev_millis;
      sprintf(b, "%i,%i,%i,%i,%i\n", \
          (int) dt, data->ch1_x, data->ch1_y, data->ch2_x, data->ch2_y);
      // Serial.print(b);
      
      // Send a reply
      // uint8_t data[] = "And hello back to you";
      // nrf24.send(data, sizeof(data));
      // nrf24.waitPacketSent();
      // Serial.println("Sent a reply");

      // Serial.println(millis()-prev_millis);
      prev_millis = millis();

      int servo1_micros = map(data->ch1_y,0,4096,2000,1000);
      int servo2_micros = map(data->ch1_x,0,4096,1200,2000);
      int servo3_micros = map(data->ch2_x,0,4096,1200,1800);
      int servo4_micros = map(data->ch2_y,2048,4096,700,2000);

      if (servo1_micros < 1000) servo1_micros = 1000;
      if (servo1_micros > 2000) servo1_micros = 2000;
      if (servo2_micros < 1200) servo2_micros = 1200;
      if (servo2_micros > 2000) servo2_micros = 2000;



      sprintf(b, "%i,%i,%i,%i\n", \
          servo1_micros, servo2_micros, servo3_micros, servo4_micros);
      Serial.print(b);

      servo2.writeMicroseconds(servo2_micros);
      servo1.writeMicroseconds(servo1_micros);
      servo3.writeMicroseconds(servo3_micros);
      servo4.writeMicroseconds(servo4_micros);
    }
    else
    { 
      Serial.println("recv failed");
    }
  }
}
