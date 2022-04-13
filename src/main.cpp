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

class LowPassFilter{
  public:
    LowPassFilter(float alpha){
      this->alpha = alpha;
      this->prev_value = 0;
    }
    LowPassFilter(){
      this->alpha = 0.5;
      this->prev_value = 0;
    }
    float filter(float value){
      float result = this->alpha * value + (1 - this->alpha) * this->prev_value;
      this->prev_value = result;
      return result;
    }
    void set_alpha(float alpha){
      this->alpha = alpha;
    }
  private:
    float alpha;
    float prev_value;
};

class ServoController{
  public:
    ServoController(Servo servo, int min, int max){
      this->servo = servo;
      this->min = min;
      this->max = max;
      this->prev_value = 0;
      this->filter = LowPassFilter(0.9);
    }
    int map_servo(int value){
      return map(value, 0, 4096, this->min, this->max);
    }
    void set_position(int position){
      // Serial.print("Setting position to ");
      int limited_position = this->map_servo(position);
      int filtered_position = this->filter.filter(limited_position);
      // Serial.println(this->prev_value);

      // Check if the position is different from the previous one
      if(filtered_position != this->prev_value){
        // Check if position is within bounds
        if(filtered_position < this->min){
          filtered_position = this->min;
        }
        if(filtered_position > this->max){
          filtered_position = this->max;
        }
        // Update the servo position only if it has changed
        this->servo.writeMicroseconds(filtered_position);
        // Serial.println(filtered_position);
        this->prev_value = filtered_position;
      }
    }
    void set_filter_constant(float alpha){
      this->filter.set_alpha(alpha);
    }
  private:
    Servo servo;
    int min;
    int max;
    int prev_value;
    LowPassFilter filter;
};

// Declare servo controllers for each servo
ServoController sc1(servo1, 1200, 1800); // aileron
ServoController sc2(servo2, 1200, 1800); // elevator
ServoController sc3(servo3, 1200, 1800); // rudder
ServoController sc4(servo4, 700, 2000); // throttle

void initPins(void){
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);

  sc1.set_position(2048);
  sc2.set_position(2048);
  sc3.set_position(2048);
  sc4.set_position(500);

}
void setup() 
{
  Serial.begin(9600);
  while(!Serial){} // wait for serial
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
    // Define buffer for incoming message
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
      // create a DataFrame object from the received data
      DataFrame *data = (DataFrame*)buf;

      // Print out the data for debugging
      char b [50];
      // sprintf(b, "CH1X: %i CH1Y: %i\nCH2X: %i CH2Y: %i\n", \
          data->ch1_x, data->ch1_y, data->ch2_x, data->ch2_y);
      unsigned long dt = millis()-prev_millis;
      // sprintf(b, "%i,%i,%i,%i,%i\n", \
      //     (int) dt, data->ch1_x, data->ch1_y, data->ch2_x, data->ch2_y);
      // Serial.print(b);
      
      // Serial.println(millis()-prev_millis);
      prev_millis = millis();

      sc1.set_position(data->ch1_x); // aileron
      sc2.set_position(data->ch1_y); // elevator
      sc3.set_position(data->ch2_x); // rudder
      sc4.set_position(data->ch2_y); // throttle
    

    }
    else
    { 
      Serial.println("recv failed");
    }
  }
}
