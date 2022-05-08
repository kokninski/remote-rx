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

#define SERVO0 5 // aileron_left
#define SERVO1 6 // aileron_right 
#define SERVO2 7 // elevator
#define SERVO3 8 // rudder
#define SERVO4 4 // throttle

Servo servo0; // aileron_left
Servo servo1; // aileron_right
Servo servo2; // elevator
Servo servo3; // rudder
Servo servo4; // throttle

#define AILERON_LEFT_INVERTED 1 // -1 for inverted 1 for not inverted
#define AILERON_RIGHT_INVERTED 1 // -1 for inverted 1 for not inverted
#define ELEVATOR_INVERTED 1 // -1 for inverted 1 for not inverted
#define RUDDER_INVERTED 1 // -1 for inverted 1 for not inverted

#define AILERON_LPF_ALPHA 0.8 // low pass filter alpha (0.0 - 1.0)
#define ELEVATOR_LPF_ALPHA 0.8 // low pass filter alpha (0.0 - 1.0)
#define RUDDER_LPF_ALPHA 0.8 // low pass filter alpha (0.0 - 1.0)
#define THROTTLE_LPF_ALPHA 0.8 // low pass filter alpha (0.0 - 1.0)

#define THROTTLE_MODE 0 // 0 for normal 1 for incremental

#if THROTTLE_MODE == 1
#define MAX_THROTTLE_INCREMENT 300 
#endif

#define AILERON_MIN_PULSE_WIDTH 1200 // minimum pulse width in microseconds
#define AILERON_MAX_PULSE_WIDTH 1800 // maximum pulse width in microseconds
#define ELEVATOR_MIN_PULSE_WIDTH 1200 // minimum pulse width in microseconds
#define ELEVATOR_MAX_PULSE_WIDTH 1800 // maximum pulse width in microseconds
#define RUDDER_MIN_PULSE_WIDTH 1200 // minimum pulse width in microseconds
#define RUDDER_MAX_PULSE_WIDTH 1800 // maximum pulse width in microseconds
#define THROTTLE_MIN_PULSE_WIDTH 700 // minimum pulse width in microseconds
#define THROTTLE_MAX_PULSE_WIDTH 2000 // maximum pulse width in microseconds

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
    // overload the constructor to accept alpha
    ServoController(Servo servo, int min, int max, float alpha){
      this->servo = servo;
      this->min = min;
      this->max = max;
      this->prev_value = 0;
      this->filter = LowPassFilter(alpha);
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
    int get_position(){
      return this->prev_value;
    }
    void set_filter_constant(float alpha){
      this->filter.set_alpha(alpha);
    }
    void set_min(int min){
      this->min = min;
    }
    void set_max(int max){
      this->max = max;
    }

  private:
    Servo servo;
    int min;
    int max;
    int prev_value;
    LowPassFilter filter;
};

// Declare servo controllers for each servo
ServoController sc0(servo0, AILERON_MIN_PULSE_WIDTH, AILERON_MAX_PULSE_WIDTH); // aileron
ServoController sc1(servo1, AILERON_MIN_PULSE_WIDTH, AILERON_MAX_PULSE_WIDTH); // aileron
ServoController sc2(servo2, ELEVATOR_MIN_PULSE_WIDTH, ELEVATOR_MAX_PULSE_WIDTH); // elevator
ServoController sc3(servo3, RUDDER_MIN_PULSE_WIDTH, RUDDER_MAX_PULSE_WIDTH); // rudder
ServoController sc4(servo4, THROTTLE_MIN_PULSE_WIDTH, THROTTLE_MAX_PULSE_WIDTH); // throttle

void initPins(void){
  pinMode(SERVO0, OUTPUT);
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  servo0.attach(SERVO0);
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);

  sc0.set_position(2048);
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

  sc0.set_filter_constant(AILERON_LPF_ALPHA);
  sc1.set_filter_constant(AILERON_LPF_ALPHA);
  sc2.set_filter_constant(ELEVATOR_LPF_ALPHA);
  sc3.set_filter_constant(RUDDER_LPF_ALPHA);
  sc4.set_filter_constant(THROTTLE_LPF_ALPHA);
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

      // Diferential control for the aileron
      // The aileron deflecting up shoud move more than the aileron deflecting down
      // scale factors define the amount of movement
      
      int aileron_position = data->ch1_y - 2048;
      int positive_scale_factor = 1;
      int negative_scale_factor = 2;
      int scale_l = (aileron_position > 0) ? positive_scale_factor : negative_scale_factor;
      int scale_r = (aileron_position > 0) ? negative_scale_factor : positive_scale_factor;
      
      int aileron_r = AILERON_RIGHT_INVERTED ? 2048 - aileron_position * scale_r : aileron_position * scale_r + 2048;
      int aileron_l = AILERON_LEFT_INVERTED ? 2048 - aileron_position * scale_l : aileron_position * scale_l + 2048;
      
      int elevator_position = ELEVATOR_INVERTED ? 4096 - data->ch1_x : data->ch1_x; 
      int rudder_position = RUDDER_INVERTED ? 4096 - data->ch2_x : data->ch2_x;


      #if THROTTLE_MODE == 0
      int throttle_position = data->ch2_y;

      // Incremental throttle implementation
      #elif THROTTLE_MODE == 1
          int prev_throttle_position = sc4.get_position();
          int throttle_increment = map(data->ch2_y, 0, 4096, -MAX_THROTTLE_INCREMENT, MAX_THROTTLE_INCREMENT);
          int throttle_position = prev_throttle_position + data->ch2_y;
          if(throttle_position < 0){
            throttle_position = 0;
          }
          if(throttle_position > 4096){
            throttle_position = 4096;
          }
      #endif


      // Write the new positions to servos
      sc0.set_position(aileron_l); // aileron_left
      sc1.set_position(aileron_r); // aileron_right
      sc2.set_position(elevator_position); // elevator
      sc3.set_position(rudder_position); // rudder
      sc4.set_position(throttle_position); // throttle
    
    }
    else
    { 
      Serial.println("recv failed");
    }
  }
}
