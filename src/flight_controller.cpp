// Basic fligh controller for the fixed wing aircraft

#include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>


#define SDA 10
#define SCL 11

// structure for the accelerometer and gyro data
struct MPUData{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
};

// structure for the flight state
struct FlightSetpoint{
    int pitch;
    int roll;
    int yaw;
    int throttle;
};

struct FlightState{
    int pitch;
    int roll;
    int yaw;
    int throttle;
};

struct FlightControlPosition{
    int pitch;
    int roll;
    int yaw;
    int throttle;
};

struct Envelope{
    float max = 30; // deg
    float min = -30; // deg
    int max_input = 4096;
    int min_input = 0;
};

struct FlightEnvelope{
    Envelope pitch;
    Envelope roll;
    Envelope yaw;
    Envelope throttle;
};

struct PID_Params{
    float kp = 1.0;
    float ki = 0.0;
    float kd = 0.0;
};
// this class is a basic flight controller with PID loops for pitch, roll, yaw, and throttle
class FlightControler{
    public:
        FlightControler(){
            
            this->pitch_pid = PID((double *) &this->flight_state.pitch,(double *) &this->control_position.pitch,(double *) &this->flight_setpoint.pitch,
                                this->pitch.kp, this->pitch.ki, this->pitch.kd, DIRECT);
            this->roll_pid = PID((double *) &this->flight_state.pitch,(double *) &this->control_position.pitch,(double *) &this->flight_setpoint.pitch,
                                this->roll.kp, this->roll.ki, this->roll.kd, DIRECT);
            this->yaw_pid = PID((double *) &this->flight_state.pitch,(double *) &this->control_position.pitch,(double *) &this->flight_setpoint.pitch,
                                this->yaw.kp, this->yaw.ki, this->yaw.kd, DIRECT);
            this->throttle_pid = PID((double *) &this->flight_state.pitch,(double *) &this->control_position.pitch,(double *) &this->flight_setpoint.pitch, 
                                this->throttle.kp, this->throttle.ki, this->throttle.kd, DIRECT);

            this->pitch_pid.SetMode(AUTOMATIC);
            this->roll_pid.SetMode(AUTOMATIC);
            this->yaw_pid.SetMode(AUTOMATIC);
            this->throttle_pid.SetMode(AUTOMATIC);
        };
        FlightControlPosition get_control_position(){
            return this->control_position;
        }
        FlightState compute_flight_state(MPUData data){
            // compute the euler angles from the gyroscope data
            float pitch = atan2(data.accel_y, data.accel_z) * 180 / M_PI;
            float roll = atan2(data.accel_x, data.accel_z) * 180 / M_PI;
            float yaw = atan2(data.accel_y, data.accel_x) * 180 / M_PI;

            // compute the throttle from the accelerometer data
            float throttle = map(data.accel_z, 0, 4096, 0, 100);

            // map the euler angles to the control position
            FlightState state;
            state.pitch = map(pitch, this->flight_envelope.pitch.min, this->flight_envelope.pitch.max, this->flight_envelope.pitch.min_input, this->flight_envelope.pitch.max_input);
            state.roll = map(roll, this->flight_envelope.roll.min, this->flight_envelope.roll.max, this->flight_envelope.roll.min_input, this->flight_envelope.roll.max_input);
            state.yaw = map(yaw, this->flight_envelope.yaw.min, this->flight_envelope.yaw.max, this->flight_envelope.yaw.min_input, this->flight_envelope.yaw.max_input);
            state.throttle = map(throttle, this->flight_envelope.throttle.min, this->flight_envelope.throttle.max, this->flight_envelope.throttle.min_input, this->flight_envelope.throttle.max_input);

            return state;
        }
        void update_flight_state(){
            MPUData mpu_data = getMPUData();
            FlightState state = compute_flight_state(mpu_data);

            // TODO
            this->flight_state.pitch = state.pitch;
            this->flight_state.roll = state.roll;
            this->flight_state.yaw = state.yaw;
            this->flight_state.throttle = state.throttle;            
        }
        void set_pid(PID *pid, float kp, float ki, float kd){
            pid->SetTunings(kp, ki, kd);
        }
        void set_envelope(FlightEnvelope *envelope){
            this->flight_envelope = *envelope;
        }

        void run(MPUData *mpu_data, FlightState *flight_state){
            update_flight_state();
            this->pitch_pid.Compute();
            this->roll_pid.Compute();
            this->yaw_pid.Compute();
            this->throttle_pid.Compute();
        }


    private:
        PID pitch_pid;
        PID roll_pid;
        PID yaw_pid;
        PID throttle_pid;

        PID_Params pitch;
        PID_Params roll;
        PID_Params yaw;
        PID_Params throttle;
        
        FlightEnvelope flight_envelope;
        FlightSetpoint flight_setpoint;
        FlightControlPosition control_position;
        FlightState flight_state;


}