#include <Wire.h>
#include <SPI.h> //idk if we need this
#include <Adafruit_Sensor.h> //BMP
#include <SoftwareSerial.h> //XBEE
#include "Adafruit_BMP5xx.h" //BMP581
#include <Adafruit_INA260.h> //INA260
#include <SparkFun_u-blox_GNSS_v3.h> //SAM-M10Q
#include <Adafruit_BNO08x.h> //BNO085
#include <Servo.h> //Servos ofc

// Pins
const uint8_t XBee_RX = 34; //idk what pins are connected
const uint8_t XBee_TX = 35; //idk what pins are connected

// Sensors
SoftwareSerial XBee(XBee_TX, XBee_RX); //XBEE
Adafruit_BMP5xx bmp; //Declaring BMP581

// Servo's
Servo release_s; //Release Mechanism
Servo port_s; //Right Pull Mechanism
Servo starboard_s; //Left Pull Mechanism

//Defines Pressure for the BMP
#define SEALEVELPRESSURE_HPA (1013.25)

//Declarations
double speed_x;
double speed_y;
double speed_z;

// Servo Pins
const uint8_t release_s_pin = 9; //idk what servo pin yet
const uint8_t port_s_pin = 9; //idk what servo pin yet
const uint8_t starboard_s_pin = 9; //idk what servo pin yet

// states
String sw_states[7] = {"LAUNCH_PAD", "ASCENT", "APOGEE", "DESCENT", "PROBE_RELEASE", "PAYLOAD_RELEASE", "LANDED"};
String sw_state = sw_state[0];

// telemetry declarations
int TEAM_ID = 1094;
//MISSION_TIME idk how to do this just yet hh:mm:ss
int PACKET_COUNT = 0;
//char MODE = "F"; //F or S
//STATE has already been decleared
//ALLTITUDE will probe be a double but has a resolution of 0.1 meters
double TEMPERATURE;
double PRESSURE;
double VOLTAGE;
double CURRENT;
int GYRO_R;
int GYRO_P;
int GYRO_Y;
int ACCEL_R;
int ACCEL_P;
int ACCEL_Y;
//gps time same as time idk
double GPS_ALLTITUDE;
double GPS_LATITUDE;
double GPS_LONGITUDE;
int GPS_STATS;
String CMD_ECHO;
double STRB_ANGLE;
double PORT_ANGLE;
double VECTOR_PRODUCT;


//temporary
    double ALTITUDE = 0;
    double VELOCITY = 0;
    double peak_altitude = 0;
    double release = 100000000;


void heading_error() {


}

void data() {


}

void telemetry() {


}

void pro_restart() {


}

void setup() {
  Serial.begin(9600);
  release_s.attach(release_s_pin);
  port_s.attach(port_s_pin);
  starboard_s.attach(starboard_s_pin);

}


void loop() {
  if(sw_state == "LAUNCH_PAD"){
    if(ALTITUDE >= 5 && VELOCITY >=5){
      sw_state = sw_states[1];
    }
  }

  else if(sw_state == "ASCENT"){
    if(VELOCITY <=0){     //this might not catch it
      sw_state = sw_states[2];
    }
  }

  else if(sw_state == "APOGEE"){
    peak_altitude = ALTITUDE;
    release = peak_altitude * 0.8;
    sw_state = sw_states[3];
  }

  else if(sw_state == "DESCENT"){
    if(ALTITUDE == release){
      sw_state = sw_states[4];
    }
  }

  else if(sw_state == "PROBE_RELEASE"){
    if(ALTITUDE <= 2){
      //release prob ofc
      sw_state = sw_states[5];
    }
  }

  else if(sw_state == "PAYLOAD_RELEASE"){
    if(VELOCITY <= 0.2){
      sw_state = sw_states[6];
    }
  }

}
