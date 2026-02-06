#include <Wire.h>
#include <Arduino.h>
#include <SPI.h> //idk if we need this
#include <Adafruit_Sensor.h> //BMP581
#include <SoftwareSerial.h> //XBEE
#include "Adafruit_BMP5xx.h" //BMP581
#include <Adafruit_INA260.h> //INA260
#include <SparkFun_u-blox_GNSS_v3.h> // SAM-M10Q
#include <Adafruit_BNO08x.h> // BNO085
#include <Servo.h> // Servos ofc

//======
// Pins
//======
//const uint8_t XBee_RX = 35; // idk what pins are connected
//const uint8_t XBee_TX = 34; // idk what pins are connected

//=========
// Sensors
//=========
//SoftwareSerial XBee(XBee_TX, XBee_RX); //XBEE use <Serial8.begin(9600);
Adafruit_BMP5xx bmp581; //Declaring BMP581
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode
SFE_UBLOX_GNSS sam_m10q; //Declaring SAM-M10Q
Adafruit_Sensor *bmp_temp = NULL;
Adafruit_Sensor *bmp_pressure = NULL;

//=========
// Servo's
//=========
Servo release_s; //Release Mechanism
Servo port_s; //Right Pull Mechanism
Servo starboard_s; //Left Pull Mechanism

//=============================
//Defines Pressure for the BMP
//=============================
#define SEALEVELPRESSURE_HPA (1013.25)

//=============
//Declarations
//=============
double speed_x;
double speed_y;
double speed_z;

//============
// Servo Pins
//============
const uint8_t release_s_pin = 9; //idk what servo pin yet
const uint8_t port_s_pin = 9; //idk what servo pin yet
const uint8_t starboard_s_pin = 9; //idk what servo pin yet

//========
// states
//========
String sw_states[7] = {"LAUNCH_PAD", "ASCENT", "APOGEE", "DESCENT", "PROBE_RELEASE", "PAYLOAD_RELEASE", "LANDED"};
String sw_state = sw_states[0];

//========================
// telemetry declarations
//========================
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
String GPS_TIME;
long GPS_ALLTITUDE;
long GPS_LATITUDE;
long GPS_LONGITUDE;
byte GPS_SATS;
String CMD_ECHO;
double STRB_ANGLE;
double PORT_ANGLE;
double VECTOR_PRODUCT;

// IDK what this does but i think we need it
long last_time = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

//temporary
    double ALTITUDE = 0;
    double VELOCITY = 0;
    double peak_altitude = 0;
    double release = 100000000;


void heading_error() {


}

void collect_telemetry() {

if (millis() - last_time > 1000) // Waits 1 second 
{
  //==========
  // SAM-M10Q
  //==========
  GPS_LATITUDE = sam_m10q.getLatitude();
  GPS_LONGITUDE = sam_m10q.getLongitude();
  GPS_ALLTITUDE = sam_m10q.getAltitude();
  GPS_SATS = sam_m10q.getSIV();
  GPS_TIME = String(sam_m10q.getHour()) + ":" + String(sam_m10q.getMinute()) +  ":" + String(sam_m10q.getSecond());

  //========
  // BMP581
  //========
  sensors_event_t temp_event, pressure_event;
  TEMPERATURE = temp_event.temperature;
  PRESSURE = pressure_event.timestamp;

  //========
  // BNO085
  //========
}
}

void send_telemetry() {


}

void recive_command(){


}

void pro_restart() {


}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  release_s.attach(release_s_pin);
  port_s.attach(port_s_pin);
  starboard_s.attach(starboard_s_pin);

  // BMP581
  bmp_temp = bmp581.getTemperatureSensor();
  bmp_pressure = bmp581.getPressureSensor();

  bmp581.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp581.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp581.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp581.setOutputDataRate(BMP5XX_ODR_1_HZ);
  bmp581.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  //Print to terminal to show connection
  Serial.println("Intitalizing Metal_Hawk2a");

  // GPS
  sam_m10q.setI2COutput(COM_TYPE_UBX);
  

  //-----------
  // Debugging
  //-----------
  if (sam_m10q.begin() == false)
   {
    Serial.println("SAM-M10Q Has not connected. Check wires Hayden or Rudra");
    while (1);
   }
  if (!bmp581.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    Serial.println("The BMP581 is not being detected,. Check wires Hayden or Rudra");
    while (1);
  }
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
