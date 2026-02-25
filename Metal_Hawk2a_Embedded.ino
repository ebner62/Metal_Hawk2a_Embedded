#include <Wire.h>
#include <Arduino.h>
#include <SD.h> //SD card
#include <SPI.h> //SD card
#include <EEPROM.h> //EEPROM
#include <Adafruit_Sensor.h> //BMP581
#include "Adafruit_BMP5xx.h" //BMP581
#include <Adafruit_INA260.h> //INA260
#include <SparkFun_u-blox_GNSS_v3.h> // SAM-M10Q
#include <Adafruit_BNO08x.h> // BNO085
#include <PWMServo.h> // Servos ofc

//======
// Pins
//======

#define BNO08X_INT 19
#define BNO08X_RESET -1
const int ledpin = 13;

const int MMF_ADDR = 0; // The memory "slot" we will use (EEPROM)

//=========
// Sensors
//=========
Adafruit_BMP5xx bmp581; //Declaring BMP581
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode
SFE_UBLOX_GNSS sam_m10q; //Declaring SAM-M10Q
Adafruit_Sensor *bmp_temp = NULL;
Adafruit_Sensor *bmp_pressure = NULL;
Adafruit_BNO08x  bno085(BNO08X_RESET);
sh2_SensorValue_t sensorValue; //bno idk what we use it for
Adafruit_INA260 ina260 = Adafruit_INA260();


//=======
// Servo
//=======
PWMServo release_s; //Release Mechanism
PWMServo port_s; //Right Pull Mechanism
PWMServo starboard_s; //Left Pull Mechanism

const uint8_t release_s_pin = 25;
const uint8_t port_s_pin = 29; 
const uint8_t starboard_s_pin = 28; 

int probe_release = 160; //change, also for nose_engage
int probe_engage = 179; //change
int nose_release = 140; //change, also for egg_engage
int egg_release = 115; //change


//========================
// telemetry declarations
//========================
String TELEMETRY;
int TEAM_ID = 1094; // Done
char MISSION_TIME[15]; // Done it is = to GPS_TIME
int PACKET_COUNT = 0; // Done
char MODE = 'F'; //F or S
String sw_state = "LAUNCH_PAD"; // Done
float ALTITUDE; // Done
float TEMPERATURE; // Done
float PRESSURE; // Done
float VOLTAGE; // Done
float CURRENT; // Done
float GYRO_R; // Done
float GYRO_P; // Done
float GYRO_Y; // Done
float ACCEL_R; // Done
float ACCEL_P; // Done
float ACCEL_Y; // Done
char GPS_TIME[15]; // Done
double GPS_ALTITUDE; // Done
double GPS_LATITUDE; // Done
double GPS_LONGITUDE; // Done
int GPS_SATS; // Done
char ECHO[64];
double STRB_ANGLE;
double PORT_ANGLE;
double VECTOR_PRODUCT;

//================================
// Very importatatnt declarations
//================================
float velocity = 0;
float apogee;
float release;
bool nose_fired = false;
bool probe_fired = false;
bool egg_fired = false;

//=======================
// Required Declarations
//=======================
long last_time = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
float lastVelR = 0, lastVelP = 0, lastVelY = 0, last_altitude = 0;
unsigned long bno_last_micros = 0;
unsigned long last_vel_time = 0;
char rx_buffer[50]; // Command handler
int rx_index = 0; // Command handler
float SEALEVELPRESSURE_HPA  = (1013.25); //bmp
unsigned long time_offset = 0;
uint32_t total_seconds;
int hours;
int minutes;
int seconds;

//==========
// Commands
//==========
bool CX = false;
bool SIM_ENABLE = false;
bool SIM_ACTIVATE = false;
bool MMF = false;

void steering() {
}

void collect_telemetry() {
  //======
  // TIME
  //======
  total_seconds = (millis() / 1000) + time_offset;

  total_seconds %= 86400; //prevents error at 24:00

  hours = total_seconds / 3600;
  minutes = (total_seconds % 3600) / 60;
  seconds = total_seconds % 60;

  sprintf(MISSION_TIME, "%02d:%02d:%02d", hours, minutes, seconds);

  //========
  // BNO085
  //========
  if (bno085.getSensorEvent(&sensorValue)){
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      unsigned long currentMicros = micros();
      float dt = (currentMicros - bno_last_micros) / 1000000.0;

      if (dt > 0) {
        // Get current velocity in deg/s
        float currVelP = sensorValue.un.gyroscope.x * 57.2958;
        float currVelR = sensorValue.un.gyroscope.y * 57.2958;
        float currVelY = sensorValue.un.gyroscope.z * 57.2958;

        // Calculate ACCEL (deg/s^2)
        ACCEL_P = (currVelP - lastVelP) / dt;
        ACCEL_R = (currVelR - lastVelR) / dt;
        ACCEL_Y = (currVelY - lastVelY) / dt;

        // Store current for next calculation
        lastVelP = currVelP;
        lastVelR = currVelR;
        lastVelY = currVelY;
        bno_last_micros = currentMicros;

        // Update your Telemetry Gyro vars (current velocity)
        GYRO_P = currVelP;
        GYRO_R = currVelR;
        GYRO_Y = currVelY;
      }
    }
  }
  if (millis() - last_telem_time > 100) {// Waits 0.1 second
    last_telem_time = millis();

    //==========
    // SAM-M10Q
    //==========

    GPS_LATITUDE = sam_m10q.getLatitude() / 10000000.0;
    GPS_LONGITUDE = sam_m10q.getLongitude() / 10000000.0;
    GPS_ALTITUDE = sam_m10q.getAltitudeMSL() / 1000.0;
    GPS_SATS = sam_m10q.getSIV();
    sprintf(GPS_TIME, "%02d:%02d:%02d", sam_m10q.getHour(), sam_m10q.getMinute(), sam_m10q.getSecond());

    //========
    // BMP581
    //========

    sensors_event_t temp_event;
    bmp_temp->getEvent(&temp_event);

    TEMPERATURE = temp_event.temperature;

    if(!SIM_ENABLE || !SIM_ACTIVATE) {
      sensors_event_t pressure_event;
      bmp_pressure->getEvent(&pressure_event);

      PRESSURE = pressure_event.pressure;
      // Altitude filter
      float raw_altitude = 44330 * (1.0 - pow(PRESSURE / SEALEVELPRESSURE_HPA, 0.1903));
      ALTITUDE = (ALTITUDE * 0.7) + (ALTITUDE * 0.3)
    }


    //========
    // INA260
    //========

    CURRENT = (ina260.readCurrent() / 1000.0);
    VOLTAGE = ina260.readBusVoltage();

    //==========
    // velocity
    //==========
    unsigned long current_vel_millis = millis();
    float dt_vel = (current_vel_millis - last_vel_time) / 1000.0;

    if (dt_vel > 0){
      float instant_velocity = (ALTITUDE - last_altitude) / dt_vel;

      velocity = (velocity * 0.8) + (instant_velocity * 0.2);

      last_altitude = ALTITUDE;
    }
  }

}





void send_telemetry() {
  if (millis() - last_time > 1000) {// Waits 1 second
    char tel_buffer[250]; //I need to change buffer amount

    //Telemetry string========================================================
    sprintf(tel_buffer, "%d,%s,%d,%c,%s,%.1f,%.1f,%.1f,%.1f,%.2f,%f,%f,%f,%f,%f,%f,%s,%.1f,%.4f,%.4f,%d,%s", 
            TEAM_ID, 
            MISSION_TIME, 
            PACKET_COUNT, 
            MODE, 
            sw_state.c_str(), 
            ALTITUDE,
            TEMPERATURE,
            PRESSURE,
            VOLTAGE,
            CURRENT,
            GYRO_R,
            GYRO_P,
            GYRO_Y,
            ACCEL_R,
            ACCEL_P,
            ACCEL_Y,
            GPS_TIME,
            GPS_ALTITUDE,
            GPS_LATITUDE,
            GPS_LONGITUDE,
            GPS_SATS,
            ECHO
            );

    //========================================================================

    TELEMETRY = String(tel_buffer);
    Serial8.print(TELEMETRY);
    PACKET_COUNT += 1;
  }
}

void recive_command(){
  if (Serial8.available() > 0) {
    char c = Serial8.read();

    if (c == '\n'){
      if (rx_index > 0) {
        rx_buffer[rx_index] = '\0';

        //====== Gemini added this =============================================================
        while(rx_index > 0 && (rx_buffer[rx_index-1] == '\r' || rx_buffer[rx_index-1] == ' ')) {
          rx_buffer[--rx_index] = '\0';
        }
        //======================================================================================
        //ECHO
        strncpy(ECHO, rx_buffer, sizeof(ECHO) -  1);
        handleCommand(rx_buffer);
        rx_index = 0;
      }
    }
    else if (rx_index < 49) {
      rx_buffer[rx_index++] = c;
    }
  }
}

void handleCommand(char* message){

  char temp[64];
  strncpy(temp, message, sizeof(temp) - 1); //make a copy of message into temp bc strtok changes
  temp[sizeof(temp) - 1] = '\0';
  

  char* header = strtok(temp, ","); // CMD
  char* teamID = strtok(NULL, ","); // 1094
  char* type = strtok(NULL, ","); // family of command
  char* state = strtok(NULL, ","); // specific command

  //==================
  // Testing Commands
  //==================
  if (header != NULL && strcmp(header, "CMD") == 0) { //strcmp == 0 when its the same
    if (teamID != NULL && strcmp(teamID, "1094") == 0) {
      /*====*/
      /* CX */
      /*====*/
      if (type != NULL && strcmp(type, "CX") == 0) {
        if (state != NULL && strcmp(state, "ON") == 0) {
          CX = true;
        }
        else if (state != NULL && strcmp(state, "OFF") == 0) {
          CX = false;
        } 
      }
      /*====*/
      /* ST */
      /*====*/
      else if (type != NULL && strcmp(type, "ST") == 0) {
        if (state != NULL && strcmp(state, "GPS") == 0) {
          uint32_t gps_total_seconds = (sam_m10q.getHour() * 3600UL) + (sam_m10q.getMinute() * 60UL) + sam_m10q.getSecond();
          time_offset = gps_total_seconds - (millis() / 1000);
        }
      }
        else if (state != NULL) {
          int h, m, s;
          if (sscanf(state, "%d:%d:%d", &h, &m, &s) == 3) {
            uint32_t sync_seconds = (h * 3600UL) + (m * 60UL) + s;
            time_offset = sync_seconds - (millis() / 1000);
          }
      }
      /*=====*/
      /* SIM */
      /*=====*/
      else if (type != NULL && strcmp(type, "SIM") == 0) {
        if (state != NULL && strcmp(state, "ENABLE") == 0) {
          SIM_ENABLE = true;
          }
        else if (state != NULL && strcmp(state, "ACTIVATE") == 0) {
          SIM_ACTIVATE = true;
        }
        else if (state != NULL && strcmp(state, "DISABLE") == 0) {
          SIM_ENABLE = false;
          SIM_ACTIVATE = false;
        }
      }
      /*======*/
      /* SIMP */
      /*======*/
      else if (type != NULL && strcmp(type, "SIMP") == 0) {
        if ((SIM_ENABLE && SIM_ACTIVATE) == 1) {
          PRESSURE = atof(state);
        }
      }
      /*=====*/
      /* CAL */
      /*=====*/
      else if (type != NULL && strcmp(type, "CAL") == 0) {
        ALTITUDE = 0;
        SEALEVELPRESSURE_HPA = PRESSURE;
      }
      /*=====*/
      /* MEC */
      /*=====*/
      else if (type != NULL && strcmp(type, "MEC") == 0) {
        if (state != NULL && strcmp(state, "REL") == 0){
          release_s.write(probe_release);
        }
        if (state != NULL && strcmp(state, "ENG") == 0) {
          release_s.write(probe_engage);
        }
      }
      /*=======*/
      /* NOSE */ 
      /*=======*/
      else if (type != NULL && strcmp(type, "NOSE") == 0) {
        if (state != NULL && strcmp(state, "REL") == 0) {
          release_s.write(nose_release);
        }
        else if (state != NULL && strcmp(state, "ENG") == 0) {
          release_s.write(probe_release);
        }
      }
      /*=====*/
      /* EGG */
      /*=====*/
      else if (type != NULL && strcmp(type, "EGG") == 0) {
        if (state != NULL && strcmp(state, "REL") == 0) {
          release_s.write(egg_release);
        }
        else if (state != NULL && strcmp(state, "ENG") == 0) {
          release_s.write(nose_release);
        }
      }
      /*=====*/
      /* LED */
      /*=====*/
      else if (type != NULL && strcmp(type, "LED") == 0) {
        digitalWrite(ledpin, !digitalRead(ledpin));
      }
      /*======*/
      /* PORT */
      /*======*/
      else if ((type != NULL && strcmp(type, "PORT") == 0)) {
        if (type != NULL) {
          port_s.write(atoi(state));
        }
      }
      /*===========*/
      /* STARBOARD */
      /*===========*/
      else if ((type != NULL && strcmp(type, "STRB") == 0)) {
        if (state !=NULL) {
          starboard_s.write(atoi(state));
        }
      }
      /*=====*/
      /* MMF */
      /*=====*/
      else if ((type != NULL && strcmp(type, "MMF") == 0)) {
        if (state != NULL && strcmp(state, "TRUE") == 0) {
          MMF = true;
          EEPROM.write(MMF_ADDR, 1);
        } 
        else if (state != NULL && strcmp(state, "FALSE") == 0) {
          MMF = false;
          EEPROM.write(MMF_ADDR, 0);
        }
      }
          
    }  
  }
}

void setup() {
  Serial8.begin(9600);
  Wire1.begin();
  release_s.attach(release_s_pin);
  port_s.attach(port_s_pin);
  starboard_s.attach(starboard_s_pin);
  pinMode(ledpin, OUTPUT);

  //========
  // BMP581
  //========
  bmp_temp = bmp581.getTemperatureSensor();
  bmp_pressure = bmp581.getPressureSensor();

  bmp581.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp581.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp581.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp581.setOutputDataRate(BMP5XX_ODR_15_HZ);
  bmp581.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  //======================================
  // Print to terminal to show connection
  //======================================
  Serial.println("Intitalizing Metal_Hawk2a");

  //=====
  // GPS
  //=====
  sam_m10q.setI2COutput(COM_TYPE_UBX);

  //========
  // BNO085
  //========
  //bno085.enableReport(SH2_ROTATION_VECTOR);
  //bno085.enableReport(SH2_ACCELEROMETER);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno_last_micros = micros();

  //===========
  // Debugging
  //===========
  if (sam_m10q.begin() == false)
   {
    Serial8.println("SAM-M10Q Has not connected. Check wires Hay Hay");
    while (1);
   }
  if (!bmp581.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire1)) {
    Serial8.println("The BMP581 is not being detected,. Check wires Hayden or Rudra");
    while (1);
  }
  if (!bno085.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1, BNO08X_INT)) { // Added &Wire1
    Serial8.println("BNO085 not found on Wire1");
  while (1);
  }
  if (!ina260.begin()) {
    Serial8.println("Rudra ai the ina260 is not detected");
    while (1);
  }
 
}


void loop() {
  collect_telemetry();
  recive_command();
  send_telemetry();

  if(sw_state == "LAUNCH_PAD"){
    if(ALTITUDE >= 20 && velocity >=10){
      sw_state = "ASCENT";
    }
  }

  else if(sw_state == "ASCENT"){
    if(ALTITUDE > apogee){
      apogee = ALTITUDE;
    }
    if(ALTITUDE < (apogee - 5.0)){
      sw_state = "DESCENT";
      release = apogee * 0.8;
    }
  }

  else if(sw_state == "DESCENT"){
    steering();
    if (ALTITUDE <= (apogee * 0.7) && !nose_fired) {
      release_s.write(nose_release);
      nose_fired = true;
    }
    if(ALTITUDE <= release){
      sw_state = "PROBE_RELEASE";
      release_s.write(probe_release);
      probe_fired = true;
    }
  }

  else if(sw_state == "PROBE_RELEASE"){
    if(ALTITUDE <= 2.0){
      sw_state = "PAYLOAD_RELEASE";
      release_s.write(egg_release);
      egg_fired = true;
    }
  }

  else if(sw_state == "PAYLOAD_RELEASE"){
    if(velocity <= 0.2){
      sw_state = "LANDED";
    }
  }

}
