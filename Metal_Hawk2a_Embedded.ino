#include <Wire.h>
#include <Arduino.h>
#include <SD.h> //SD card
#include <SPI.h> //SD card
#include <EEPROM.h> //EEPROM
#include <Adafruit_Sensor.h> //BMP581
#include "Adafruit_BMP5xx.h" //BMP581
#include <Adafruit_INA260.h> //INA260
#include <SparkFun_u-blox_GNSS_v3.h> // SAM-M10Q
#include <Adafruit_BNO08x.h>  // BNO085
#include <Servo.h> // Servos ofc
#include <math.h> // for steering
#include <SerialPIO.h> // creates uarts
#include <cmath> 

//======
// Pins
//======

// BNO085: no physical INT or RESET pin connected
const int ledpin = LED_BUILTIN;
const byte scl_one = 27;
const byte sda_one = 26;
const byte scl_zero = 9;
const byte sda_zero = 8;
SerialPIO ground_cam_serial(14, 15); //change pins

//****************************************//

const int MMF_ADDR = 0; // The memory "slot" we will use (EEPROM)

const int chipSelect = 17; //SD card

//=========
// Sensors
//=========
Adafruit_BMP5xx bmp581; //Declaring BMP581
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode
SFE_UBLOX_GNSS sam_m10q; //Declaring SAM-M10Q
Adafruit_Sensor *bmp_temp = NULL;
Adafruit_Sensor *bmp_pressure = NULL; 
Adafruit_BNO08x  bno085;                                     // Declaring BNO085 (I2C, no INT/RESET pins)
sh2_SensorValue_t bno085_value;                              // Sensor report value
Adafruit_INA260 ina260 = Adafruit_INA260();


//=======
// Servo
//=======
Servo release_s; //Release Mechanism
Servo port_s; //Right Pull Mechanism
Servo starboard_s; //Left Pull Mechanism

const uint8_t release_s_pin = 2;
const uint8_t port_s_pin = 1; 
const uint8_t starboard_s_pin = 0; 

int probe_release = 70; //change
int probe_engage = 140; //Max
int nose_release = 25; //change
int nose_engage = 65; //change
int egg_release = 2; //Min
int egg_engage = 20; //change

//==================
// TARGET LOCATIONS
//==================
const double target_lat_one = 00.000; // FILL IN LATER
const double target_lon_one = 00.000; // FILL IN LATER
bool target_one_reached = false;

const double target_lat_two = 00.000; // FILL IN LATER
const double target_lon_two = 00.000; // FILL IN LATER
bool target_two_reached = false;



//========================
// telemetry declarations
//========================
String TELEMETRY;
int TEAM_ID = 1094; // Done
char MISSION_TIME[15]; // Done it is = to GPS_TIME
int PACKET_COUNT = 1; // Done
char MODE = 'F'; //F or S
String sw_state = "LAUNCH_PAD"; // Done
float ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT; // Done
float GYRO_R, GYRO_P, GYRO_Y;
float ACCEL_R, ACCEL_P, ACCEL_Y; // Done
char GPS_TIME[15]; // Done
double GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE; // Done
int GPS_SATS; // Done
char ECHO[128]; // Done
double STRB_ANGLE = 0.0;
double PORT_ANGLE = 0.0;
double VECTOR_PRODUCT = 0.0;

//================================
// Very importatatnt declarations
//================================
float velocity = 0;
float apogee = 0;
float release;

bool nose_fired = false, probe_fired = false, egg_fired = false;

bool gps_online = false, bmp_online = false, ina_online = false, bno_online = false, sd_online = false;

float last_heading = 0;
float current_heading = 0;

float ground_altitude = 0;
float ground_pressure = 0;
float raw_hPa = 0;
float sum_pressure = 0;

//============Rudra===========
// --- Bezier & PID Globals ---
const int RADIUS = 6371000;
struct Point2D { double x; double y; };

// Target/Gate Coordinates (Radians) - Calculated once in setup
double TARGET_LAT_RAD, TARGET_LON_RAD;
double GATE1_LAT_RAD, GATE1_LON_RAD, GATE2_LAT_RAD, GATE2_LON_RAD;

// Navigation Points in Meters
Point2D p0, p1, p2; 
Point2D gate1_m, gate2_m;
bool curve_generated = false;

struct PIDController {
    double Kp, Ki, Kd;
    double integral_sum = 0, last_error = 0, last_derivative = 0;
    unsigned long last_time = 0;
    double filter_alpha = 0.5;

    PIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}

    double compute(double error) {
        if (std::abs(error) < 0.05) error = 0;
        unsigned long now = millis();
        if (last_time == 0) { last_time = now; last_error = error; return 0; }
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0) return 0;

        integral_sum = constrain(integral_sum + (error * dt), -1.0, 1.0);
        double raw_der = (error - last_error) / dt;
        double filtered_der = (filter_alpha * raw_der) + ((1 - filter_alpha) * last_derivative);
        
        last_error = error; last_time = now; last_derivative = filtered_der;
        return (Kp * error) + (Ki * integral_sum) + (Kd * filtered_der);
    }
};

PIDController steeringPID(400.0, 0.0, 50.0); // Adjust Kp/Kd based on servo response
//============================

//=======================
// Required Declarations
//=======================
long last_time = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
float lastVelR = 0, lastVelP = 0, lastVelY = 0, last_altitude = 0;
unsigned long bno_last_micros = 0;
unsigned long last_telem_time = 0;
unsigned long last_vel_time = 0;
char rx_buffer[128]; // Command handler
int rx_index = 0; // Command handler
unsigned long time_offset = 0;
uint32_t total_seconds;
int hours;
int minutes;
int seconds;
int apogee_counter = 0;

double lat_rad;
double lon_rad;

//==========
// Commands
//==========
bool CX = false;
bool SIM_ENABLE = false;
bool SIM_ACTIVATE = false;
bool MMF = false;

//=========
// Cameras
//=========

uint8_t cmd_ShortPress[] = {0xCC, 0x01, 0x01, 0xD9}; // Start/Stop Record
uint8_t cmd_LongPress[]  = {0xCC, 0x01, 0x02, 0x72}; // Power On/Off Toggle

void setup() {
  Serial.begin(9600);
  delay(500);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));
  
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(9600);
  pinMode(ledpin, OUTPUT);
  
  ground_cam_serial.begin(115200);

  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200); //Release mech cam

  Wire.setSDA(sda_zero);
  Wire.setSCL(scl_zero);
  Wire.begin();
  Wire.setClock(100000); // Start slow for stability
  delay(100); // Give the bus a moment to stabilize;

  Wire1.setSDA(sda_one);
  Wire1.setSCL(scl_one);
  Wire1.begin();
  Wire1.setClock(100000);

  SPI.setRX(16);
  SPI.setTX(19);
  SPI.setSCK(18);

  release_s.attach(release_s_pin);
  port_s.attach(port_s_pin);
  starboard_s.attach(starboard_s_pin);
  delay(100);

  //==========
  // BMP & SD
  //==========

  bmp_online = bmp581.begin(0x46, &Wire);
  if (!bmp_online) bmp_online = bmp581.begin(0x47, &Wire);
  sd_online = SD.begin(chipSelect);

  if !(bmp_online) Serial.println("BMP not online");
  

  //=================
  // GPS & INA & BNO
  //=================
  bno_online = bno085.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1);
  gps_online = sam_m10q.begin(Wire1, 0x42);
  if (!gps_online) {
    delay(500);
    gps_online = sam_m10q.begin(Wire1, 0x42); // Second attempt
  }
  ina_online = ina260.begin(0x40, &Wire1);
  if (bno_online) {
    // Enable Rotation Vector (fused heading) and Gyroscope reports
    bno085.enableReport(SH2_ROTATION_VECTOR, 10000);  // 10 ms = 100 Hz
    bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno_last_micros = micros();
  }

  if (!gps_online) {
    Serial.println("GPS is not connected");}
  if (!ina_online) {
    Serial.println("INA is not connected");}
  if (!bno_online) {
    Serial.println("BNO is not connected");}

  //==================
  // MID MISSION FLAG
  //==================
  MMF = EEPROM.read(MMF_ADDR);
  if (MMF == true){                
    if (sd_online) {                
      File stateFile = SD.open("state.txt", FILE_READ);                
      if (stateFile) {                
        String savedData = stateFile.readString();                
        stateFile.close();                
    
        if (savedData.length() > 0) {                
          // Parse "sw_state,ground_pressure,ground_altitude,apogee"
          int comma1 = savedData.indexOf(',');                
          int comma2 = savedData.indexOf(',', comma1 + 1);                
          int comma3 = savedData.indexOf(',', comma2 + 1);                
      
          if (comma1 != -1 && comma2 != -1 && comma3 != -1) {                
            sw_state = savedData.substring(0, comma1);                
            ground_pressure = savedData.substring(comma1 + 1, comma2).toFloat();                
            ground_altitude = savedData.substring(comma2 + 1, comma3).toFloat();                
            apogee = savedData.substring(comma3 + 1).toFloat(); 
          }                
        }                
      }                
    }                
  }

  //======================================
  // Print to terminal to show connection
  //======================================
  Serial.println("Intitalizing Metal_Hawk2a");

  //========
  // BMP581
  //========
  if (bmp_online){
    bmp_temp = bmp581.getTemperatureSensor();
    bmp_pressure = bmp581.getPressureSensor();
    //
    Serial.print("BMP init: "); Serial.println(bmp_online ? "SUCCESS" : "FAILED");

    bmp581.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp581.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    //bmp581.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    //bmp581.setOutputDataRate(BMP5XX_ODR_50_HZ);
    //bmp581.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    float sum_alt = 0;
    sum_pressure = 0;

    //-----------------
    // Ground Altitude
    //-----------------
    for(int i = 0; i < 20; i++) {
      if (bmp581.performReading()) {
        float current_p = bmp581.pressure / 100.0; // Convert Pa to hPa
        sum_pressure += current_p;
        sum_alt += 44330.0 * (1.0 - pow(current_p / 1013.25, 0.1903));
      }
      delay(50);
    }
    
    ground_altitude = 0;
    ground_pressure = sum_pressure / 20.0;
  }

  //=====
  // GPS
  //=====
  if (gps_online){
    sam_m10q.setI2COutput(COM_TYPE_UBX);
    sam_m10q.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  }

  //========
  // BNO085
  //========
  if (bmp_online){
    bmp_temp = bmp581.getTemperatureSensor();
    bmp_pressure = bmp581.getPressureSensor();
    bmp581.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp581.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    ///
    Serial.print("BMP init: "); Serial.println(bmp_online ? "SUCCESS" : "FAILED");
    Serial.print("BNO085 init: "); Serial.println(bno_online ? "SUCCESS" : "FAILED");
    ///
  }

  //====================================================================Rudra===============
  // Initialize competition coordinates
  TARGET_LAT_RAD = dmsToRadians(31, 7, 20.9);
  TARGET_LON_RAD = -dmsToRadians(86, 5, 33.02);
  GATE1_LAT_RAD = dmsToRadians(31, 7, 21.05);
  GATE1_LON_RAD = -dmsToRadians(86, 5, 29.28);
  GATE2_LAT_RAD = dmsToRadians(31, 7, 20.86);
  GATE2_LON_RAD = -dmsToRadians(86, 5, 36.82);

  // Set p2 (Target) as the origin (0,0)
  p2 = {0, 0};
  gate1_m = gps_to_meters(GATE1_LAT_RAD, GATE1_LON_RAD, TARGET_LAT_RAD, TARGET_LON_RAD);
  gate2_m = gps_to_meters(GATE2_LAT_RAD, GATE2_LON_RAD, TARGET_LAT_RAD, TARGET_LON_RAD);
}


void loop() {
  collect_telemetry();
  receive_command();
  unsigned long currentMillis = millis();
  if (CX == true){
    send_telemetry();
  }

  if(sw_state == "LAUNCH_PAD"){
    if(ALTITUDE >= 20 && velocity >=10){
      sw_state = "ASCENT";
      save_flight_state();
    }
  }
  else if(sw_state == "ASCENT"){
    if(ALTITUDE > apogee){
      apogee = ALTITUDE;
    }
    if((ALTITUDE < apogee - 5.0 ) && (velocity < -1.0)){
      apogee_counter += 1;
      if(apogee_counter >= 3){
        sw_state = "APOGEE";
        save_flight_state();
      }
    }
    else {apogee_counter = 0;}
  }
  else if(sw_state == "APOGEE"){
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      sw_state = "DESCENT";
    }
  }
  else if(sw_state == "DESCENT"){
    if(ALTITUDE <= apogee * 0.8 && velocity < -1.0){
      sw_state = "PROBE_RELEASE";
      release_s.write(probe_release);
      probe_fired = true;
      save_flight_state();
    }
  }
  else if(sw_state == "PROBE_RELEASE"){

    if (ALTITUDE <= (apogee * 0.7) && !nose_fired) {
      release_s.write(nose_release);
      nose_fired = true;
    }
    if(ALTITUDE <= 2.0){
      sw_state = "PAYLOAD_RELEASE";
      release_s.write(egg_release);
      egg_fired = true;
      save_flight_state();
    }
    if (!curve_generated && gps_online) {
      p0 = gps_to_meters(lat_rad, lon_rad, TARGET_LAT_RAD, TARGET_LON_RAD);
      
      // Dynamic Gate Selection: Choose the gate closest to our release point
      double d1 = sqrt(pow(p0.x - gate1_m.x, 2) + pow(p0.y - gate1_m.y, 2));
      double d2 = sqrt(pow(p0.x - gate2_m.x, 2) + pow(p0.y - gate2_m.y, 2));
      p1 = (d1 < d2) ? gate1_m : gate2_m;
      
      curve_generated = true;
    }

    // 2. Active Steering Logic
    if (curve_generated) {
      Point2D current_pos = gps_to_meters(lat_rad, lon_rad, TARGET_LAT_RAD, TARGET_LON_RAD);
      double heading_rad = current_heading * (M_PI / 180.0);
      
      // Get error and compute PID correction
      double error = calculate_steering_error(current_pos, heading_rad, p0, p1, p2);
      double correction = steeringPID.compute(error);

      // Map to servos (90 is neutral)
      int p_out = 90 + (int)correction;
      int s_out = 90 - (int)correction;

      port_s.write(constrain(p_out, 45, 135));
      starboard_s.write(constrain(s_out, 45, 135));

      // Update telemetry variables
      VECTOR_PRODUCT = error;
      PORT_ANGLE = p_out;
      STRB_ANGLE = s_out;
    }
  }
  else if(sw_state == "PAYLOAD_RELEASE"){
    if((abs(velocity) <= 0.2) && (ALTITUDE  < 5.0)){
      sw_state = "LANDED";
      release_s.write(probe_engage);
      save_flight_state();
    }
  }
  else if(sw_state == "LANDED"){
    digitalWrite(ledpin, (millis() / 500) % 2);
    if ((millis() / 1000) % 2 == 0) {
        
    }
  }
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
  if (bno_online && bno085.getSensorEvent(&bno085_value)) {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - bno_last_micros) / 1000000.0;

    if (bno085_value.sensorId == SH2_GYROSCOPE_CALIBRATED && dt > 0) {
      float currVelP = bno085_value.un.gyroscope.x * RAD_TO_DEG; // Pitch rate (rad/s → deg/s)
      float currVelR = bno085_value.un.gyroscope.y * RAD_TO_DEG; // Roll rate
      float currVelY = bno085_value.un.gyroscope.z * RAD_TO_DEG; // Yaw rate

      // Calculate ACCEL (deg/s^2)
      ACCEL_P = (currVelP - lastVelP) / dt;
      ACCEL_R = (currVelR - lastVelR) / dt;
      ACCEL_Y = (currVelY - lastVelY) / dt;

      lastVelP = currVelP;
      lastVelR = currVelR;
      lastVelY = currVelY;
      bno_last_micros = currentMicros;

      GYRO_P = currVelP;
      GYRO_R = currVelR;
      GYRO_Y = currVelY;
    }

    if (bno085_value.sensorId == SH2_ROTATION_VECTOR) {
      // Convert quaternion to yaw (heading) in degrees
      float qr = bno085_value.un.rotationVector.real;
      float qi = bno085_value.un.rotationVector.i;
      float qj = bno085_value.un.rotationVector.j;
      float qk = bno085_value.un.rotationVector.k;

      // Yaw from quaternion (NED convention)
      float yaw = atan2(2.0f * (qr * qk + qi * qj),
                        1.0f - 2.0f * (qj * qj + qk * qk));
      float heading = yaw * RAD_TO_DEG;
      if (heading < 0) heading += 360.0f;

      last_heading    = current_heading;
      current_heading = heading;
    }
  }
  

  if (millis() - last_telem_time > 100) {// Waits 0.1 second
    last_telem_time = millis();

    //==========
    // SAM-M10Q
    //==========
    if (gps_online && sam_m10q.getPVT()){
      GPS_LATITUDE = sam_m10q.getLatitude() / 10000000.0;
      GPS_LONGITUDE = sam_m10q.getLongitude() / 10000000.0;
      GPS_ALTITUDE = sam_m10q.getAltitudeMSL() / 1000.0;
      GPS_SATS = sam_m10q.getSIV();
      sprintf(GPS_TIME, "%02d:%02d:%02d", sam_m10q.getHour(), sam_m10q.getMinute(), sam_m10q.getSecond());

      lat_rad = (GPS_LATITUDE * M_PI) / 180.0;
      lon_rad = (GPS_LONGITUDE * M_PI) / 180.0;
    }

    //========
    // BMP581
    //========

    if (bmp_online){
      if (!(SIM_ENABLE && SIM_ACTIVATE)) {
        if (bmp581.performReading()) {
          TEMPERATURE = bmp581.temperature;
          raw_hPa = bmp581.pressure / 100.0; 
          PRESSURE = raw_hPa / 10.0; 
        }
      } 
      else {
        raw_hPa = PRESSURE * 10.0;
      }
    }
    // Altitude filter
    if (raw_hPa > 0 && ground_pressure > 0) {
      float current_rel_alt = 44330.0 * (pow(ground_pressure / raw_hPa, 0.1903) - 1.0);

      if (!isnan(current_rel_alt) && !isinf(current_rel_alt)){
        ALTITUDE = (ALTITUDE * 0.7) + (current_rel_alt * 0.3);
      }
    }

    //========
    // INA260
    //========
    if (ina_online){
      CURRENT = (ina260.readCurrent() / 1000.0);
      VOLTAGE = (ina260.readBusVoltage() / 1000.0);
    }

    //==========
    // velocity
    //==========
    unsigned long current_vel_millis = millis();
    float dt_vel = (current_vel_millis - last_vel_time) / 1000.0;

    if (dt_vel > 0 && !isnan(ALTITUDE)){
      float instant_velocity = (ALTITUDE - last_altitude) / dt_vel;

      if(!isnan(instant_velocity) && !isinf(instant_velocity)) {
        velocity = (velocity * 0.8) + (instant_velocity * 0.2);
        last_altitude = ALTITUDE;
        last_vel_time = current_vel_millis;
      }
    }
  }

}





void send_telemetry() {
  if (millis() - last_time > 1000) {// Waits 1 second
    last_time = millis();
    char tel_buffer[800]; //I need to change buffer amount

    //Telemetry string========================================================
    sprintf(tel_buffer, "%d,%s,%d,%c,%s,%.1f,%.1f,%.1f,%.1f,%.2f,%f,%f,%f,%f,%f,%f,%s,%.1f,%.4f,%.4f,%d,%s,,%.1f,%.1f,%.1f", 
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
            ECHO,
            STRB_ANGLE,
            PORT_ANGLE,
            VECTOR_PRODUCT
            );

    //========================================================================

    TELEMETRY = String(tel_buffer);
    Serial1.print(TELEMETRY);
    Serial1.print('\r');
    PACKET_COUNT += 1;

    //REMOVE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Serial.println(TELEMETRY);
    //REMOVE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    if (sd_online) {
      File logFile = SD.open("flight.csv", FILE_WRITE);
      if (logFile) {
        logFile.print(TELEMETRY);
        logFile.print('\r');
        logFile.close();
      }
    }
  }
}

void receive_command(){
  if (Serial1.available() > 0) {
    char c = Serial1.read();

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
        ECHO[sizeof(ECHO) - 1] = '\0';

        for (size_t i = 0; i < sizeof(ECHO); i++) {
          if (ECHO[i] == '\0') break;
          if (ECHO[i] == ',') ECHO[i] = '_';
        }
        handleCommand(rx_buffer);
        rx_index = 0;
        
      }
    }
    else if (rx_index < 127) {
      rx_buffer[rx_index++] = c;
    }
  }
}

void handleCommand(char* message){

  char temp[128];
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
      
        else if (state != NULL) {
          int h, m, s;
          if (sscanf(state, "%d:%d:%d", &h, &m, &s) == 3) {
            uint32_t sync_seconds = (h * 3600UL) + (m * 60UL) + s;
            time_offset = sync_seconds - (millis() / 1000);
          }
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
        if (SIM_ENABLE && SIM_ACTIVATE) {
          PRESSURE = atof(state) / 1000.0;
          raw_hPa = PRESSURE * 10.0;
          if (ground_pressure == 0){
            ground_pressure = raw_hPa;
            ground_altitude = 44330.0 * (1.0 - pow(ground_pressure / 1013.25, 0.1903));
          }
        }
      }
      /*=====*/
      /* CAL */
      /*=====*/
      else if (type != NULL && strcmp(type, "CAL") == 0) {
        ground_pressure = raw_hPa;
        ground_altitude = 0;
        ALTITUDE = 0;
        last_altitude = 0;
        save_flight_state();
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
          release_s.write(nose_engage);
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
          release_s.write(egg_engage);
        }
      }
      /*=====*/
      /* LED */
      /*=====*/
      else if (type != NULL && strcmp(type, "LED") == 0) {
        digitalWrite(ledpin, !digitalRead(ledpin));
      }
      /*=====*/
      /* MMF */
      /*=====*/
      else if ((type != NULL && strcmp(type, "MMF") == 0)) {
        if (state != NULL && strcmp(state, "TRUE") == 0) {
          MMF = true;
          EEPROM.write(MMF_ADDR, 1);
          save_flight_state();
        } 
        else if (state != NULL && strcmp(state, "FALSE") == 0) {
          MMF = false;
          EEPROM.write(MMF_ADDR, 0);
          sw_state = "LAUNCH_PAD";
        }
      }
      /*======*/
      /* GCAM */
      /*======*/
      else if ((type != NULL && strcmp(type, "GCAM") == 0)) {
        if (state != NULL && strcmp(state, "ON") == 0) {
          power_g_cam();
        }
        else if (state != NULL && strcmp(state, "RECORD") == 0){
          record_g_cam(true);
        }
      }
      /*======*/
      /* DCAM */
      /*======*/
      else if ((type != NULL && strcmp(type, "DCAM") == 0)) {
        if (state != NULL && strcmp(state, "ON") == 0) {
          power_d_cam();
        }
        else if (state != NULL && strcmp(state, "RECORD") == 0) {
          record_d_cam(true);
        }
      }
    }  
  }
}

void save_flight_state() {
  if (sd_online) {
    SD.remove("state.txt");
    File stateFile = SD.open("state.txt", FILE_WRITE);
    if (stateFile) {
      // Save State, Cal Data, and Apogee (4 items)
      stateFile.print(sw_state + ",");
      stateFile.print(String(ground_pressure) + ",");
      stateFile.print(String(ground_altitude) + ",");
      stateFile.print(String(apogee));
      stateFile.close();
      Serial.println("Saved Flight State & Calibration");
    }
  }
}

void record_d_cam(bool start) { //Deployment
  Serial2.write(cmd_ShortPress, sizeof(cmd_ShortPress));
}

void record_g_cam(bool start) { //Ground
  ground_cam_serial.write(cmd_ShortPress, sizeof(cmd_ShortPress));
}

void power_d_cam() {
  Serial2.write(cmd_LongPress, sizeof(cmd_LongPress));
}

void power_g_cam() {
  ground_cam_serial.write(cmd_LongPress, sizeof(cmd_LongPress));
}

//============================================================Rudra======================

Point2D gps_to_meters(double lat_rad, double lon_rad, double ref_lat, double ref_lon) { 
    Point2D pos;
    pos.y = (lat_rad - ref_lat) * RADIUS;
    pos.x = (lon_rad - ref_lon) * RADIUS * std::cos((ref_lat + lat_rad) / 2.0);
    return pos;
}

Point2D quadBezier(double t, Point2D p0, Point2D p1, Point2D p2) {
    double u = 1.0 - t;
    Point2D p;
    p.x = (u * u) * p0.x + 2 * u * t * p1.x + (t * t) * p2.x;
    p.y = (u * u) * p0.y + 2 * u * t * p1.y + (t * t) * p2.y;
    return p;
}

Point2D get_active_target(Point2D current_pos, Point2D p0, Point2D p1, Point2D p2) {
    double closest_t = 0, min_dist = 999999; 
    for (double t = 0; t <= 1.0; t += 0.05) {
        Point2D curve_pt = quadBezier(t, p0, p1, p2);
        double d = std::sqrt(pow(curve_pt.x - current_pos.x, 2) + pow(curve_pt.y - current_pos.y, 2));
        if (d < min_dist) { min_dist = d; closest_t = t; }
    }
    return quadBezier(std::min(closest_t + 0.10, 1.0), p0, p1, p2);
}

double calculate_steering_error(Point2D current_pos, double heading_rad, Point2D p0, Point2D p1, Point2D p2) {
    Point2D target = get_active_target(current_pos, p0, p1, p2);
    double Tx = target.x - current_pos.x, Ty = target.y - current_pos.y;
    double mag = sqrt(Tx*Tx + Ty*Ty);
    if (mag > 0) { Tx /= mag; Ty /= mag; }
    
    double Hx = sin(heading_rad), Hy = cos(heading_rad);
    double cross = (Hx * Ty) - (Hy * Tx);
    double dot = (Hx * Tx) + (Hy * Ty);
    
    if (dot < 0) return (cross > 0) ? 1.0 : -1.0; // Target behind, hard turn
    return cross;
}

double dmsToRadians(double degrees, double minutes, double seconds) {
    double decimalDegrees = degrees + (minutes / 60.0) + (seconds / 3600.0);
    return decimalDegrees * (M_PI / 180.0);
}