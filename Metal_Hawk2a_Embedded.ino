#include <Wire.h>
#include <Arduino.h>
#include <SD.h> //SD card
#include <SPI.h> //SD card
#include <EEPROM.h> //EEPROM
#include <Adafruit_Sensor.h> //BMP581
#include "Adafruit_BMP5xx.h" //BMP581
#include <Adafruit_INA260.h> //INA260
#include <SparkFun_u-blox_GNSS_v3.h> // SAM-M8Q
#include <Adafruit_BNO08x.h>  // BNO085
#include <Servo.h> // Servos ofc
#include <math.h> // for steering
#include <cmath> 
#include "Navigation.h" 

//======
// Pins
//======

// BNO085: no physical INT or RESET pin connected
const int ledpin = LED_BUILTIN;
const byte scl_one = 27;
const byte sda_one = 26;
const byte scl_zero = 9;
const byte sda_zero = 8;
const int ground_cam = 22;
const int release_cam = 14;

//****************************************//

const int MMF_ADDR = 0; // The memory "slot" we will use (EEPROM)

const int chipSelect = 17; //SD card

//=========
// Sensors
//=========
Adafruit_BMP5xx bmp581; //Declaring BMP581
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode
SFE_UBLOX_GNSS sam_m8q; //Declaring SAM-M8Q
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

//=========================
// TARGET LOCATIONS
//=========================
const double TARGET_LAT_RAD = dmsToRadians(38, 22, 33.54); // Get these from google earth
const double TARGET_LON_RAD = -dmsToRadians(79, 36, 28.3);
const double GATE1_LAT_RAD = dmsToRadians(38, 22, 32.78); // Intermediate point
const double GATE1_LON_RAD = -dmsToRadians(79, 36, 26.36); 
const double GATE2_LAT_RAD = dmsToRadians(38, 22, 34.28); // Intermediate point 
const double GATE2_LON_RAD = -dmsToRadians(79, 36, 30.11); 

//========================
// Navigation Globals
//========================
const float MAGNETIC_DECLINATION = -8.5; // 8.5 degrees West for Monterey, VA
bool curve_generated = false;
Point2D p0, p1, p2;
Point2D gate1_m, gate2_m;

// Initialize the PID Controller (Kp, Ki, Kd) - Tune these values later!
PIDController steeringPID(45.0, 0.0, 10.0);


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
int GPS_SATS; // Donee
char ECHO[128]; // Done
double VECTOR_PRODUCT = 0.0;

//================================
// Very importatatnt declarations
//================================
float velocity = 0;
float apogee = 0;
float release;
unsigned long flare_start_time = 0;
const int flare_duration = 2000;
const int start_brake_angle = 20;
const int max_brake_angle = 135;

bool nose_fired = false, probe_fired = false, egg_fired = false, flare = false;

bool gps_online = false, bmp_online = false, ina_online = false, bno_online = false, sd_online = false;

float last_heading = 0;
float current_heading = 0;

float ground_altitude = 0;
float ground_pressure = 0;
float raw_hPa = 0;
float sum_pressure = 0;
unsigned long previousMillis = 0;

int cam_loop_count = 0;

float total_velocity = 0;
float avg_velocity = 0;
int braking_loop = 0;
bool braking = false;
int braking_bool_loop = 0;

//=======================
// Required Declarations
//=======================
unsigned long last_time = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
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

void setup() {
  Serial.begin(9600);
  delay(500);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));
  
  pinMode(ground_cam, OUTPUT);
  pinMode(release_cam, OUTPUT);
  digitalWrite(release_cam, HIGH);
  digitalWrite(ground_cam, HIGH);


  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(115200);

  pinMode(ledpin, OUTPUT);

  delay(1000);

  Wire.setSDA(sda_zero);
  Wire.setSCL(scl_zero);
  Wire.begin();
  Wire.setClock(100000); // Start slow for stability
  delay(100); // Give the bus a moment to stabilize;

  Wire1.setSDA(sda_one);
  Wire1.setSCL(scl_one);
  Wire1.begin();
  Wire1.setClock(100000);
  delay(100);

  SPI.setRX(16);
  SPI.setTX(19);
  SPI.setSCK(18);

  release_s.attach(release_s_pin);
  port_s.attach(port_s_pin);
  starboard_s.attach(starboard_s_pin);
  delay(100);

  starboard_s.writeMicroseconds(500);
  port_s.writeMicroseconds(2500);

  //==========
  // BMP & SD
  //==========

  bmp_online = bmp581.begin(0x46, &Wire);
  if (!bmp_online) bmp_online = bmp581.begin(0x47, &Wire);
  sd_online = SD.begin(chipSelect);

  if (!bmp_online) Serial.println("BMP not online");
  if (!sd_online) Serial.println ("SD not online");

  //=================
  // INA & BNO & GPS
  //=================
  sam_m8q.begin(Wire1, 0x42);  
  gps_online = true;
  if (!gps_online) {
    delay(500);
    gps_online = sam_m8q.begin(Wire1, 0x42); // Second attempt
  }

  ///
  /// temp
  ///
  Wire1.beginTransmission(0x42);
  byte gps_error = Wire1.endTransmission();
  Serial.print("GPS I2C ping: ");
  Serial.println(gps_error == 0 ? "ALIVE" : "DEAD");
  Serial.print("GPS begin() returned: ");
  Serial.println(gps_online ? "true" : "false");
  ///
  ///
  ///
  delay(200);
  bno_online = bno085.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1);
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
    sam_m8q.setNavigationFrequency(5);
    sam_m8q.setAutoPVT(true);
    sam_m8q.setI2COutput(COM_TYPE_UBX);
    sam_m8q.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
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

  digitalWrite(release_cam, HIGH);
  digitalWrite(ground_cam, HIGH);
  digitalWrite(ledpin, HIGH);

  // Add this near the bottom of your setup() function
  gate1_m = gps_to_meters(GATE1_LAT_RAD, GATE1_LON_RAD, TARGET_LAT_RAD, TARGET_LON_RAD);
  gate2_m = gps_to_meters(GATE2_LAT_RAD, GATE2_LON_RAD, TARGET_LAT_RAD, TARGET_LON_RAD);
  p2 = gps_to_meters(TARGET_LAT_RAD, TARGET_LON_RAD, TARGET_LAT_RAD, TARGET_LON_RAD); // This will just be (0,0)
}


void loop() {
  collect_telemetry();
  receive_command();

  Point2D current_pos = gps_to_meters(lat_rad, lon_rad, TARGET_LAT_RAD, TARGET_LON_RAD);
  double heading_rad = current_heading * (M_PI / 180.0);
  braking_system(current_pos, heading_rad, p2);

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


      if (cam_loop_count == 0){
        digitalWrite(ground_cam, LOW);
      }
      if (cam_loop_count == 20){
      digitalWrite(release_cam, LOW);
      }
      cam_loop_count += 1;

        save_flight_state();
      }
    }
    else {apogee_counter = 0;}
  }

  else if(sw_state == "APOGEE"){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 3000) {
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

    // Nose Release
    if (ALTITUDE <= (apogee * 0.7) && !nose_fired) {
      release_s.write(nose_release);
      nose_fired = true;
    }
    
    // Braking
    else if (ALTITUDE <= 20.0){
      brake_flare();
    }

    // Egg release
    else if (ALTITUDE <= 4.0 || velocity <= 2){
      sw_state = "PAYLOAD_RELEASE";
      release_s.write(egg_release);
      egg_fired = true;
      save_flight_state();
    }

    // Curve generator
    if (!curve_generated && gps_online) {
      p0 = gps_to_meters(lat_rad, lon_rad, TARGET_LAT_RAD, TARGET_LON_RAD); // This point should be (0,0)
      
      // Dynamic Gate Selection: Choose the gate closest to our release point
      double d1 = sqrt(pow(p0.x - gate1_m.x, 2) + pow(p0.y - gate1_m.y, 2));
      double d2 = sqrt(pow(p0.x - gate2_m.x, 2) + pow(p0.y - gate2_m.y, 2));
      p1 = (d1 < d2) ? gate1_m : gate2_m;
      
      curve_generated = true;
    }

    // 2. Active Steering Logic
    if (curve_generated && !(braking)) {

<<<<<<< HEAD
    check_and_redraw_path(current_pos, p0, p1, p2, gate1_m, gate2_m, 20.0);

    // Get error (Positive = Turn Left, Negative = Turn Right)
    double error = calculate_steering_error(current_pos, heading_rad, p0, p1, p2);

    // Your set mechanical throw amount (how many microseconds to pull the line)
    int fixed_turn_amount = 400; 

    // Establish baselines for both servos
    int p_out = 2500; // Left servo default (relaxed)
    int s_out = 500;  // Right servo default (relaxed)

    // Steering Logic
    if (std::abs(error) < 0.05) {
        // 1. Moving straight / in deadband: Both servos remain at their initial/zeroed states
        p_out = 2500;
        s_out = 500;
    } 
    else if (error > 0) {
        // 2. Turn LEFT: Pull left servo down, leave right servo relaxed at initial state
        p_out = 2500 - fixed_turn_amount; 
        s_out = 500; 
    } 
    else if (error < 0) {
        // 3. Turn RIGHT: Pull right servo up, leave left servo relaxed at initial state
        p_out = 2500;
        s_out = 500 + fixed_turn_amount;
=======
      check_and_redraw_path(current_pos, p0, p1, p2, gate1_m, gate2_m, 20.0);
      
      // Get error and compute PID correction
      double error = calculate_steering_error(current_pos, heading_rad, p0, p1, p2);

      double fixed_turn_amount = 300.0; 
      double correction = 0.0;

      // 2. The Steering Logic with a Deadband
      if (std::abs(error) < 0.05) {
          // We are basically straight. Do nothing to prevent servo jitter.
          correction = 0.0; 
      } 
      else if (error > 0) {
          // We need to turn LEFT
          correction = fixed_turn_amount;
      } 
      else if (error < 0) {
          // We need to turn RIGHT
          correction = -fixed_turn_amount;
      }

      // Map to servos (90 is neutral)
      int p_out = 2500 - (int)correction; // correction needs to be small
      int s_out = 500 + (int)correction;

      port_s.write(constrain(p_out, 500, 2500)); // Left
      starboard_s.write(constrain(s_out, 500, 2500)); // Right

      // Update telemetry variables
      VECTOR_PRODUCT = error;
>>>>>>> parent of 1143eb0 (Update Metal_Hawk2a_Embedded.ino)
    }
  }

  else if(sw_state == "PAYLOAD_RELEASE"){
    if((abs(velocity) <= 0.2) && (ALTITUDE  < 5.0)){
      sw_state = "LANDED";
      release_s.write(egg_release);
      save_flight_state();
    }
  }

  else if(sw_state == "LANDED"){
    digitalWrite(ledpin, (millis() / 500) % 2);
    digitalWrite(release_cam, HIGH);
    digitalWrite(ground_cam, HIGH);
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
      heading += MAGNETIC_DECLINATION;

      if (heading < 0) heading += 360.0f;

      last_heading    = current_heading;
      current_heading = heading;
    }
  }
  

  if (millis() - last_telem_time > 100) {// Waits 0.1 second
    last_telem_time = millis();

    //==========
    // SAM-M8Q
    //==========
    if (gps_online && sam_m8q.getPVT(250)){
      GPS_LATITUDE = sam_m8q.getLatitude() / 10000000.0;
      GPS_LONGITUDE = sam_m8q.getLongitude() / 10000000.0;
      GPS_ALTITUDE = sam_m8q.getAltitude() / 1000.0;
      GPS_SATS = sam_m8q.getSIV();
      sprintf(GPS_TIME, "%02d:%02d:%02d", sam_m8q.getHour(), sam_m8q.getMinute(), sam_m8q.getSecond());

      lat_rad = (GPS_LATITUDE * M_PI) / 180.0;
      lon_rad = (GPS_LONGITUDE * M_PI) / 180.0;
    } 
    ///
    ///
    ///
    else { 
      Serial.print("GPS poll failed. gps_online=");
      Serial.println(gps_online);      
    }
    ///
    ///
    
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
        ALTITUDE = (ALTITUDE * 0.5) + (current_rel_alt * 0.5);
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
        velocity = (velocity * 0.5) + (instant_velocity * 0.5);
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
    Serial1.print(TELEMETRY);
    Serial1.print('\r');
    PACKET_COUNT += 1;

    ///REMOVE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ///Serial.println(TELEMETRY);
    ///REMOVE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
  while (Serial1.available() > 0) {
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
          uint32_t gps_total_seconds = (sam_m8q.getHour() * 3600UL) + (sam_m8q.getMinute() * 60UL) + sam_m8q.getSecond();
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
          digitalWrite(ground_cam, LOW);
        }
        else if (state != NULL && strcmp(state, "OFF") == 0){
          digitalWrite(ground_cam, HIGH);
        }
      }
      /*======*/
      /* DCAM */
      /*======*/
      else if ((type != NULL && strcmp(type, "DCAM") == 0)) {
        if (state != NULL && strcmp(state, "ON") == 0) {
          digitalWrite(release_cam, LOW);
        }
        else if (state != NULL && strcmp(state, "OFF") == 0) {
          digitalWrite(release_cam, HIGH);
        }
      }
    }  
  }
}

void brake_flare() {
  if (!flare){
    flare_start_time = millis();
    flare = true;
  }

  unsigned long elapsed = millis() - flare_start_time;
  float progress = constrain((float)elapsed / (float)flare_duration, 0.0, 1.0);
  int current_flare_angle = start_brake_angle + (progress * (max_brake_angle - start_brake_angle));
  port_s.write(current_flare_angle);
  starboard_s.write(current_flare_angle);
}

void braking_system(Point2D current_pos, double heading_rad, Point2D p2){
  if (ALTITUDE <= (apogee * 0.7)){
    total_velocity += velocity;
    braking_loop += 1;
    avg_velocity = total_velocity/braking_loop;
  }

  if (ALTITUDE <= (apogee * 0.65)){
    if ((avg_velocity >= 6.5) && (braking == false) && (is_aligned_for_braking(current_pos, heading_rad, p2) == true)){
      starboard_s.writeMicroseconds(2500);
      port_s.writeMicroseconds(500);
      braking = true;
      braking_bool_loop = 0;
    }

    else if (braking == true){
      braking_bool_loop += 1;
      if (braking_bool_loop == 20){
        braking = false;
        starboard_s.writeMicroseconds(500);
        port_s.writeMicroseconds(2500);
      }
    }

    else if (avg_velocity <= 5.5){
      starboard_s.writeMicroseconds(2500);
      port_s.writeMicroseconds(500);
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

