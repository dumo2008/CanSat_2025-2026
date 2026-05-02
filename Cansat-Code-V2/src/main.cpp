// ----------------------------------------------------------------------------
// CanSat Project 2025-2026 - Team SkyByte V2
// - BMP280 sensor reading                                                 done
// - GPS module reading                                                    done
// - Data logging to SD card                                               done
// - Sending data through LoRa (SPI connected)                             done
// - AI input from RPi Zero                                                done   
// - Motor steering                                                        done
// 
// LoRa frequency is set for Europe (868.6 MHz)
// ----------------------------------------------------------------------------

//#define DEBUG           // define / undefine to enable / disable debug mode
//#define SD_EXTRA_INFO   // define / undefine to enable / disable extra SD card info printout at startup
//#define CYCLE_TIME      // define / undifine to enable / disable message to show time of each cycle
//#define FAKE_GPS        // define / undefine to enable / disable fake GPS data mode (for testing without GPS module)

#ifdef CYCLE_TIME
  unsigned long cycle_start_time = 0; // Tracks the start of each loop
#endif

#ifdef FAKE_GPS
  bool sim_descending = false; // Simulate ascent and descent
  
  // Elsenborn Military Base coordinates
  double sim_lat = 50.464504;    // Starting Lat
  double sim_lon = 6.189159;     // Starting Lon
  float  sim_alt = 600.0;        // Start at GROUND_LEVEL
#endif

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "SPI.h"                 // SPI
#include <Wire.h>                // I2C

#include <SD.h>                  // SD card
#include "FS.h"                  // SD Card 

#include <Adafruit_Sensor.h>     // Adafruit unified sensor base
#include <Adafruit_BMP280.h>     // BMP280 

#include <TinyGPSPlus.h>         // GPS parsing (TinyGPS++)
#include <HardwareSerial.h>      // Adafruit GPS Sensor

#include <LoRa.h>                // LoRa
#include <ESP32Servo.h>          // Servo

// Declarations functions
bool I2C_check(TwoWire *bus, byte address);                 // I2C device presence check

bool mountSD();                                             // Mount SD card and set up SPI connection
void SetFileName(fs::FS &fs, const char * path);            // Read last used version from VERSION_FILE
void writeToSD(fs::FS &fs, const char * path, String msg);  // Append one CSV line to current file

void receiveEvent(int howMany);                             // I2C receive event handler for RPi communication, updates rpi_state variable
void checkRPi();                                            // Checks if RPi is still connected and updates rpi_state

void updateSteering();                                      // Apply steering logic based on altitude and RPi state
void steerMotorStraight();                                  // Drive servo straight
void steerMotorLogic();                                     // Calculate bearing and steer towards target based on bearing error
float berHoek();                                            // Calculate bearing and angle to target 
void formatAImsg( String &msg );                            // Build message with motor and RPi state info

void readBMP( String &msg );                                // Read values from BMP280 
void formatBMPmsg( String &msg );                           // Format BMP280 values into CSV record

void readGPS(unsigned long ms);                             // Read and parse data, keep last valid GPS values
void formatGPSmsg( String &msg );                           // Format last valid GPS values into CSV record

void sendMsg( String outgoing );                            // Send message through LoRa

//-------------------------------------------------------------------------
// Parameters that need to be changed before flight
// ------------------------------------------------------------------------

// Set the value of the sea level pressure correct at your location
// You can find it at https://www.meteo.be/nl/weer/waarnemingen/belgie
// If you do so, the height will be calculated approx. correctly.
// + or - 8 m height difference is normal, as the sensor has a deviation.
#define SEALEVELPRESSURE_HPA (1027.3)

// Ground level altitude at lauch site (meters above sea level)
// Set this before uploading based on the launch location
// You can find it at e.g. https://whataltitude.com
#define GROUND_LEVEL 607

// Target coordinates
#define TARGET_LAT 50.469831
#define TARGET_LON 6.195742

// -------------------------------------------------------------------------

// Frequentie-instelling (voor Europa: 868 MHz)
#define LORA_FREQ 868.6E6

#define MAX_RETRIES 5 // Maximum number of retries

#define DELAY_TIME 1000 // Delay time between measurements (ms) & Delay time between 2 transmissions
#define MOTOR_STEERING_UPDATE_INTERVAL 4000 // Minimum time between motor updates (ms)
#define MOTOR_ACTIVATION_DELAY 5000 // Time to wait after reaching motor activation altitude before activating motor (ms)

#define csPIN     5  // Set chip select PIN for SPI connection 
#define resetPIN  14 // Set reset PIN for SPI connection
#define irqPIN    2  // Set IReq PIN for SPIconnection        

// Adres settings
#define SENDER_ADDRESS   0xA7
#define RECEIVER_ADDRESS 0xBB // 122 Decimal (our given adress)
#define LORA_ID          0x1B

// LoRa Paremeters
// For long distance — low data rate, high sensitivity
#define TxPower         20      // Maximum power (20 dBm) for long range
#define SignalBandwidth 125E3   // 125 kHz standard
#define CodingRate4     5       // 4/5 coding rate
#define SpreadingFactor 11      // 6–12 (higher = longer range, but slower data rate)
#define CRC             1       // CRC validation ON  
// #define SYNCWORD  0xF3 // Set Syncword for LoRa transmitter
                          // !! ensure same syncword is used at LoRa receiver

// General defines
#define MSG_LEN    150 // Message length of to be transmitted message
#define MSG_SD_LEN 500 // Message length of to be stored message
#define DELIMETER ";"  // Set CSV DELIMETER to ;

// Define record types
#define MOTOR_RECORD  "M"
#define BMP_RECORD    "B"
#define GPS_RECORD    "G"

#define ESP_SLAVE_ADDR  0x08 // Set ESP Slave address for RPi connection 
                             // Ensure RPi uses the same address

#define SDA_PIN 21 // I2C SDA pin for RPi connection
#define SCL_PIN 22 // I2C SCL pin for RPi connection

// BMP280 I2C address (some boards use 0x77)
#define BMP280_ADR 0x76

// Non-default pins for I2C connection
#define SDA_2 32    // Use this pin as secondary I2C SDA connection
#define SCL_2 33    // Use this pin as secondary I2C SCL connection

// SD Card SPI pins 
#define SD_CS_PIN   25    // CS        = GPIO25
#define SD_MOSI_PIN 23    // MOSI (DI) = GPIO23
#define SD_MISO_PIN 19    // MISO (DO) = GPIO19
#define SD_SCK_PIN  18    // SCK (CLK) = GPIO18

// Filenames / paths on SD
#define DATA_FILE_BASE "/CanSatSend"          // Base name for data files
#define VERSION_FILE   "/CanSatVersion.txt"   // Stores last used version number

// GPS Serial configuration
#define GPS_RX 17          // ESP32 RX2  <- GPS TX
#define GPS_TX 16          // ESP32 TX2  -> GPS RX
#define GPS_BAUD 9600      // GPS module baudrate

// Servo configuration
#define SERVO_PIN 13       // GPIO13 pin connected to servo signal wire

// Steering speeds
#define SERVO_LEFT  1022   // Steer left
#define SERVO_STOP  1500   // Neutral
#define SERVO_RIGHT 1974   // Steer right

// Steering durations
#define STEER_DURATION_L 1000
#define STEER_DURATION_R 1017

#define RPI_ACTIVE_ALTITUDE 200   // Minimum altitude (m) for RPi input 
#define DROPPED_ALT_TRESHOLD 50   // Minimum altitude difference to consider the CanSat as dropped (meters)

#define RPI_CONNECTION_LOST_ASSUMPTION_TIME 20000 // Time without RPi updates after which we assume connection is lost (ms)

bool bmp_connected = false;                     // Tracks BMP280 availability
bool sd_available = false;                      // Tracks SD availability
bool gps_valid = false;                         // Tracks GPS validity (at least 1 satellite)

bool START_COUNTDOWN_MOTOR_ACTIVATION = false;  // Start countdown for motor activation once we reach MOTOR_ACTIVE_ALTITUDE
bool MOTOR_ACTIVE = false;                      // Tracks whether motor is active
bool STEER = false;                             // Tracks whether we should steer

unsigned long trigger = 0;                      // Timer before motor activation 
unsigned long motor_update_interval = 0;        // Tracks the last time the motor moved

unsigned long last_RPi_update = 0;               // Tracks the last time we received an update from the RPi

Servo s;
TinyGPSPlus gps; // TinyGPS++ parser instance

// Create secondary I2C bus object on ESP32
TwoWire I2Ctwo = TwoWire(1);

// Bind BMP280 to the secondary I2C bus
Adafruit_BMP280 bmp(&I2Ctwo);

HardwareSerial GPSSerial(2);   // Use UART2 for GPS (hardware serial 2)

float bmp_temp = 0.0;          // Temperature in °C
float bmp_press = 0.0;         // Pressure in hPa
float bmp_alt = 0.0;           // Altitude in meters

int    gps_sats = 0;           // Last valid satellites count
float  gps_hdop = 0.0;         // Last valid HDOP
double gps_lat  = 0.0;         // Last valid latitude
double gps_lon  = 0.0;         // Last valid longitude
double gps_alt = 0.0;          // Last valid GPS altitude in meters

int currentVersion;            // version number (saved on SD)

String currentFilename = "";   // Active data filename for this session

String AI_msg = "";
String BMP_msg = "";
String GPS_msg = "";
String SD_msg  = "";

String rpi_state = "WAITING";  // Last received RPi state (default to Waiting until we receive something)

float altitude;
float max_altitude = 0.0;

float targetBearing;
float currentBearing;
float bearingError;

// Motor steering state
int turnL = 0;
int turnR = 0;

int servoState = 0; // -2 = Hard Left
                    // -1 = Light Left
                    // 0  = Straight
                    // 1  = Light Right
                    // 2  = Hard Right

void setup() {
  unsigned bmp_status; // Holds BMP initialization result

  // Serial Monitor start
  #ifdef DEBUG
    Serial.begin( 115200 ); 
    while (!Serial) {
      delay( 10 );
    }
    delay( 200 ); // Short delay to allow printout message on serial monitor
  #endif

  #ifdef DEBUG
    Serial.println();
    Serial.println( "=======================================" );
    Serial.println( "Start of Program" );
    Serial.println( "=======================================" );
    Serial.println();
  #endif

  #ifdef FAKE_GPS
    Serial.println("\nFAKE GPS MODE ENABLED: Simulating ascent and descent with changing GPS coordinates\n");
    Serial.println("------------------------------------------------------------------------\n");
    delay(15000); // Short delay to allow printout message on serial monitor
  #endif

  sd_available = mountSD(); // Try to mount SD card and set sd_available accordingly
  SetFileName(SD, VERSION_FILE);

  #ifdef DEBUG
    Serial.println("\n----------------------------------------------------------------------\n");
  #endif

  // Start LoRa communication
  LoRa.setPins( csPIN, resetPIN, irqPIN );

  // Start de LoRa-module
  #ifdef DEBUG
    Serial.println("2. => Starting LoRa module (SPI)\n");
  #endif

  while (!LoRa.begin(LORA_FREQ)) {
    #ifdef DEBUG
      Serial.println("Fout: kon LoRa-module niet starten!");
      delay(1000);
    #endif 
  }

  // Voor lange afstand — lage datasnelheid, hoge gevoeligheid
  LoRa.setTxPower( TxPower );                   // maximaal vermogen
  LoRa.setSignalBandwidth( SignalBandwidth );   // 125 kHz standaard
  LoRa.setCodingRate4( CodingRate4 );           // 4/5 codering
  LoRa.setSpreadingFactor( SpreadingFactor );   // 6–12 (hoger = groter bereik)
  #if CRC
    LoRa.enableCrc();                           // CRC aan (idem pico)
  #endif
  
  #ifdef DEBUG
    Serial.println( "LoRa module successful connected (SPI)" );
    Serial.println("\n----------------------------------------------------------------------\n");
  #endif

  // Bring up secondary I2C bus on custom pins
  I2Ctwo.begin(SDA_2, SCL_2);  // SDA=32, SCL=33

  // Initialize BMP280 at given address
  #ifdef DEBUG
    Serial.println("3. => Initializing BMP280 sensor (I2C-2)\n");
  #endif

  bmp_status = bmp.begin(BMP280_ADR);
  if (!bmp_status) {
    #ifdef DEBUG
      Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
    #endif
  } 
  else {
    bmp_connected = true;  // Sensor found

    // Optional: configure oversampling/filter to improve stability/noise
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.        
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling          
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling  
                    Adafruit_BMP280::FILTER_X16,      // IIR Filtering.         
                    Adafruit_BMP280::STANDBY_MS_500); // Standby time.          

    #ifdef DEBUG
      Serial.println("BMP-280 successful connected (I2C-2)");
      Serial.println("\n----------------------------------------------------------------------\n");
    #endif
  }

  // Connect RPi (master) to ESP (slave) through I2C 
  #ifdef DEBUG
    Serial.println("4. => Connecting RPi (master) to ESP (slave) (I2C-1)\n");
  #endif

  Wire.setPins(SDA_PIN, SCL_PIN);
  #ifdef DEBUG
    Serial.println("I2C Pins Set...");
  #endif

  bool I2C_State = Wire.begin((uint8_t)ESP_SLAVE_ADDR, SDA_PIN, SCL_PIN, 100000); 
  
  if (I2C_State) {
    Wire.onReceive(receiveEvent);
    #ifdef DEBUG
      Serial.println("RPi connected and in receive mode (I2C - interrupt)");
    #endif
  } 
  else {
    #ifdef DEBUG
      Serial.println("I2C Failed to start!");
    #endif
  }

  #ifdef DEBUG
    Serial.println("\n----------------------------------------------------------------------\n");
  #endif

  // GPS init
  #ifdef DEBUG
    Serial.println("5. => Initializing GPS module (UART2)\n");
  #endif

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);  // Start UART2 for GPS

  #ifdef DEBUG
    Serial.println("GPS module initialized on UART2 @ 9600 baud.");
    Serial.println("\n----------------------------------------------------------------------\n");
  #endif

  // Servo init
  #ifdef DEBUG
    Serial.println("6. => Initializing Servo (PWM)\n");
  #endif

  s.setPeriodHertz(50); 
  s.attach(SERVO_PIN, 500, 2500);
  s.writeMicroseconds(SERVO_STOP);

  #ifdef DEBUG
    Serial.println("Servo Initialized");
    Serial.println("\n======================================================================\n");
  #endif
}

void loop() {
  #ifdef CYCLE_TIME
    cycle_start_time = millis(); // Record start time
  #endif

  // Re-check BMP280 each loop
  bmp_connected = I2C_check(&I2Ctwo, BMP280_ADR);

  // Read BMP data
  readBMP(BMP_msg);

  // Collect and parse GPS data each loop
  readGPS(200); // Read NMEA chars for ~200 ms and update last valid GPS values

  checkRPi(); // Check RPi connection and update state

  // Apply steering logic based on altitude and RPi state
  updateSteering();

  // Build messages for LoRa transmission
  formatAImsg( AI_msg );
  formatBMPmsg( BMP_msg );
  formatGPSmsg( GPS_msg );
  
  // Transmit msg through LoRa
  sendMsg( AI_msg );  // Transmit AI msg through LoRa
  sendMsg( BMP_msg ); // Transmit BMP msg through LoRa
  sendMsg( GPS_msg ); // Transmit GPS msg through LoRa
  
  #ifdef DEBUG
    Serial.println( "\n----------------------------------------------------------------------\n" );
    Serial.println( AI_msg );  // Print data on Serial monitor
    Serial.println( BMP_msg ); // Print data on Serial monitor
    Serial.println( GPS_msg ); // Print data on Serial monitor
  #endif

  // SD logging
  if (sd_available) {
    SD_msg = AI_msg + '\n' + BMP_msg + '\n' + GPS_msg;
    writeToSD(SD, currentFilename.c_str(), SD_msg ); // append the dataline to the backup file
  }

  delay( DELAY_TIME );

  // Calculate complete cycle time
  #ifdef CYCLE_TIME
    unsigned long duration = millis() - cycle_start_time;
    Serial.print("\nCycle Time: ");
    Serial.print(duration);
    Serial.println(" ms");
  #endif

  #ifdef DEBUG
    Serial.println("\n----------------------------------------------------------------------\n");
  #endif
}

// -------------------------------------------------------------------------
// This function reads NMEA bytes for a given time and update last valid GPS values
// -------------------------------------------------------------------------
void readGPS(unsigned long ms) {

#ifdef FAKE_GPS
  if (!sim_descending) {
    sim_alt += 100.0; // Fast ascent
    if (sim_alt >= 1700.0) {
      sim_descending = true;
      #ifdef DEBUG
        Serial.println("SIM: BEGIN DESCEND\n");
      #endif
    }
  }
  else {
    if (sim_alt >= GROUND_LEVEL) {
      sim_alt -= 70.0;   // Our descent speed
      sim_lat += 0.000500; // Move North
    }
  }
 
  gps_lat  = sim_lat;
  gps_lon  = sim_lon;
  gps_alt  = sim_alt;
  gps_sats = 10;
  gps_hdop = 0.8;
  gps_valid = true;

  return;
#endif

  unsigned long start = millis();

  while (millis() - start < ms) {
    while (GPSSerial.available()) {
      char c = GPSSerial.read();
      gps.encode(c);
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
      gps_lat = gps.location.lat();
      gps_lon = gps.location.lng();
      gps_valid = true;
    }

    if (gps.satellites.isUpdated()) {
      gps_sats = gps.satellites.value();
    }

    if (gps.hdop.isUpdated()) {
      gps_hdop = gps.hdop.hdop();
    }

    if (gps.altitude.isUpdated()) {
      gps_alt = gps.altitude.meters();
    }
  }
}

// -------------------------------------------------------------------------
// This function gets the values from the GPS Module
// and appends them to the CSV-record 
// Values read and appended are: Number of satelites in view
//                               Quality of GPS data
//                               GPS coordinates - latitude, longitude
//                               GPS altitude
// Note: altitude can only be measured with > 3 satelites in view
// Or may take a while before the GPS sensor "connects" to GPOS satelites
// Therefore start this program well ahead of rocket launch to ensure proper 
// GPS reading.
// -------------------------------------------------------------------------
void formatGPSmsg( String &msg ) { 

  msg = String( GPS_RECORD ) + DELIMETER;
  msg = msg + String( gps_sats ) + DELIMETER;
  msg = msg + String( gps_hdop, 2 ) + DELIMETER;
  msg = msg + String( (float)gps_lat, 6 ) + DELIMETER;
  msg = msg + String( (float)gps_lon, 6 ) + DELIMETER;
  msg = msg + String( (float)gps_alt, 2 );
}

// -------------------------------------------------------------------------
// These functions read values from the BMP280 Sensor
// and appends them to the CSV-record 
// Values read and appended are: Temperature in °C
//                               Pressure in hPa
//                               Altitude in m
//                               Humidity in %
// For accurate altitude calculation, set the sea level pressure correct at 
// begin of this program!
// -------------------------------------------------------------------------
void readBMP( String &msg ) {
  if (!bmp_connected) {
    msg = String( BMP_RECORD ) + DELIMETER + "0" + DELIMETER + "0" + DELIMETER + "0";
    return;
  }

  bmp_temp = bmp.readTemperature();                 // °C
  bmp_press = bmp.readPressure() / 100.0F;          // hPa 
  bmp_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); // m
}

void formatBMPmsg( String &msg ) {
  msg = String( BMP_RECORD ) + DELIMETER;
  msg = msg + String( bmp_temp, 2 ) + DELIMETER;
  msg = msg + String( bmp_press, 2 ) + DELIMETER;
  msg = msg + String( bmp_alt, 2 );
}

// -------------------------------------------------------------------------
// This function applies the steering logic each loop:
// -  Wait until MOTOR_ACTIVE_ALTITUDE (800m) is reached to prevent 
//    motor activation during ascent or on the launch site.
// -  Once 800m is hit, wait 10 seconds (countdown) to ensure 
//    the CanSat has cleared the rocket and stabilized.a
// -  Above RPI_ACTIVE_ALTITUDE: Steer towards target using GPS bearing.
// -  Below RPI_ACTIVE_ALTITUDE: If RPi says "Safe", stop motor and fly straight 
//                               If "Unsafe", continue steering towards target.
// -------------------------------------------------------------------------
void updateSteering() {
  if (!gps_valid) {
      #ifdef DEBUG
        Serial.println("GPS data invalid");
      #endif
    steerMotorStraight(); 
    return;
  }

  altitude = bmp_alt - GROUND_LEVEL; // Calculate altitude above ground level

  if (altitude > max_altitude) {
    max_altitude = altitude; // Update max altitude reached
  }

  // Prevent motor activation during ascent or on launch site by waiting until we reach the dropped altitude threshold
  if (!START_COUNTDOWN_MOTOR_ACTIVATION) {
    if (altitude < (max_altitude - DROPPED_ALT_TRESHOLD)) {
      START_COUNTDOWN_MOTOR_ACTIVATION = true;
      trigger = millis(); // Start timer for motor activation
      #ifdef DEBUG
        Serial.println("MOTOR ACTIVATION COUNTDOWN STARTED\n");
      #endif
    } 
    else {
      // Below motor activation altitude, keep motor off
      steerMotorStraight();
      return;
    }
  }

  // Once we have reached MOTOR_ACTIVE_ALTITUDE, wait for the countdown to finish before activating the motor
  if (!MOTOR_ACTIVE) {
    // Wait few seconds after reaching motor activation altitude
    if (millis() - trigger >= MOTOR_ACTIVATION_DELAY) { 
      MOTOR_ACTIVE = true;
      #ifdef DEBUG
        Serial.println("MOTOR ACTIVATED\n");
      #endif
    } 
    else {
      // Countdown not finished, keep motor off
      steerMotorStraight();
      return;
    }
  }

  // Motor is active, apply steering logic based on altitude and RPi state
  if (millis() - motor_update_interval >= MOTOR_STEERING_UPDATE_INTERVAL) {
    motor_update_interval = millis(); // Reset the 4-second timer

    if (altitude > RPI_ACTIVE_ALTITUDE) { 
      #ifdef DEBUG
        Serial.print("AI STEERING INACTIVE\n\n");
      #endif
      steerMotorLogic(); // Above RPI_ACTIVE_ALTITUDE, steer towards target
    } 
    else {  
      // Below RPI_ACTIVE_ALTITUDE, check RPi state
      #ifdef DEBUG
        Serial.println("AI STEERING ACTIVE\n");
      #endif
      if (rpi_state == "SAFE") {
        steerMotorStraight();   // Landing is safe, stop motor and fly straight
      } 
      else {
        steerMotorLogic(); // Landing is unsafe, steer towards target
      }
    } 
  }
}

// -------------------------------------------------------------------------
// This function calculates bearing angle and moves the servo
// It applies a 2-level correction (light/strong) based on the bearing error
// -------------------------------------------------------------------------
void steerMotorLogic() {
  targetBearing = berHoek();
  currentBearing = gps.course.deg();
  bearingError = targetBearing - currentBearing;

  // Normalize bearing error to range [-180, 180]
  if (bearingError > 180) {
    bearingError -= 360;
  }
  if (bearingError < -180) {
    bearingError += 360;
  }

  #ifdef DEBUG
    Serial.print("target bearing: ");
    Serial.println(targetBearing);
    Serial.print("current bearing: ");
    Serial.println(currentBearing);
    Serial.print("bearing error: ");
    Serial.println(bearingError);
    Serial.println("");
  #endif

  int goal = 0;

  // If error is smaller then 10 degrees, 
  if (abs(bearingError) < 10) { 
    goal = 0; // Go straight
  } 
  else if (bearingError < 0) { // Target left
    if (bearingError > -45) { 
      goal = -1; // Turn left light
    } 
    else { 
      goal = -2; // Turn left strong
    }
  } 
  else { // Target right
    if (bearingError < 45) { 
      goal = 1; // Turn right light
    } else { 
      goal = 2; // Turn right strong
    }
  }

  #ifdef DEBUG
    if (servoState != goal) {
      Serial.print("Changing servo state from ");
      Serial.print(servoState);
      Serial.print(" to ");
      Serial.println(goal);
      Serial.println("");
    }
  #endif

  while (servoState != goal) {
    if (servoState < goal) {
      s.writeMicroseconds(SERVO_RIGHT);
      delay(STEER_DURATION_R);
      servoState++;
    }
    else if (servoState > goal) {
      s.writeMicroseconds(SERVO_LEFT);
      delay(STEER_DURATION_L);
      servoState--;
    }
  }

  s.writeMicroseconds(SERVO_STOP); // Stop servo after reaching desired position

  if (goal < 0) {
    turnL = abs(goal);
    turnR = 0;
  } 

  if (goal > 0) {
    turnL = 0;
    turnR = goal;
  }
}

// -------------------------------------------------------------------------
// This function includes the logic to fly straight when being called
// -------------------------------------------------------------------------
void steerMotorStraight() {
  if (servoState != 0) {
    #ifdef DEBUG
      Serial.print("Changing servo state from ");
      Serial.print(servoState);
      Serial.println(" to 0 (straight)");
      Serial.println("");
    #endif

    while (servoState != 0) {
      if (servoState > 0) {
        s.writeMicroseconds(SERVO_LEFT);
        delay(STEER_DURATION_L);
        servoState--;
      }
      else{
        s.writeMicroseconds(SERVO_RIGHT);
        delay(STEER_DURATION_R);
        servoState++;
      }
    }

    #ifdef DEBUG
      Serial.println("Servo is now straight");
      Serial.println("");
    #endif
  }

  s.writeMicroseconds(SERVO_STOP);
  turnL = 0;
  turnR = 0;
}


// -------------------------------------------------------------------------
// This function runs automatically when the RPi sends data
// It reads the single byte sent by the RPi (0 or 1) and updates the 
// rpi_state variable accordingly ("SAFE" for 1, "UNSAFE" for 0)
// --------------------------------------------------------------------------
void receiveEvent(int howMany) {
  if (Wire.available()) {
    int val = Wire.read(); // Read the single byte (0 or 1)

    if (altitude <= RPI_ACTIVE_ALTITUDE && MOTOR_ACTIVE) { // Only update RPi state if we are below the RPI active altitude and motor is active
      if (val == 1) {
        rpi_state = "SAFE";
      } else if (val == 0) {
        rpi_state = "UNSAFE";
      }
    }
    
    last_RPi_update = millis(); // Update last RPi update time
    
    #ifdef DEBUG
      Serial.print("I2C Received: ");
      Serial.print(val);
      Serial.print(" -> ");
      Serial.println(rpi_state);
    #endif
  }
}

// -------------------------------------------------------------------------
// This function checks if we have received an update from the RPi in the last 20
// seconds. It only triggers COMM_LOST if we are below the RPI_ACTIVE_ALTITUDE
// where we actually expect AI data to be useful.
// -------------------------------------------------------------------------
void checkRPi() {
  // Only monitor connection if we are low enough for the RPi to be "Active"
  if (altitude <= RPI_ACTIVE_ALTITUDE && MOTOR_ACTIVE) {
    if (millis() - last_RPi_update > RPI_CONNECTION_LOST_ASSUMPTION_TIME) {
      rpi_state = "COMM_LOST"; 
    }
  }
}

// -------------------------------------------------------------------------
// This function calculates bearing angle (0–360°) from current GPS position
// to the target coordinates
// -------------------------------------------------------------------------
float berHoek() {
  float canLatRad    = gps_lat    * M_PI / 180.0;
  float canLonRad    = gps_lon    * M_PI / 180.0;
  float targetLatRad = TARGET_LAT * M_PI / 180.0;
  float targetLonRad = TARGET_LON * M_PI / 180.0;

  float deltaLon = targetLonRad - canLonRad;

  float x = sin(deltaLon) * cos(targetLatRad);
  float y = cos(canLatRad) * sin(targetLatRad)
          - sin(canLatRad) * cos(targetLatRad) * cos(deltaLon);

  float radHoek = atan2(x, y);
  float degHoek = fmod((radHoek * 180.0 / M_PI) + 360.0, 360.0);

  return degHoek;
}

// -------------------------------------------------------------------------
// This function makes msg with motor steering and RPi state
// -------------------------------------------------------------------------
void formatAImsg( String &msg ) {
    msg = String( MOTOR_RECORD ) + DELIMETER;
    msg = msg + String(turnL) + DELIMETER;
    msg = msg + String(turnR) + DELIMETER;
    msg = msg + rpi_state;
}

// -------------------------------------------------------------------------
// This function sends the output CSV record through LoRa to the receiver
// -------------------------------------------------------------------------
void sendMsg( String outgoing ) {
  char buffer[MSG_LEN];

  outgoing.toCharArray(buffer, MSG_LEN); // convert String to char array

  // Transmit msg through LoRa
  LoRa.beginPacket();
  // Voeg ontvangeradres, zenderadres, ID en bericht lengte toe aan de LoRa header
  LoRa.write(RECEIVER_ADDRESS);
  LoRa.write(SENDER_ADDRESS);
  LoRa.write( LORA_ID );
  LoRa.write( strlen( buffer ) ); // bericht lengte

  // Voeg het bericht toe als één blok
  LoRa.write((const uint8_t*)buffer, strlen( buffer ) );
  
  LoRa.endPacket();
}

// -------------------------------------------------------------------------
// This function appends one CSV line to currentFilename
// -------------------------------------------------------------------------
void writeToSD(fs::FS &fs, const char * path, String msg) {
  char msgSD[ MSG_SD_LEN ] = ""; // Needed do convert string to char array

  msg.toCharArray(msgSD, sizeof( msgSD ) ); // Convert String to Char array
  // Add new line at the end of the char array
  int len = strlen( msgSD );
  msgSD[len] = '\n';
  msgSD[len + 1] = '\0';

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    #ifdef DEBUG
      Serial.printf("\tFailed to open file %s for appending\n", path );
    #endif
    return;
  }
  if( !file.print(msgSD) ) {
    #ifdef DEBUG
      Serial.println( "\tERROR : Append record failed" );
    #endif
  }
  file.close();
}

// -------------------------------------------------------------------------
// This function returns true if device at 'address' ACKs on given bus
// -------------------------------------------------------------------------
bool I2C_check(TwoWire *bus, byte address) {
  bus->beginTransmission(address);
  byte error = bus->endTransmission();
  return (error == 0);
}

// -------------------------------------------------------------------------
//  This function tries to mount the SD card and set up the SPI connection
// -------------------------------------------------------------------------
bool mountSD() {
  int count = 0;

  // Initialize the SPI bus with explicit pins
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  delay(100);

  // Mount SD Card File System
  #ifdef DEBUG
    Serial.println("1. => Mounting SD Card\n");
    Serial.print("Initializing SD card...");
  #endif

  // Try a limited number of time to connect to SC-Card Module
  while ( (!SD.begin( SD_CS_PIN )) && count < MAX_RETRIES ) {
    #ifdef DEBUG
      Serial.print(".");
    #endif
    delay( 50 );
    count++;
  }
  if ( count >= MAX_RETRIES ) {
    #ifdef DEBUG
      Serial.println( "\n\tInitialization of SD Card failed !!" );
    #endif
    return false;
  }

  #ifdef DEBUG
    Serial.println("\tinitialization SD Card successful\n");
  #endif

  #ifdef SD_EXTRA_INFO
    uint64_t cardSize = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.print("SD kaart grootte: ");
    Serial.print(cardSize);
    Serial.println(" MB");

    uint8_t cardType = SD.cardType();
    Serial.print("SD kaart type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }
    #endif

  return true;
}

void SetFileName(fs::FS &fs, const char * path){
  String line = "";
  int versieNr = 0;

  #ifdef DEBUG
    Serial.printf("Reading file: %s\n", path);
  #endif

  // 1. Open versie file for reading
  // Als het bestand niet bestaat, start dan met versie 0
  File file = fs.open(path, FILE_READ);
  if(!file){
    #ifdef DEBUG
      Serial.println("Version file not found, creating new one with version 0");
    #endif
    versieNr = 0;

  } else { // Versie File bestaat wel, lees laatste versie nummer
    #ifdef DEBUG
      Serial.println("Version file found, reading last version number");
    #endif

    // 2. Lees de laatste regel van het bestand (moet de laatste versie bevatten)
    while(file.available()){
      line = file.readStringUntil('\n');
    }
    file.close();
    // 3. String → int
    versieNr = line.toInt();
  }  

  #ifdef DEBUG
    Serial.printf("Oude waarde: %d\n", versieNr);
  #endif

  // 4. Verhoog waarde
  versieNr++;

  #ifdef DEBUG
    Serial.printf("Nieuwe waarde: %d\n", versieNr);
  #endif

  // 5. Bestand opnieuw openen om te schrijven (overschrijven!)
  file = fs.open(path, FILE_WRITE);
  if(!file){
    #ifdef DEBUG
      Serial.println("Failed to open file for writing");
    #endif
    return;
  }

  // 6. Nieuwe waarde schrijven en bestand sluiten
  file.println(versieNr);
  file.close();

  #ifdef DEBUG
    Serial.printf("Nieuwe waarde geschreven naar file: %d\n", versieNr);
  #endif 

  // Set Gloabal filename
  currentFilename = String(DATA_FILE_BASE) + String(versieNr) + ".txt";
  #ifdef DEBUG
    Serial.printf("Nieuwe filename: %s\n", currentFilename.c_str());
  #endif
}