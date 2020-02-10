//#define DEBUG 1
#define ARDUINOJSON_USE_DOUBLE 1
//#include "OTAUpdate.h"
#include "Motor.h"
#include "Geo.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <ros.h>

// Set the connection to rosserial socket server
ros::NodeHandle nh;

Geo geo(10,1,true);
Motor * motor_LF;
Motor * motor_RF;

const char* SSID = "Frontier4720";
const char* PASSWORD =  "6253020202";
const char* MQTTSERVER = "96.31.86.129";
#define PUBLISHER "TT"
#define SUBSCRIBER "TT/47b5be2c4ed69233"

const int MQTTPORT = 1883;
const char* MQTTUSER = "rbarnes";
const char* MQTTPASSWORD = "sasha23";

//ARUCO
int arucoDir=0;

//Make this changeable via config or front end interface
const int MAXWAYPOINTS = 1024;

static PVector gpsLocal;
PVector gpsRemote(27.9115465,-82.45744233);

PVector master;
//PVector wp;
StaticJsonDocument<350> doc;
bool saveWp = true;
#define PI2         PI * 2

// Get the Chip/MacId one time for reuse
char cMacId[16];

//#define MAG_ADDR 0x0e //7-bit address for the MAG3110, doesn't change
#define SDA_PIN 21
#define SCL_PIN 22

//GPS Baud Rates
static const uint32_t GPSBaud = 9600;

//Definition for Geo Calculatios
#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif


// Pin variables
//#define SERVO_PIN 3

//#define GPS_TX_PIN 6
//#define GPS_RX_PIN 5

//#define BLUETOOTH_TX_PIN 10
//#define BLUETOOTH_RX_PIN 11

#define L_EN_R 15
#define L_EN_F 2

#define L_PWM_REVERSE 18
#define L_PWM_FORWARD 19

#define R_EN_R 13
#define R_EN_F 12

#define R_PWM_REVERSE 27
#define R_PWM_FORWARD 14

uint16_t frequency = 12000;
uint8_t resolution = 8; //Resolution 8, 10, 12, 15
uint16_t max_resolution = 255;


/*
  #define MOTOR_A_EN_PIN 10
  #define MOTOR_B_EN_PIN 9
  #define MOTOR_A_IN_1_PIN 7
  #define MOTOR_A_IN_2_PIN 8
  #define MOTOR_B_IN_1_PIN 12
  #define MOTOR_B_IN_2_PIN 4
*/
// If one motor tends to spin faster than the other, add offset
#define MOTOR_A_OFFSET 0
#define MOTOR_B_OFFSET 0

// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
#define DECLINATION_ANGLE -0.100938208f  //33606 area code in radians

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 2.4f

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cart from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

//Mode 0 is not operational, 1 is FlySky, 2 if Follow Me, 3 is PurePursuit
int mode = 0;

/*
typedef struct FlySky {
  uint16_t x;
  uint16_t y;
  uint16_t swa;
  uint16_t swb;
  uint16_t swc;
};

FlySky flySky;
*/



typedef struct MotorDrive {
  uint8_t ldir;
  uint8_t lspeed;
  uint8_t rdir;
  uint8_t rspeed;
};

MotorDrive motorDrive;

String debugPrint;

//Accumulate debug info to be displayed at one time
void createDebugInfo(String debug, char printIt) {
#ifdef DEBUG
  return;
#endif
  debugPrint += debug + '\n';

  if (printIt == 'Y') {
    Serial.println(debugPrint);
    debugPrint = "";
  }
}

//Calculate Unix Time from NMEA date info
unsigned long unixTimestamp(int year, int month, int day,
                            int hour, int min, int sec)
{
  const short days_since_beginning_of_year[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

  int leap_years = ((year - 1) - 1968) / 4
                   - ((year - 1) - 1900) / 100
                   + ((year - 1) - 1600) / 400;

  long days_since_1970 = (year - 1970) * 365 + leap_years
                         + days_since_beginning_of_year[month - 1] + day - 1;

  if ( (month > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */

  return sec + 60 * ( min + 60 * (hour + 24 * days_since_1970));
}

// Overloaded Debug function
void createDebugInfo(String debug) {
  createDebugInfo(debug, 'N');
}

// Get the Machine ID from the ESP32
void getMacId() {
  uint64_t chipid = ESP.getEfuseMac();
  snprintf (cMacId, 16, "%04X%8x", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  createDebugInfo(cMacId);
  //Serial.println(cMacId);
}

void setDefaultJsonDocument() {
  getMacId();  //Get the Machine ID Esp32
  doc["macid"] = cMacId;
  doc["utime"] = 0;
  doc["lat"] = 0.0;
  doc["lon"] = 0.0;
  doc["dist"] = 0.0;
  doc["heading"] = 0;
  doc["bearing"] = 0;
  doc["fsX"] = 0;
  doc["fsY"] = 0;
  doc["fsSwa"] = 0;
  doc["fsSwb"] = 0;
  doc["fsSwc"] = 0;
  doc["lmd"] = 0;
  doc["rmd"] = 0;
  doc["lms"] = 0;
  doc["rms"] = 0;
  doc["ulf"] = 0;
  doc["urf"] = 0;
  doc["ulr"] = 0;
  doc["urr"] = 0;
  doc["olf"] = 0;
  doc["orf"] = 0;
  doc["olr"] = 0;
  doc["orr"] = 0;
}
