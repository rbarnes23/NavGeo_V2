#include "OTAUpdate.h"
#include "Definitions.h"
#include "Config.h"
#include "FlySky.h"
#include "Location.h"
#include "Mqtt.h"
#include <SimpleKalmanFilter.h>
#include "Imu.h"
#include "ArucoNav.h"
#include "US_sensors.h"
#include "Interrupts.h"

//Create Over the Air update object
OTA OTA(ssid, password, true);

//Create configuration object
Config config(geo);

void setup() {
  Serial.begin(115200);
  //Check for File updates
  //OTA.setup();


  //Default doc to use to store info to send
  setDefaultJsonDocument();

  //Interrupts for Odometer Readings
  attachInterrupts();

  //Setup Motor Controllers
  motor_LF = new Motor(L_EN_R, L_EN_F, L_PWM_REVERSE, L_PWM_FORWARD, frequency, resolution, 0, 1);
  motor_RF = new Motor(R_EN_R, R_EN_F, R_PWM_REVERSE, R_PWM_FORWARD, frequency, resolution, 2, 3);

  setupMqtt();  //MQTT - Also receives remote GPS info if available
  setupFlySky();//FLYSKY
  setupUS();  //UltraSonic Sensors
  setupImu();//Imu Compass
  setupLocation(); //GPS
  setupArucoNav();

  //addPath();

}

void loop() {
  //OTA.loop();
  loopMqtt();  //MQTT does not use freertos tasks so need to loop manually
  loopLocation(); //GPS
  loopArucoNav(); //Check for Aruco Markers
  // Get the course from gpsLocal to destination
  //  double course = radians(geo.courseTo(position.getX(), position.getY(), destination.getX(), destination.getY()));
  //  Motor::followMe(gpsLocal.getX(), gpsLocal.getY(), gpsRemote.getX(), gpsRemote.getY(), 10);
}

void addPath() {
  //Read the spiffs file to get configuration information
  //config.readFile(SPIFFS, "/mission.jsn");
  //SPIFFS.format();
  //config.readFile(SPIFFS, "/path.geojson");
  //geo.purePursuit(gpsLocal);
}
