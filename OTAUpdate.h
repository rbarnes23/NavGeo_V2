#ifndef __OTA_H__
#define __OTA_H__
#endif

#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

class OTA {
  public:
    // Constructor
    OTA(const char * ssid,const char * password,bool displayMsg);

    //Variables

    // Methods
    void setup();
    void loop();


  private:
    //Variables
    bool _displayMsg;
    const char* _ssid="Frontier9328";
    const char * _password="6164187456";
    //Methods

};
