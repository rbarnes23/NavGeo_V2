#ifndef __CONFIG_H__
#define __CONFIG_H__


#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <FS.h>
#include <SPIFFS.h>

//Spiffs
#define FORMAT_SPIFFS_IF_FAILED true

extern bool saveWp;


class Config {
  public:
    // Constructor
    /**
      Constructor for Config class.
    */
    Config(Geo geo) {}


bool first = true;

void saveWayPoint(PVector wp) {
  File wpFile;
  if (saveWp) {
    wpFile = SPIFFS.open("/wpx.json", "w");
    if (!wpFile) {
      Serial.println("failed to open config file for writing");
    }  else {
      if (first) {
        wpFile.print("[");
        first = false;
      }
    }
    wpFile.println(String(wp.getX(), 8) + "," + String(wp.getY(), 8) + "," + String(wp.getZ(), 8));

    wpFile.close();
    // Serialize JSON to file
    //if (serializeJson(jsonDoc, configFile) == 0) {
    //  Serial.println(F("Failed to write to file"));
    // }
    //    json.printTo(Serial);
    //json.printTo(configFile);
    //    configFile.close();
    //end save
  }  else {
    wpFile.print("[");
    wpFile.close();
  }

}

static void printHeap() {
  Serial.print(F("Heap size: "));
  Serial.println(ESP.getFreeHeap());
}


static void startSPIFFS() {
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  printHeap();
}

static void stopSPIFFS() {
  SPIFFS.end();
  printHeap();
}

//Reads files from missionplanner, google earth kmz or json file from robot on mapmymotion
static size_t readFile(fs::FS & fs, const char * path) {
//size_t readFile(fs::FS & fs, const char * path) {
  startSPIFFS();
  size_t filesize = 0;
  size_t jsonsize = 0;
  Serial.println("Reading file: " + String(path));
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return filesize;
  }

  Serial.println("- read from file:");
  filesize = file.size();
  if ( filesize == 0 ) {
    Serial.println("Config file empty !");
  } else {
    std::unique_ptr<char[]> payload(new char[filesize]);

    file.readBytes(payload.get(), filesize);
    file.close();
    stopSPIFFS();
    //Serial.println(payload.get());

    //Increase file size a bit to make sure
    filesize++;// *= 1.02;
    printf("FileSize: %i\n", filesize);

    DynamicJsonDocument jsonDoc(filesize);
    DeserializationError err = deserializeJson(jsonDoc, payload.get());
    if (err.code() ==  DeserializationError::Ok) {
      Serial.println(F("OK"));
      if (path == "/mission.jsn") {//This is a missionplanner xml file converted to json via https://www.freeformatter.com/xml-to-json-converter.html
        jsonsize = jsonDoc["MISSIONITEM"].size();
        for (int i = 0; i < jsonsize; i++) {
          //double no = jsonDoc["MISSIONITEM"][i]["no"];
          double lat = jsonDoc["MISSIONITEM"][i]["lat"];
          double lon = jsonDoc["MISSIONITEM"][i]["lon"];
          PVector p(lat, lon, 0);
          geo.addPoint(p);
        }
      } else if (path == "/path.geojson") { //This is from a kmz file converted to geojson via https://mygeodata.cloud/converter/kmz-to-json
        jsonsize = jsonDoc["features"][0]["geometry"]["coordinates"].size();

        for (int i = 0; i < jsonsize; i++) {
          double lat = jsonDoc["features"][0]["geometry"]["coordinates"][i][1];
          double lon = jsonDoc["features"][0]["geometry"]["coordinates"][i][0];
          PVector p(lat, lon, 0);
          geo.addPoint(p);
        }
      } else if (path == "/wp.jsn") {
        // extract the values
        JsonArray array = jsonDoc.as<JsonArray>();
        for (JsonVariant v : array) {
          double lat = v["lat"];
          double lon = v["lon"];
          double alt = v["alt"];
          PVector p(lat, lon, alt);
          geo.addPoint(p);
          printf("XXX %.10lf\n", p.getX());
        }
      }

    } else if (err.code() ==  DeserializationError::InvalidInput) {
      printf("Invalid input!");
    } else if (err.code() ==  DeserializationError::NoMemory) {
      printf("No Memory!");
    } else if (err.code() ==  DeserializationError::NotSupported) {
      printf("Not Supported!");
    }
  }
  return geo.getEnd();//jsonsize;
}

static void readConfigFile(fs::FS & fs, const char * path) {
  Serial.printf("Reading file: % s\r\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(" - failed to open file for reading");
    return;
  }

  Serial.println(" - read from file : ");
  size_t size = file.size();
  DynamicJsonDocument jsonDoc(size + 1);
  if ( size == 0 ) {
    Serial.println("Config file empty !");
  } else {
    std::unique_ptr<char[]> payload(new char[size]);

    file.readBytes(payload.get(), size);

    DeserializationError err = deserializeJson(jsonDoc, payload.get());
    if (err.code() ==  DeserializationError::Ok) {
      JsonObject jsonPayload = jsonDoc.as<JsonObject>();
      //      publisher = jsonDoc["pub"].as<String>();
      //      subscriber = jsonDoc["sub"].as<String>();
      //      mqtt_server = jsonDoc["mqtt"].as<String>();
      const char * ss = jsonDoc["ssid"].as<char*>();
      const char * pass = jsonDoc["pwd"].as<char*>();
      //const char *mqtt = jsonDoc["mqtt"].as<char*>();

      //      ssid=(char*)ss;
      //      password=(char*)pass;
      //mqtt_server = (char*)mqtt;
      //      Serial.println(String(ssid) + " " + String(pass) + " " + String(mqtt_server));
    } else if (err.code() ==  DeserializationError::InvalidInput) {
      Serial.print(F("Invalid input!"));
    } else if (err.code() ==  DeserializationError::NoMemory) {
      Serial.print(F("No Memory!"));
    } else if (err.code() ==  DeserializationError::NotSupported) {
      Serial.print(F("Not Supported!"));
    }
  }
}
    
  protected:
  private:
};
#endif
