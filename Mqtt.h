#include <WiFi.h>
#include <PubSubClient.h>

#include "Config.h"

//GeoLoc gpsRemote;
TaskHandle_t TaskMqtt;
/* keep the status of sending data */
BaseType_t xStatus;
/* time to block the task until the queue has free space */
const TickType_t xTicksToWait = pdMS_TO_TICKS(20);

// Update these with values suitable for your network.
const char* ssid = "Frontier4720";
const char* password = "6253020202";
const char* mqtt_server = "96.31.86.129";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;

// Define Slave I2C Address
#define SLAVE_ADDR 23
// Define Slave answer size
#define TRANSMITSIZE 32

//FOR ROSSERIAL server and port and node handle
// Set the rosserial socket server IP address
IPAddress ros_socket_server(192, 168, 254, 100);

// Set the rosserial socket server port
const uint16_t ros_socket_server_Port = 11411;

// Set the connection to rosserial socket server
//ros::NodeHandle nh;
//nh.getHardware()->setConnection(server, serverPort);
//nh.initNode();

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());
  /*
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  */
}

// The callback is for receiving messages back from other sources
void callback(char* topic, byte* payload, unsigned int length) {
  //Config config(geo);
  //config.printHeap();
  //printf("TIO: %s\n",topic);
  DynamicJsonDocument jsonDoc(length * 1.6);

  DeserializationError err = deserializeJson(jsonDoc, payload);
  if (err.code() ==  DeserializationError::Ok) {
    gpsRemote.set(jsonDoc["lat"], jsonDoc["lon"]);
#ifdef DEBUG
    printf("PAYLOAD:  Lat: %.6lf Lon: %.6lf\n", gpsRemote.getX(), gpsRemote.getY());
#endif
  } else if (err.code() ==  DeserializationError::InvalidInput) {
    Serial.print(F("Invalid input!"));
  } else if (err.code() ==  DeserializationError::NoMemory) {
    Serial.print(F("No Memory!"));
  } else if (err.code() ==  DeserializationError::NotSupported) {
    Serial.print(F("Not Supported!"));
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
#ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    if (client.connect(cMacId)) {
#ifdef DEBUG
      Serial.println("connected");
#endif
      // Once connected, publish an announcement...
      client.publish(PUBLISHER, "Started Navigation Module");
      // ... and resubscribe
      client.subscribe(SUBSCRIBER);
    } else {
#ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
#endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setupMqtt() {
  //  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  //Serial.begin(115200);

  // Initialize I2C communications as Master
  Wire.begin();

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(ros_socket_server, ros_socket_server_Port);
  nh.initNode();

  delay(500);
}

void loopMqtt() {

  if (!client.connected()) {
    reconnect();
  }

  //   long now = millis();
  //   if (now - lastMsg >xDelay ) {
  //     lastMsg = now;
  if (gpsLocal.empty() == 0) {
    doc["bearing"] = (int) gpsLocal.courseTo(gpsLocal, gpsRemote);
    doc["dist"] = master.distanceBetween(gpsLocal, gpsRemote);
  }
  // Lastly, you can print the resulting JSON to a String
  String msg;
  size_t sz = serializeJson(doc, msg);

  String strs = "TT/NAV/" + String(cMacId);
  client.publish(strs.c_str(), msg.c_str());
  client.loop();
}
