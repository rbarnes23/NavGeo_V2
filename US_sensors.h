#include <Ultrasonic.h>
//#include <RunningMedian.h>

#include <sensor_msgs/Range.h>

//initializing msg type to publish on ROS
sensor_msgs::Range rangeMsg0;
sensor_msgs::Range rangeMsg1;
sensor_msgs::Range rangeMsg2;
sensor_msgs::Range rangeMsg3;
#define FOV 0.5    //fake FOV
String frameid = "/sonar/";

ros::Publisher pub_sonar0( "sonar/ulf", &rangeMsg0);
ros::Publisher pub_sonar1( "sonar/urf", &rangeMsg1);
ros::Publisher pub_sonar2( "sonar/ulr", &rangeMsg2);
ros::Publisher pub_sonar3( "sonar/urr", &rangeMsg3);


#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
TaskHandle_t TaskUS;

void TaskUltrasonicCode( void * pvParameters ) {
  Serial.print("ultrasonic_task running on core ");
  Serial.println(xPortGetCoreID());

  /* Block for 100ms. */
  const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

  Ultrasonic ultrasonic_ulf(32, 34, 20000UL); // An ultrasonic sensor Left Front
  Ultrasonic ultrasonic_urf(33, 35, 20000UL); // An ultrasonic sensor Right Front
  Ultrasonic ultrasonic_ulr(25, 36, 20000UL); // An ultrasonic sensor left Rear
  Ultrasonic ultrasonic_urr(26, 39, 20000UL); // An ultrasonic sensor Right Rear
  //Maximum readings to use for the median
  const int MAX_READINGS = 3;
  SimpleKalmanFilter kf_ulf(MAX_READINGS, MAX_READINGS, 0.01);
  SimpleKalmanFilter kf_urf(MAX_READINGS, MAX_READINGS, 0.01);
  SimpleKalmanFilter kf_ulr(MAX_READINGS, MAX_READINGS, 0.01);
  SimpleKalmanFilter kf_urr(MAX_READINGS, MAX_READINGS, 0.01);

  //RunningMedian mulf = RunningMedian(MAX_READINGS);
  //RunningMedian murf = RunningMedian(MAX_READINGS);
  //RunningMedian mulr = RunningMedian(MAX_READINGS);
  //RunningMedian murr = RunningMedian(MAX_READINGS);

  for (;;) {
    int ulf = ultrasonic_ulf.read(CM);
    int urf = ultrasonic_urf.read(CM);
    int ulr = ultrasonic_ulr.read(CM);
    int urr = ultrasonic_urr.read(CM);
    ulf = constrain(ulf, 0, MAX_DISTANCE);
    urf = constrain(urf, 0, MAX_DISTANCE);
    ulr = constrain(ulr, 0, MAX_DISTANCE);
    urr = constrain(urr, 0, MAX_DISTANCE);

    //mulf.add(ulf);
    //murf.add(urf);
    //mulr.add(ulr);
    //murr.add(urr);

    int e_ulf = kf_ulf.updateEstimate(ulf);
    int e_urf = kf_urf.updateEstimate(urf);
    int e_ulr = kf_ulr.updateEstimate(ulr);
    int e_urr = kf_urr.updateEstimate(urr);

    //doc["ulf"] = mulf.getMedian();
    //doc["urf"] = murf.getMedian();
    //doc["ulr"] = mulr.getMedian();
    //doc["urr"] = murr.getMedian();
    doc["ulf"] = e_ulf;
    doc["urf"] = e_urf;
    doc["ulr"] = e_ulr;
    doc["urr"] = e_urr;
    rangeMsg0.header.stamp = rangeMsg1.header.stamp = rangeMsg2.header.stamp = rangeMsg3.header.stamp = nh.now();
    rangeMsg0.range = e_ulf;
    rangeMsg1.range = e_urf;
    rangeMsg2.range = e_ulr;
    rangeMsg3.range = e_urr;
    pub_sonar0.publish(&rangeMsg0);
    pub_sonar1.publish(&rangeMsg1);
    pub_sonar2.publish(&rangeMsg2);
    pub_sonar3.publish(&rangeMsg3);
    nh.spinOnce();
    vTaskDelay(xDelay);

  }

  /* delete a task when finish,
    this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}
void initRanges() {
  uint8_t rType = sensor_msgs::Range::ULTRASOUND;
  float minRange = 0.0;
  float maxRange = 1000.0;
  rangeMsg0.radiation_type = rType;
  rangeMsg0.header.frame_id =  "/sonar/ulf";
  rangeMsg0.field_of_view = FOV;
  rangeMsg0.min_range = minRange;
  rangeMsg0.max_range = maxRange;
  rangeMsg1.header.frame_id =  "/sonar/urf";
  rangeMsg1.field_of_view = FOV;
  rangeMsg1.min_range = minRange;
  rangeMsg1.max_range = maxRange;
  rangeMsg2.header.frame_id =  "/sonar/ulr";
  rangeMsg2.field_of_view = FOV;
  rangeMsg2.min_range = minRange;
  rangeMsg2.max_range = maxRange;
  rangeMsg3.header.frame_id =  "/sonar/urr";
  rangeMsg3.field_of_view = FOV;
  rangeMsg3.min_range = minRange;
  rangeMsg3.max_range = maxRange;

}

void setupUS() {
  nh.initNode();
  nh.advertise(pub_sonar0);
  nh.advertise(pub_sonar1);
  nh.advertise(pub_sonar2);
  nh.advertise(pub_sonar3);
  initRanges();
  xTaskCreatePinnedToCore(&TaskUltrasonicCode, "ultrasonic_task", 2048, NULL, 2, &TaskUS,0);
  delay(500);
}
