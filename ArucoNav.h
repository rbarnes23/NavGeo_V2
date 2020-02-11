#include <geometry_msgs/Point.h>


unsigned long startTime;


//Maximum readings to use for the filter
const int MAX_READINGS = 3;
SimpleKalmanFilter kf_angle_dist(MAX_READINGS, MAX_READINGS, 0.02);

void messageCb( const geometry_msgs::Point &angle_dist_msg) {
  if (startTime - millis() > 100) {
    startTime = millis();
    //int dir = (int) angle_dist_msg.y;
    int dir = map(angle_dist_msg.y, -90.0, 90.0, -90, 90);
    arucoDir = kf_angle_dist.updateEstimate(dir);
    //Serial.println("Val: "+String(dir)+" Val1: "+String(arucoDir));
    //printf("Val %lf Val1 %lf\n", val, val1);
  }
}

ros::Subscriber<geometry_msgs::Point> aruco("aruco/angle_dist", &messageCb );

void setupArucoNav() {
  nh.subscribe(aruco);
  delay(200);
  startTime = millis();
}

void loopArucoNav() {
  arucoDir = 0;
  nh.spinOnce();
  delay(10);
}
