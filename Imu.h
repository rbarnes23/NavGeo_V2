#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <DFRobot_BMP280.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
TaskHandle_t TaskImu;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********

BMP   bmp(&Wire, BMP::eSdo_low);

//#define SEA_LEVEL_PRESSURE    1024.10f   // sea level pressure
#define SEA_LEVEL_PRESSURE    1013.23f   // sea level pressure

char frameid_imu[] = "/world";
char child_imu[] = "/imu_frame";

//Publishers

geometry_msgs::TransformStamped t;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_stat_pub("imu_status", &imu_msg);

geometry_msgs::Vector3 bmp280_info;
ros::Publisher bmp280_pub("bmp280_status", &bmp280_info);

geometry_msgs::Vector3 rpy_msg;
ros::Publisher rpy_pub("rpy_stat", &rpy_msg);

double getRoll(double q0, double q1, double q2, double q3) {
  return -atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
}
double getPitch(double q0, double q1, double q2, double q3) {
  return asin(2 * (q0 * q2 - q3 * q1));
}
double getYaw(double q0, double q1, double q2, double q3) {
  return -atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) - PI / 2;
}

double getHeading(double q0, double q1, double q2, double q3, float mag_decl) {
  double hDegrees = (getYaw(q0, q1, q2, q3) + mag_decl) * RAD_TO_DEG;
  (hDegrees < 0) ? (hDegrees += 360) : hDegrees; //degrees after Compass Adjustment could be more than 360 or less than 0
  (hDegrees > 360) ? (hDegrees -= 360) : hDegrees;
  return hDegrees;
}



//TaskCompassCode: Get compass info every 100ms
void TaskImuCode( void * pvParameters ) {
  Serial.print("TaskImu running on core ");
  Serial.println(xPortGetCoreID());
  /* Block for 100ms. */
  const TickType_t xDelay = 20 / portTICK_PERIOD_MS;
  /* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)


  // Check alt,hum,temp sensor
  bmp.reset();
  while (bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    //printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(false);

  // Set the connection to rosserial socket server
  nh.initNode();
  // Start to be polite
  broadcaster.init(nh);
  nh.advertise(rpy_pub);
  nh.advertise(bmp280_pub);
  nh.advertise(imu_stat_pub);
  //setupFlySky();//FLYSKY


  int hDegrees = 0;
  int compassAdjustment = 0;
  for (;;) {
    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (nh.connected()) {
      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      //Assigning values to TF header and publish the transform
      t.header.frame_id = frameid_imu;
      t.child_frame_id = child_imu;
      t.transform.translation.x = 0.0;
      t.transform.rotation.x = quat.x();
      t.transform.rotation.y = quat.y();
      t.transform.rotation.z = quat.z();
      t.transform.rotation.w = quat.w();
      t.header.stamp = nh.now();
      broadcaster.sendTransform(t);
      imu_msg.header.frame_id = "imu";
      imu_msg.header.stamp = nh.now();
      imu_msg.orientation.x = quat.x();
      imu_msg.orientation.y = quat.y();
      imu_msg.orientation.z = quat.z();
      imu_msg.orientation.w = quat.w();
      imu_msg.angular_velocity.x = gyroscope.x();
      imu_msg.angular_velocity.y = gyroscope.y();
      imu_msg.angular_velocity.z = gyroscope.z();

      imu_msg.linear_acceleration.x = linear_accel.x();
      imu_msg.linear_acceleration.y = linear_accel.y();
      imu_msg.linear_acceleration.z = linear_accel.z();

      imu_stat_pub.publish(&imu_msg);
      rpy_msg.x = getRoll(quat.w(), quat.x(), quat.y(), quat.z());
      rpy_msg.y = getPitch(quat.w(), quat.x(), quat.y(), quat.z());
      //rpy_msg.z=getYaw(quat.w(),quat.x(),quat.y(),quat.z());
      rpy_msg.z = getHeading(quat.w(), quat.x(), quat.y(), quat.z(), -0.100938208f);
      rpy_pub.publish(&rpy_msg);
    } else {
      Serial.println("Not Connected");
    }

    nh.spinOnce();
    vTaskDelay(xDelay);  // one tick delay (15ms) in between reads for stability
  }
  /* delete a task when finish,
    this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}



void setupImu() {
  Wire.begin(SDA_PIN, SCL_PIN);
  //delay(500);
  //Compass task that will be executed in the TaskCompassCode() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    TaskImuCode,   /* Task function. */
    "TaskImu",     /* name of task. */
    4096,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &TaskImu,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);

}
