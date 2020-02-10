#include <TinyGPS++.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
// The TinyGPS++ object
TinyGPSPlus gps;

//initializing msg type to publish on ROS
sensor_msgs::NavSatFix fix_local;

char frameidLoc[] = "/world";
char childLoc[] = "/gps";

//Publishers

geometry_msgs::TransformStamped tfs;
tf::TransformBroadcaster broadcaster;

ros::Publisher pub_navsatfix( "nav/fix_local", &fix_local);

//Queue to send gps info
//xQueueHandle xQueueGPS;

//Handle for location task
TaskHandle_t TaskLocation;

// The serial connection to the GPS device
HardwareSerial gps_dev(2);
//GeoLoc gpsLocal;

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gps_dev.available())
      gps.encode(gps_dev.read());
  } while (millis() - start < ms);
}


static ulong getUnixTime(TinyGPSPlus &gps) {
  ulong  utime = unixTimestamp(gps.date.year(), gps.date.month(), gps.date.day(),
                               gps.time.hour(), gps.time.minute(), gps.time.second()) ;
  return utime;
}

void loopLocation() {
  smartDelay(GPS_UPDATE_INTERVAL);
  //Need local and remote to make calculations.  Remote comes from callback function of mqtt
  if (fabs(gps.location.lat()) > 0) {
    gpsLocal.set( gps.location.lat(), gps.location.lng());
    fix_local.longitude = gpsLocal.getX();
    fix_local.latitude = gpsLocal.getY();
    fix_local.altitude = gps.altitude.meters();
    //Assigning values to TF header and publish the transform
    double x1, y1 = 0.0;
    int zone = geo.LatLonToUTMXY(gpsLocal.getX(), gpsLocal.getY(), 0, x1, y1);
    tfs.header.stamp = nh.now();
    tfs.header.frame_id = frameidLoc;
    tfs.child_frame_id = childLoc;
    tfs.transform.rotation.x = .071884;
    tfs.transform.rotation.y = -.05488;
    tfs.transform.rotation.z = .0;
    tfs.transform.rotation.w = .996;
    tfs.transform.translation.x = x1;
    tfs.transform.translation.y = y1;
    tfs.transform.translation.z = zone;


    broadcaster.sendTransform(tfs);
  }

  //Do a gpsRemote take from mqtt to get latest gpsRemote data
  fix_local.header.stamp = nh.now();
  fix_local.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

  pub_navsatfix.publish(&fix_local);
  nh.spinOnce();
}

void setupLocation() {
  nh.initNode();
  nh.advertise(pub_navsatfix);
  broadcaster.init(nh);
  gps_dev.begin(GPSBaud);//, SERIAL_8N1, 16, 17, 0);
  delay(500);
}
