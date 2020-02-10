#include <IBusBM.h>
#include <rosserial_msgs/FlySky.h>

TaskHandle_t TaskFS;

rosserial_msgs::FlySky flysky_msg;
ros::Publisher flysky_stat("nav/flysky_stat", &flysky_msg);

//TaskFlySkyCode: Get Flysky IBUS info every 20 ms
void TaskFlySkyCode( void * pvParameters ) {
  Serial.print("flysky_task running on core ");
  Serial.println(xPortGetCoreID());
  //Serial1 was changed to 4,2 in HardwareSerial.cpp
  HardwareSerial flysky(1);

  /* Block for 20ms. */
  const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

  IBusBM IBus;    // IBus object
  IBus.begin(flysky);

  //Loop Forever get Ibus data
  for (;;) {
    //IBus.loop();
    //Initialize to zero in case it is turned off
    doc["fsX"] = 0;
    doc["fsY"] = 0;
    doc["fsSwc"] = 0;
    flysky_msg.x0 = 0;
    flysky_msg.y1 = 0;
    flysky_msg.x2 = 0;
    flysky_msg.y3 = 0;
    flysky_msg.swa4 = 0;
    flysky_msg.swb5 = 0;
    flysky_msg.swc6 = 0;
    for (int i = 0; i <= 7; i++) {
      uint16_t reading =  IBus.readChannel(i);
      //Serial.println(reading);
      switch (i) {
        case 0:
          //flySky.x = reading;
          flysky_msg.x0 = reading;
          doc["fsX"] = reading;
          break;
        case 1:
          //flySky.y = reading;
          flysky_msg.y1 = reading;
          doc["fsY"] = reading;
          break;
        case 4:
          flysky_msg.swa4 = reading;
          doc["fsSwa"] = reading;
          break;
        case 5:
          flysky_msg.swb5 = reading;
          //flySky.swb = reading;
          doc["fsSwb"] = reading;
          break;

        case 7:
          //flySky.swc = reading;
          flysky_msg.y3 = reading;
          //Serial.println("7: "+String(reading));
          break;

        case 6:
          flysky_msg.swc6 = reading;

          static _motorDrive mDrive;
          switch (flysky_msg.swc6) {
            case 2000:  //FlySky Controller Mode
              mode = 1;
              mDrive = Motor::calculateMotorSpeeds(flysky_msg.x0, flysky_msg.y1, mode);
              doc["lmd"] = mDrive.ldir;
              doc["rmd"] = mDrive.rdir;
              doc["lms"] = mDrive.lspeed;
              doc["rms"] = mDrive.rspeed;
              motor_LF->drive(mDrive.ldir, mDrive.lspeed);
              motor_RF->drive(mDrive.rdir, mDrive.rspeed);
              break;
            case 1500:  //Follow Me Mode - Get Remote Coordinates
              doc["lmd"] = mDrive.ldir = 0;
              doc["rmd"] = mDrive.rdir = 0;
              doc["lms"] = mDrive.lspeed = 0;
              doc["rms"] = mDrive.rspeed = 0;
              if (flysky_msg.swa4 == 2000 && flysky_msg.swb5 == 1000) {
                mode = 2;//Follow Me
                float direction = (float) doc["heading"] - (float)doc["bearing"];
                direction = 0; //Put this here as heading and bearing were giving me ? #
                if (arucoDir != 0) {
                  direction = arucoDir;
                  //Serial.println("arucoDir: " + String(arucoDir));}
                }
                float e = atan2(sin(master.angleBetween(gpsRemote, gpsLocal)), cos(master.angleBetween(gpsRemote, gpsLocal)));
                float speed = 5;
                mDrive = Motor::calculateMotorSpeeds(direction, speed, mode, speed);
                motor_LF->drive(mDrive.ldir, mDrive.lspeed);
                motor_RF->drive(mDrive.rdir, mDrive.rspeed);
                doc["lmd"] = mDrive.ldir;
                doc["rmd"] = mDrive.rdir;
                doc["lms"] = mDrive.lspeed;
                doc["rms"] = mDrive.rspeed;
                //Serial.println("SWC: " + String(flysky_msg.swc6) + " SWA: " + String(flysky_msg.swa4) + " SWB: " + String(flysky_msg.swb5));
                //Serial.println("lDIR: " + String(mDrive.ldir) + " rDIR: " + String(mDrive.rdir) + " lSP: " + String(mDrive.lspeed) + " rSP: " + String(mDrive.rspeed) + " arucoDir: " + String(direction));
              } //Follow Me mode
              if (flysky_msg.swb5 == 2000 && flysky_msg.swa4 == 1000) {
                mode = 3;//Pure Pursuit
                float direction = -45;
                float speed = -10;
                geo.purePursuit(gpsLocal);
                mDrive = Motor::calculateMotorSpeeds(direction, speed, mode);
                motor_LF->drive(mDrive.ldir, mDrive.lspeed);
                motor_RF->drive(mDrive.rdir, mDrive.rspeed);
                doc["lmd"] = mDrive.ldir;
                doc["rmd"] = mDrive.rdir;
                doc["lms"] = mDrive.lspeed;
                doc["rms"] = mDrive.rspeed;
              } //Pure Pursuit
              if (flysky_msg.swb5 == 1000 && flysky_msg.swa4 == 1000) {
                motor_LF->drive(0, 0);
                motor_RF->drive(0, 0);
              }
              break;
            case 1000:
              motor_LF->drive(0, 0);
              motor_RF->drive(0, 0);
              break;
            default:
              motor_LF->drive(0, 0);
              motor_RF->drive(0, 0);
              break;
          }

          break;
        default:
          motor_LF->drive(0, 0);
          motor_RF->drive(0, 0);
          break;
      }
      doc["fsX"] = flysky_msg.x0;
      doc["fsY"] = flysky_msg.y1;
      doc["fsSwa"] = flysky_msg.swa4;
      doc["fsSwb"] = flysky_msg.swb5;
      doc["fsSwc"] = flysky_msg.swc6;
    }

    flysky_msg.header.frame_id = cMacId;
    flysky_msg.header.stamp = nh.now();
    flysky_stat.publish(&flysky_msg);
    nh.spinOnce();

    //delay(20);
    vTaskDelay(xDelay);  // one tick delay (15ms) in between reads for stability
  }
  /* delete a task when finish,
    this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}

void setupFlySky() {
  nh.advertise(flysky_stat);
  //FlySky task
  xTaskCreatePinnedToCore(&TaskFlySkyCode, "flysky_task", 10000, NULL, 1, &TaskFS, 0); //This needs to have priority over everything else so set to 3
  delay(500);
}
