#include "Motor.h"

//Begin Motor Class
Motor::Motor(uint8_t motor_EN_L_pin, uint8_t motor_EN_R_pin, uint8_t motor_PWM_L_pin, uint8_t motor_PWM_R_pin, uint16_t frequency, uint8_t resolution, uint8_t channel_L, uint8_t channel_R) {
  // Anything you need when instantiating your object goes here
  _motor_EN_L_pin = motor_EN_L_pin;
  _motor_EN_R_pin = motor_EN_R_pin;
  _motor_PWM_L_pin = motor_PWM_L_pin;
  _motor_PWM_R_pin = motor_PWM_R_pin;
  _frequency = frequency;
  _resolution = resolution;
  _pwmChannel_LEFT = channel_L;
  _pwmChannel_RIGHT = channel_R;
  pinMode(_motor_EN_L_pin, OUTPUT);
  pinMode(_motor_EN_R_pin, OUTPUT);
  pinMode(_motor_PWM_L_pin, OUTPUT);
  pinMode(_motor_PWM_R_pin, OUTPUT);
  ledcSetup(_pwmChannel_LEFT, _frequency, _resolution);
  ledcSetup(_pwmChannel_RIGHT, _frequency, _resolution);
  ledcAttachPin(_motor_PWM_L_pin, _pwmChannel_LEFT);
  ledcAttachPin(_motor_PWM_R_pin, _pwmChannel_RIGHT);
#ifndef DEBUG
  Serial.println("motor\t" + String(_motor_EN_L_pin) + "\t" + String(_motor_EN_R_pin) + "\t" + String(_motor_PWM_L_pin) + "\t" + String( _motor_PWM_R_pin));
#endif
}

void Motor::TaskFollowMeCode( void * pvParameters ) {
  //Serial.print("TaskFollowME running on core ");
  //Serial.println(xPortGetCoreID());
  //Serial.println(*((int*)pvParameters));
  Geo geo(10, 1, false);
  double course;// = radians(geo.courseTo(gpsLocal.getX(), gpsLocal.getY(), gpsRemote.getX(), gpsRemote.getY()));
  //Limits the direction between -180 and 180
  double e = degrees(atan2(sin(course), cos(course)));
  printf("COURSE %.4lf\n", e);
  mDrive = calculateMotorSpeeds(e, 10, 2, 10);
  vTaskDelete( NULL );
}

void Motor::followMe(PVector gpsLocal, PVector gpsRemote, uint16_t motorSpeed) {
  //Handle for location task
  TaskHandle_t TaskFollowMe;

  xTaskCreatePinnedToCore(
    TaskFollowMeCode,   /* Task function. */
    "TaskFollowMe",     /* name of task. */
    2048,       /* Stack size of task */
    (void*)&gpsRemote,        /* parameter of the task */
    2,           /* priority of the task */
    &TaskFollowMe,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  /*
    doc["lmd"] = mDrive.ldir;
    doc["rmd"] = mDrive.rdir;
    doc["lms"] = mDrive.lspeed;
    doc["rms"] = mDrive.rspeed;
  */
}

void Motor::followMe(double llat, double llon, double rlat, double rlon, uint16_t motorSpeed) {
  // Get the course from gpsLocal to destination

  Geo geo(10, 1, false);
  double course = radians(geo.courseTo(llat, llon, rlat, rlon));
  //Limits the direction between -180 and 180
  double e = degrees(atan2(sin(course), cos(course)));
  printf("COURSE %.4lf\n", e);

  calculateMotorSpeeds(e, 10, 2, 10);
  //printf("HERE%.4lf %.4lf %.4lf %.4lf %.4lf\n",llat,llon,rlat,rlon,motorSpeed);
}

void Motor::drive(uint8_t dir, uint16_t motorSpeed) {
  //Enable the motors if moving else disable them

  //Enable pins if not stopped
  if (dir != 0) {
    //    Serial.println("EN " + String(_motorNo));
    digitalWrite(_motor_EN_L_pin, HIGH);
    digitalWrite(_motor_EN_R_pin, HIGH);
  } else {
    //    Serial.println("NOTENABLED " + String(_motorNo));
    digitalWrite(_motor_EN_L_pin, LOW);
    digitalWrite(_motor_EN_R_pin, LOW);
  }

  //Set pwm for Bts7960
  switch (dir) {
    case 1:
      //FORWARD
      //Set the speed of each motor
      ledcWrite(_pwmChannel_LEFT, motorSpeed);
      ledcWrite(_pwmChannel_RIGHT, 0);
      //printf("MotorF: %i %lf\n", getMotorDrive(), motorSpeed);
      break;
    case 2:
      //BACKWARDS
      //Set the speed of each motor
      ledcWrite(_pwmChannel_LEFT, 0);
      ledcWrite(_pwmChannel_RIGHT, motorSpeed);
      //printf("MotorB: %i %lf\n", getMotorDrive(), motorSpeed);
      break;
    default:  //STOP
      ledcWrite(_pwmChannel_LEFT, 0);//LEFT MOTOR SPEED
      ledcWrite(_pwmChannel_RIGHT, 0);//RIGHT MOTOR SPEED
      //printf("MotorStop: %i %lf\n", getMotorDrive(), motorSpeed);
      break;
  }
}

_motorDrive Motor::getMotorDrive() {
  return mDrive;
};

void Motor::setMaxSpeed(double maxSpeed) {
  _maxSpeed = maxSpeed;
}

_motorDrive Motor::calculateMotorSpeeds(double direction, double speed, int source /*FlySky or actual*/, double maxSpeed)
{
  if (source == 2 || source == 3) {
    //direction must be between -90 and 90 converted to skyfy ranges of 1000 to 2000
    direction = map(direction, -180, 180, 1000, 2000);
    //speed is between 0 and _maxSpeed converted to skyfy ranges of 1000 to 2000  *****Need to do Something about speed when actual as between 1000 and 1500 is backwards and 1500 to 2000 forward
    speed = map(speed, -maxSpeed, maxSpeed, 1000, 2000);
  }
  // OUTPUTS
  double     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  double     nMotMixR;           // Motor (right) mixed output           (-128..+127)

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  double fPivYLimit = 32.0;

  // TEMP VARIABLES
  double   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  double   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  double   nPivSpeed;      // Pivot Speed                          (-128..+127)
  double   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
#ifdef DEBUG
  printf("Direction is %f and Speed is %f\n", direction, speed);
#endif
  // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
  int x = map(direction, 1000, 2000, -128, 128); //map our speed to 0-255 range
  int y = map(speed, 1000, 2000, -128, 128); //map our speed to 0-255 range

  // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
  //  int x = map(direction, 1000, 2000, -128, 127); //map our direction to 0-255 range
  //  int y = map(speed, 1000, 2000, -128, 127); //map our speed to 0-255 range
#ifdef DEBUG
  printf( "x y: % i % i\n", x, y);
#endif
  //Serial.println(y);
  // Calculate Drive Turn output due to Joystick X input
  if (y >= 0) {
    // Forward
    nMotPremixL = (x >= 0) ? 127.0 : (127.0 + x);
    nMotPremixR = (x >= 0) ? (127.0 - x) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (x >= 0) ? (127.0 - x) : 127.0;
    nMotPremixR = (x >= 0) ? 127.0 : (127.0 + x);
  }
  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * y / 128.0;
  nMotPremixR = nMotPremixR * y / 128.0;



  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = (int) x;
  double absY = (y > 0) ? (y * 1.0) : (y * -1.0);
  fPivScale = (absY > fPivYLimit) ? 0.0 : (1.0 - absY / fPivYLimit);
#ifdef DEBUG
  printf("nMotPremixL nMotPremixR, nPivSpeed, fPivScale: % lf % lf % lf %lf\n", nMotPremixL, nMotPremixR, nPivSpeed, fPivScale);
#endif
  // Calculate final mix of Drive and Pivot
  nMotMixL = lrint((1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed));
  nMotMixR = lrint((1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed));
#ifdef DEBUG
  printf("Final nMotMixL nMotMixR, nPivSpeed, fPivYLimit: % lf % lf % lf % lf\n", nMotMixL, nMotMixR, nPivSpeed, fPivScale);
#endif
  //Set Motor Direction
  //Create an instance of Motor Drive structure to return to use to control motors
  static _motorDrive mDrive;
  if (nMotMixL > 0) {
    mDrive.lspeed = map(nMotMixL, 1, 128, 1, 255);
    mDrive.ldir = 1;
  } else if (nMotMixL < 0) {
    mDrive.lspeed = map(abs(nMotMixL), 1, 128, 1, 255);
    mDrive.ldir = 2;
  } else {
    mDrive.lspeed = 0;
    mDrive.ldir = 0;
  }

  if (nMotMixR > 0) {
    mDrive.rspeed = map(nMotMixR, 1, 128, 1, 255);
    mDrive.rdir = 1;
  } else if (nMotMixL < 0) {
    mDrive.rspeed = map(abs(nMotMixR), 1, 128, 1, 255);
    mDrive.rdir = 2;
  } else {
    mDrive.rspeed = 0;
    mDrive.rdir = 0;
  }

  //Make absolutely sure they are in range
  constrain(nMotMixL, 0, 255);
  constrain(nMotMixR, 0, 255);
  //printf("mDrive: % i % i % i % i\n", mDrive.ldir, mDrive.lspeed, mDrive.rdir, mDrive.rspeed);
  return mDrive;
}



/*
  void Motor::calculateMotorSpeeds(uint16_t pwm_X, uint16_t pwm_Y)
  {
  // OUTPUTS
  int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 32.0;

  // TEMP VARIABLES
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

  // Map the X and Y axis to positive and negative numbers -  X for direction and Y for speed
  int x = map(pwm_X, 1000, 2000, -128, 127); //map our speed to 0-255 range
  int y = map(pwm_Y, 1000, 2000, -128, 127); //map our speed to 0-255 range

  //Serial.println(y);
  // Calculate Drive Turn output due to Joystick X input
  if (y >= 0) {
    // Forward
    nMotPremixL = (x >= 0) ? 127.0 : (127.0 + x);
    nMotPremixR = (x >= 0) ? (127.0 - x) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (x >= 0) ? (127.0 - x) : 127.0;
    nMotPremixR = (x >= 0) ? 127.0 : (127.0 + x);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * y / 128.0;
  nMotPremixR = nMotPremixR * y / 128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = x;
  fPivScale = (abs(y) > fPivYLimit) ? 0.0 : (1.0 - abs(y) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  //Set Motor Direction
  int lm, rm, leftMotorDir, rightMotorDir;
  if (nMotMixL > 0) {
    lm = map(nMotMixL, 1, 128, 1, 255);
    leftMotorDir = 1;
  } else if (nMotMixL < 0) {
    lm = map(abs(nMotMixL), 1, 128, 1, 255);
    leftMotorDir = 2;
  } else {
    lm = 0;
    leftMotorDir = 0;
  }

  if (nMotMixR > 0) {
    rm = map(nMotMixR, 1, 128, 1, 255);
    rightMotorDir = 1;
  } else if (nMotMixL < 0) {
    rm = map(abs(nMotMixR), 1, 128, 1, 255);
    rightMotorDir = 2;
  } else {
    rm = 0;
    rightMotorDir = 0;
  }

  //int leftMotorDir = (nMotMixL >= 0) ? 1 : -1;
  //int rightMotorDir = (nMotMixR >= 0) ? 1 : -1;

  // Change the pwm sent to the motor to expected values
  //lm = map(nMotMixL, -128, 128, 0, 255);
  //rm = map(nMotMixR, -128, 128, 0, 255);

  //Make absolutely sure they are in range
  //constrain(nMotMixL, 0, 255);
  //constrain(nMotMixR, 0, 255);

  mDrive.ldir = leftMotorDir;
  mDrive.lspeed = lm;
  mDrive.rdir = rightMotorDir;
  mDrive.rspeed = rm;

  //  doc["lmd"] = leftMotorDir;
  //  doc["rmd"] = rightMotorDir;
  //  doc["lms"] = lm;
  //  doc["rms"] = rm;
  //  motor_LF->drive(leftMotorDir, lm);
  //  motor_RF->drive(rightMotorDir, rm);

  //  lastcount_L_F = leftMotorDir;
  //  lastcount_R_F = rightMotorDir;
  //  lastcount_L_R = lm;
  //  lastcount_R_R = rm;

  //move_go(dir, nMotMixL, nMotMixR);
  }
*/
/*
  //Change this so it takes Current latlng and desired latlng and then creates a vector of magnitude and direction
  void Motor::calculateWheelVelocity(PVector current, PVector desired) {
  double Kp = 1, Ki = 1, Kd = 1;
  //Define Variables we'll be connecting to
  double Setpoint, Input, Output;
  desired.set(10, 180);
  float R = .127; //10 inch tires in meters
  float V = desired.getX();//Magnitude
  float O = radians(desired.getY());//current.angleRad(desired);
  //float O = -1 * desired; //PID This
  float L = .6604;
  float V_r = 2*V + (O * L) / (2 * R);
  float V_l = 2*V - (O * L) / (2 * R);
  PVector master;
  float e = atan2(sin(master.angleBetween(desired,current)), cos(master.angleBetween(desired,current)));

  printf( "VlM is %f and VrM is %f using speed %f and direction of %f\n", V_l, V_r, V, O);

  Setpoint = e;
  Input = e;
  PID errorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  //PID errorPID(e, Output, e, Kp, Ki, Kd, DIRECT);
  //PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  //turn the PID on
  //errorPID.SetMode(AUTOMATIC);
  //errorPID.Compute();

  Serial.println("ERROR: " + String(e, 6));
  V_r = map(V_r, -PI, PI, 0, 255);
  V_l = map(V_l, -PI, PI, 0, 255);
  //V_r = map(V_r, 0, 10, 0, 255);
  //V_l = map(V_l, 0, 10, 0, 255);

  //Serial.println("Vlb:" + String(V_l) + '\t' + "Vr:" + String(V_r));
    printf( "VlMb is %f and VrM is %f using speed %f and direction of %f\n", V_l, V_r, V, O);

  V_r = constrain(V_r, 0, 255);
  V_l = constrain(V_l, 0, 255);
*/
/*if (abs(V_l) - abs(V_r) < 1) {
  V_r = V_l;
  }*/
/*
  char turn;
  if (V_l > V_r)
  {
   turn = 'R';
  } else if (V_l < V_r) {
   turn = 'L';
  } else {
   turn = 'N';
  };
  Serial.print("WHEEL SPEEDS ");
  Serial.println("Vl:" + String(V_l) + '\t' + "Vr:" + String(V_r) + '\t' + "TURN " + String(turn));

  float angle = current.angleBetween(current,desired); // angle is 90
  Serial.print("Angle In Degrees ");
  Serial.println(String(angle, 0) + '\t');
  };
*/

void Motor::setup() {}

void Motor::loop() {}
