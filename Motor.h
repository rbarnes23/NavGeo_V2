#ifndef __MOTOR_H__
#define __MOTOR_H__


#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID_v1.h>
#include "Geo.h"

struct _motorDrive {
  uint8_t ldir;
  uint8_t lspeed;
  uint8_t rdir;
  uint8_t rspeed;
} ;
static _motorDrive mDrive;


class Motor {
  public:

    // Constructor
    Motor(uint8_t motor_EN_L_pin = 0, uint8_t motor_EN_R_pin = 0, uint8_t motor_PWM_L_PIN = 0, uint8_t motor_PWM_R_pin = 0, uint16_t frequency = 0, uint8_t resolution = 0,  uint8_t channel_L = 0, uint8_t channel_R = 1);
    //Variables

    // Methods
    static _motorDrive getMotorDrive();

    void drive(uint8_t dir, uint16_t motorSpeed);
    //static void calculateMotorSpeeds(uint16_t pwm_X, uint16_t pwm_Y);
    static _motorDrive calculateMotorSpeeds(double direction, double speed, int source = 1 /*FlySky or actual*/, double maxSpeed = 10);
    void setMaxSpeed(double maxSpeed);
    void setup();
    void loop();
    static void followMe(double llat,double llon,double rlat,double rlon, uint16_t motorSpeed);
    static void followMe(PVector gpsLocal,PVector gpsRemote,uint16_t motorSpeed);
    static void TaskFollowMeCode( void * pvParameters );

  protected:
    static double _maxSpeed;


  private:
    //Variables
    uint8_t _motor_EN_L_pin;
    uint8_t _motor_EN_R_pin;
    uint8_t _motor_PWM_L_pin;
    uint8_t _motor_PWM_R_pin;
    uint16_t _frequency;
    uint8_t _resolution;
    uint8_t _pwmChannel_LEFT = 0;
    uint8_t _pwmChannel_RIGHT = 1;

    //Methods

};
#endif
