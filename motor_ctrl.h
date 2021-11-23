#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include <stdint.h>
#include <stdbool.h>
#include "AFMotor.h"

#define LEFT (0U)
#define RIGHT (1U)
#define MIDDLE (2U)
#define DIR_FORWARD (0U)
#define DIR_BACKWARD (1U)

typedef enum _MotorState_e
{
    MOTOR_POWER_DOWN,
    MOTOR_POWER_UP,
    MOTOR_STOP,
    MOTOR_RUN,

}MotorState_t;

class CarkitMotor
{
private:
    uint8_t id;
    uint8_t direction;
    uint16_t pwm;
    MotorState_t state;
    struct io_ctrl
    {
        uint8_t pwm_pin;
        uint8_t dir_pin;
    } io;

public:
    CarkitMotor(uint8_t _id, uint8_t _pwm_pin, uint8_t _dir_pin);
    uint8_t setMotorDir(uint8_t _dir);
    uint8_t setMotorPwm(uint16_t _pwm);
    uint8_t setMotorState(MotorState_t _state);

private:
    uint8_t ioInit();

};
#endif // __MOTOR_CTRL_H__