#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include <stdint.h>
#include <stdbool.h>
#include "AFMotor.h"

#define LEFT (0U)
#define RIGHT (1U)
#define MIDDLE (2U)
#define DIR_FORWARD (1U)
#define DIR_BACKWARD (2U)
#define MAX_MOTOR_SPEED (255U)
#define DEFAULT_PWM (200U)

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 2

typedef enum _MotorState_e
{
    MOTOR_POWER_DOWN,
    MOTOR_POWER_UP,
    MOTOR_STOP,
    MOTOR_RUN,

} MotorState_t;

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

    AF_DCMotor *motor;
    uint8_t mode;

public:
    CarkitMotor(uint8_t _id, uint8_t _pwm_pin, uint8_t _dir_pin);
    CarkitMotor(uint8_t _number, uint8_t _pwm = DEFAULT_PWM);
    uint8_t setMotorDir(uint8_t _dir);
    uint8_t setMotorSpeed(uint16_t _pwm);
    uint8_t setMotorState(MotorState_t _state);

private:
    uint8_t ioInit();
};
#endif // __MOTOR_CTRL_H__