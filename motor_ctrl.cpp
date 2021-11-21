#include "motor_ctrl.h"
#include <arduino.h>

CarkitMotor::CarkitMotor(uint8_t _id, uint8_t _pwm_pin, uint8_t _dir_pin) : id(_id), io({_pwm_pin, _dir_pin})
{
    pwm = 0;
    direction = DIR_FORWARD;
    ioInit();
}

uint8_t CarkitMotor::setMotorDir(uint8_t _dir)
{
    direction = _dir;
    digitalWrite(io.dir_pin, direction);
}

uint8_t CarkitMotor::setMotorPwm(uint16_t _pwm)
{
    pwm = _pwm;
    analogWrite(io.pwm_pin, (uint8_t)pwm);
}

uint8_t CarkitMotor::setMotorState(MotorState_t _state)
{
    state = _state;
    switch (state)
    {
    case MOTOR_POWER_DOWN:
        /* code */
        break;
    case MOTOR_POWER_UP:
        /* code */
        break;
    case MOTOR_STOP:
        analogWrite(io.pwm_pin, 0);
        break;
    case MOTOR_RUN:
        analogWrite(io.pwm_pin, (uint8_t)pwm);
        break;
    default:
        break;
    }
}

uint8_t CarkitMotor::ioInit()
{
    pinMode(io.pwm_pin, OUTPUT);
    pinMode(io.dir_pin, OUTPUT);
    analogWrite(io.pwm_pin, (uint8_t)pwm);
    digitalWrite(io.dir_pin, direction);
}
