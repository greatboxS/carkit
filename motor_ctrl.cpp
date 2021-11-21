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
}

uint8_t CarkitMotor::ioInit()
{
    pinMode(io.pwm_pin, OUTPUT);
    pinMode(io.dir_pin, OUTPUT);
    analogWrite(io.pwm_pin, (uint8_t)pwm);
    digitalWrite(io.dir_pin, direction);
}
