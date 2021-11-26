#include "motor_ctrl.h"
#include <arduino.h>

CarkitMotor::CarkitMotor(uint8_t _id, uint8_t _pwm_pin, uint8_t _dir_pin) : id(_id), io({_pwm_pin, _dir_pin})
{
    pwm = 0;
    direction = DIR_FORWARD;
    ioInit();
}

CarkitMotor::CarkitMotor(uint8_t _number, uint8_t _pwm)
{
    motor = new AF_DCMotor(_number);
    pwm = _pwm;
    direction = DIR_FORWARD;
    motor->setSpeed(0);
    motor->run(RELEASE);
}

uint8_t CarkitMotor::setMotorDir(uint8_t _dir)
{
    direction = _dir;
    if (mode == 1)
        digitalWrite(io.dir_pin, direction);
}

uint8_t CarkitMotor::setMotorSpeed(uint16_t _pwm)
{
    pwm = _pwm;
    if (mode == 1)
        analogWrite(io.pwm_pin, (uint8_t)pwm);
    else
        motor->setSpeed(pwm);
}

uint8_t CarkitMotor::setMotorState(MotorState_t _state)
{
    state = _state;
    switch (state)
    {
    case MOTOR_POWER_DOWN:
    case MOTOR_POWER_UP:
        motor->setSpeed(0);
        motor->run(RELEASE);
        break;
    case MOTOR_STOP:
        if (mode == 1)
            analogWrite(io.pwm_pin, 0);
        else
        {
            motor->setSpeed(0);
            motor->run(RELEASE);
        }
        break;
    case MOTOR_RUN:
        if (mode == 1)
            analogWrite(io.pwm_pin, (uint8_t)pwm);
        else
        {
            motor->setSpeed(pwm);
            motor->run(direction);
        }
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
