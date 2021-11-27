#include "motor_ctrl.h"
#include <arduino.h>
#include "log.h"

CarkitMotor::CarkitMotor(uint8_t _id, uint8_t _pwm_pin, uint8_t _dir_pin) : id(_id), io({_pwm_pin, _dir_pin})
{
    pwm = 0;
    mode = 1;
    direction = DIR_FORWARD;
    ioInit();
}

CarkitMotor::CarkitMotor(uint8_t _number, uint8_t _dir)
{
    id = _number;
    motor = new AF_DCMotor(id);
    pwm = 0;
    direction = _dir;
    motor->setSpeed((uint8_t)pwm);
    motor->run(RELEASE);
}

uint8_t CarkitMotor::setMotorDir(uint8_t _dir)
{
    direction = _dir;
    if (mode == 1)
        digitalWrite(io.dir_pin, direction);
    return 0;
}

uint8_t CarkitMotor::setMotorSpeed(uint16_t _pwm)
{
    pwm = _pwm;
    if (mode == 1)
        analogWrite(io.pwm_pin, (uint8_t)pwm);
    else
        motor->setSpeed((uint8_t)pwm);
    return 0;
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
            motor->run(direction);
            motor->setSpeed(0);
        }
        break;
    case MOTOR_RUN:
        if (mode == 1)
            analogWrite(io.pwm_pin, (uint8_t)pwm);
        else
        {
            FLOG_I(F("%s motor run %s with speed = %u\n"),
                   (id == LEFT_MOTOR ? "Left" : "Right"),
                   (direction == FORWARD ? "forward" : "backward"),
                   pwm);

            motor->setSpeed((uint8_t)pwm);
            motor->run(direction);
        }
        break;
    default:
        break;
    }
    return 0;
}

uint8_t CarkitMotor::ioInit()
{
    pinMode(io.pwm_pin, OUTPUT);
    pinMode(io.dir_pin, OUTPUT);
    analogWrite(io.pwm_pin, (uint8_t)pwm);
    digitalWrite(io.dir_pin, direction);
    return 0;
}
