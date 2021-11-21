#include "carkit.h"
#include <stdio.h>

#define LOG Serial
//
#define MIDDLE_LINE_SENSOR (m_lineSensor[0])
#define LEFT_LINE_SENSOR (m_lineSensor[1])
#define RIGHT_LINE_SENSOR (m_lineSensor[2])

#define _SIZE_ (m_tempPoint[0])
#define _X_ (m_tempPoint[1])
#define _Y_ (m_tempPoint[2])

#define NEXT_NODE (m_carkitMap.PointList()[m_currentCar.nodeIndex + 1])
#define CURRENT_NODE (m_currentCar.position)
#define CAR (m_currentCar)

Carkit::Carkit()
{
}

int8_t Carkit::Init()
{
    LOG.begin(115200);
    LOG.setTimeout(DEFAULT_SERIAL_TIMEOUT);
    LOG.println(F("Carkit initialized"));

    m_leftMotor = new CarkitMotor(LEFT, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN);
    m_rightMotor = new CarkitMotor(RIGHT, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);

    m_leftMotor->setMotorState(MOTOR_POWER_UP);
    m_rightMotor->setMotorState(MOTOR_POWER_UP);

    MIDDLE_LINE_SENSOR.init(DI_MIDDL_PIN, AI_MIDDLE_PIN, MIDDLE);
    LEFT_LINE_SENSOR.init(DI_LEFT_PIN, AI_LEFT_PIN, LEFT);
    RIGHT_LINE_SENSOR.init(DI_RIGHT_PIN, AI_RIGHT_PIN, RIGHT);

    timerInit();

    m_carkitMap.GetMapFromSDCard();
    return 0;
}

int8_t Carkit::Start()
{
    if (m_carkitMap.Size())
    {
        m_currentCar.position = &m_carkitMap.PointList()[0];
        m_currentCar.nodeIndex = 0;
    }

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

int8_t Carkit::ReStart()
{
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

int8_t Carkit::Stop()
{
    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_STOP);
    return 0;
}

int8_t Carkit::Reset()
{
    return 0;
}

/**
 * @brief loop()
 * All function is done here,
 * Checking in the perior time SAMPLE_INTERVAL_TIME
 * 
 * @return int8_t 
 */
inline int8_t Carkit::loop()
{
    if (millis() - m_tick > SAMPLE_INTERVAL_TIME)
    {
        m_tick = millis();

        if (m_carkitMap.Size() > 0)
        {
        }
    }

    m_carkitMap.GetMapFromSerialPort();
    return 0;
}

int8_t Carkit::turnLeft()
{
    int8_t detected = CARKIT_ERROR;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_RUN);
    while (--timeout > 0 && !LEFT_LINE_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
    {
        while (--timeout > 0 && !MIDDLE_LINE_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
            detected = CARKIT_OK;
    }

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

int8_t Carkit::turnRight()
{
    int8_t detected = CARKIT_ERROR;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    while (--timeout > 0 && !RIGHT_LINE_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
    {
        while (--timeout > 0 && !MIDDLE_LINE_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
            detected = CARKIT_OK;
    }

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

int8_t Carkit::turnAround()
{
    int8_t detected = CARKIT_ERROR;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    for (uint8_t i = 0; i < 2; i++)
    {
        timeout = CHECK_LINE_SENSOR_TIMEOUT;
        while (--timeout > 0 && !RIGHT_LINE_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
        {
            while (--timeout > 0 && !MIDDLE_LINE_SENSOR.diRead())
            {
                delay(1);
            }
            if (timeout)
                detected = CARKIT_OK;
        }
        if (detected == CARKIT_ERROR)
            break;
    }

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

int8_t Carkit::timerInit()
{
    ITimer1.init();
    // Using ATmega328 used in UNO => 16MHz CPU clock ,
    if (ITimer1.attachInterruptInterval<Carkit *>(TIMER_INTERVAL_MS, Carkit::CarkitTimerHandler, this))
    {
        Serial.println("Starting  ITimer1 OK");
    }
    else
        return -1;
    return 0;
}

int8_t Carkit::goStraight()
{
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

int8_t Carkit::findDirection()
{
    if (CURRENT_NODE->x == NEXT_NODE.x)
    {
        switch (CURRENT_NODE->view)
        {
        case VIEW_NORTH:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = VIEW_NORTH;
            break;

        case VIEW_SOUTH:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = VIEW_SOUTH;
            break;

        case VIEW_EAST:
            if (CURRENT_NODE->y > NEXT_NODE.y)
            {
                CAR.nextTurn = CAR_TURN_LEFT;
                NEXT_NODE.view = VIEW_NORTH;
            }
            else
            {
                CAR.nextTurn = CAR_TURN_RIGHT;
                NEXT_NODE.view = VIEW_SOUTH;
            }
            break;

        case VIEW_WEST:
            if (CURRENT_NODE->y > NEXT_NODE.y)
            {
                CAR.nextTurn = CAR_TURN_RIGHT;
                NEXT_NODE.view = VIEW_NORTH;
            }
            else
            {
                CAR.nextTurn = CAR_TURN_LEFT;
                NEXT_NODE.view = VIEW_SOUTH;
            }
            break;

        default:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = CURRENT_NODE->view;
            break;
        }
    }
    else if (CURRENT_NODE->y == NEXT_NODE.y)
    {
        switch (CURRENT_NODE->view)
        {
        case VIEW_NORTH:
            if (CURRENT_NODE->x > NEXT_NODE.x)
            {
                CAR.nextTurn = CAR_TURN_RIGHT;
                NEXT_NODE.view = VIEW_EAST;
            }
            else
            {
                CAR.nextTurn = CAR_TURN_LEFT;
                NEXT_NODE.view = VIEW_WEST;
            }
            break;

        case VIEW_SOUTH:
            if (CURRENT_NODE->x > NEXT_NODE.x)
            {
                CAR.nextTurn = CAR_TURN_LEFT;
                NEXT_NODE.view = VIEW_EAST;
            }
            else
            {
                CAR.nextTurn = CAR_TURN_RIGHT;
                NEXT_NODE.view = VIEW_WEST;
            }
            break;

        case VIEW_EAST:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = VIEW_EAST;
            break;

        case VIEW_WEST:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = VIEW_WEST;
            break;

        default:
            CAR.nextTurn = CAR_STRAIGHT;
            NEXT_NODE.view = CURRENT_NODE->view;
            break;
        }
    }
    else
    {
    }
}

static void Carkit::CarkitTimerHandler(Carkit *carkit)
{
    carkit->m_carState = 1;
    // Serial.println("Timer handler");
}