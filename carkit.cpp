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

Carkit::Carkit()
{
}

int8_t Carkit::Init()
{
    LOG.begin(115200);
    LOG.setTimeout(DEFAULT_SERIAL_TIMEOUT);
    FLOG_I(F("Carkit initialized\n"), NULL);

    m_leftMotor = new CarkitMotor(LEFT_MOTOR);
    m_rightMotor = new CarkitMotor(RIGHT_MOTOR);

    m_leftMotor->setMotorState(MOTOR_POWER_UP);
    m_rightMotor->setMotorState(MOTOR_POWER_UP);

    MIDDLE_LINE_SENSOR.init(DI_MIDDL_PIN, AI_MIDDLE_PIN, MIDDLE);
    LEFT_LINE_SENSOR.init(DI_LEFT_PIN, AI_LEFT_PIN, LEFT);
    RIGHT_LINE_SENSOR.init(DI_RIGHT_PIN, AI_RIGHT_PIN, RIGHT);

    timerInit();

    // if no sd card support/ or exsit then load the map from EEPROM
    if (m_carkitMap.GetMapFromSDCard() != CARKIT_OK)
    {
        m_carkitMap.LoadMapFromEEPROM();
    }
    return 0;
}

int8_t Carkit::Start()
{
    LOG_I("Start\n");
    if (m_carkitMap.Size())
    {
        m_car.position = &m_carkitMap.PointList()[0];
        m_car.nodeIndex = 0;
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
    // Check for carkit state every SAMPLE_INTERVAL_TIME (50ms) time
    if (millis() - m_tick > SAMPLE_INTERVAL_TIME)
    {
        m_tick = millis();

        /**
         * User code here
         */
    }

    // if get a map from serial port successfully, save it to EEPROM
    if (m_carkitMap.GetMapFromSerialPort() == CARKIT_OK)
    {
        m_carkitMap.SaveMapToEEPROM();
    }

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
        FLOG_I(F("Starting  ITimer1 OK\n"), NULL);
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

/**
 * @brief CarkitTimerHandler
 * Timer 1 interrupt, this timer used for async status checking,
 * or line detection algorithm
 * @param carkit 
 */
static void Carkit::CarkitTimerHandler(Carkit *carkit)
{
    static int8_t m_newPointDetected = -1;
    static int8_t m_carTurnNow = -1;
    
    FLOG_I(F("CarkitTimerHandler %ld\n"), millis());
}