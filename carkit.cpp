#include "carkit.h"
#include <stdio.h>

#define LOG Serial
//
#define MIDDLE_SENSOR (m_lineSensor[0])
#define LEFT_SENSOR (m_lineSensor[1])
#define RIGHT_SENSOR (m_lineSensor[2])

#define _SIZE_ (m_tempPoint[0])
#define _X_ (m_tempPoint[1])
#define _Y_ (m_tempPoint[2])

Carkit::Carkit()
{
}

/**
 * @brief Init()
 * 
 * @return int8_t 
 */
int8_t Carkit::Init()
{
    LOG.begin(115200);
    LOG.setTimeout(DEFAULT_SERIAL_TIMEOUT);
    FLOG_I(F("Carkit initialized\n"), NULL);

    m_leftMotor = new CarkitMotor(LEFT_MOTOR);
    m_rightMotor = new CarkitMotor(RIGHT_MOTOR);

    m_leftMotor->setMotorState(MOTOR_POWER_UP);
    m_rightMotor->setMotorState(MOTOR_POWER_UP);

    MIDDLE_SENSOR.init(DI_MIDDL_PIN, AI_MIDDLE_PIN, MIDDLE);
    LEFT_SENSOR.init(DI_LEFT_PIN, AI_LEFT_PIN, LEFT);
    RIGHT_SENSOR.init(DI_RIGHT_PIN, AI_RIGHT_PIN, RIGHT);

    timerInit();

    // if no sd card support/ or exsit then load the map from EEPROM
    if (m_carkit.map.GetMapFromSDCard() != CARKIT_OK)
    {
        m_carkit.map.LoadMapFromEEPROM();
    }
    return 0;
}

/**
 * @brief Start()
 * 
 * @return int8_t 
 */
int8_t Carkit::Start()
{
    if (m_carkit.map.Size())
    {
        CPoint_t *node = m_carkit.map.GetNextPoint();
        if (node)
        {
            m_carkit.currentNode = *node;
            CPoint_t *nextNode = m_carkit.map.GetNextPoint();
            if (nextNode)
            {
                m_carkit.nextNode = nextNode;
                m_carkit.carStatus = CAR_STARTED;
                FLOG_I(F("Carkit is stated\n"), NULL);
                goto start;
            }
            else
            {
                m_carkit.carStatus = CAR_FINISHED;
            }
        }
    }
err:
    FLOG_I(F("Can not start the carkit\n"), NULL);
    return -1;

start:
    m_carkit.leftMotorSpeed = DEFAULT_PWM;
    m_carkit.rightMotorSpeed = DEFAULT_PWM;

    m_leftMotor->setMotorSpeed(m_carkit.leftMotorSpeed);
    m_rightMotor->setMotorSpeed(m_carkit.rightMotorSpeed);

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

int8_t Carkit::ReStart()
{
    m_carkit.carStatus = CAR_STARTED;
    m_carkit.carState = CAR_RUN;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

int8_t Carkit::Stop()
{
    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_STOP);
    m_carkit.carStatus = CAR_FINISHED;
    m_carkit.carState = CAR_STOP;
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

        if (m_carkit.carStatus == CAR_FINISHED)
            return 0;
        /**
         * User code here
         */
        // new check point detected
        if (LEFT_SENSOR.diRead() && MIDDLE_SENSOR.diRead() && MIDDLE_SENSOR.diRead() && !m_carkit.newCheckPointDetected)
        {
            m_carkit.newCheckPointDetected = true;

            // each tick == SAMPLE_INTERVAL_TIME (50ms)
            m_carkit.waitTick = 3; // 150ms
            // increase the current node position
            m_carkit.currentNode++;
            // start new move between two nodes
            if (m_carkit.currentNode == *m_carkit.nextNode)
            {
                m_carkit.currentNode = *m_carkit.nextNode;
                m_carkit.nextNode = m_carkit.map.GetNextPoint();

                // carkit finished the map
                if (m_carkit.nextNode == NULL)
                {
                    this->Stop();
                    return 0;
                }
            }
        }

        if (m_carkit.newCheckPointDetected && !(m_carkit.waitTick--))
        {
            m_carkit.newCheckPointDetected = false;
            switch (m_carkit.currentNode.turn)
            {
            case GO_STRAIGHT:
                goStraight();
                break;
            case GO_LEFT:
                turnLeft();
                break;
            case GO_RIGHT:
                turnRight();
                break;
            default:
                goStraight();
                break;
            }
        }
    }

    // if get a map from serial port successfully, save it to EEPROM
    if (m_carkit.map.GetMapFromSerialPort() == CARKIT_OK)
    {
        m_carkit.map.SaveMapToEEPROM();
    }

    return 0;
}

int8_t Carkit::turnLeft()
{
    int8_t detected = CARKIT_ERROR;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_LEFT;

    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_RUN);
    while (--timeout > 0 && !LEFT_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
    {
        while (--timeout > 0 && !MIDDLE_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
            detected = CARKIT_OK;
    }

    m_carkit.carState = CAR_RUN;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

int8_t Carkit::turnRight()
{
    int8_t detected = CARKIT_ERROR;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_RIGHT;

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    while (--timeout > 0 && !RIGHT_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
    {
        while (--timeout > 0 && !MIDDLE_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
            detected = CARKIT_OK;
    }

    m_carkit.carState = CAR_RUN;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

int8_t Carkit::turnAround()
{
    int8_t detected = CARKIT_ERROR;
    m_carkit.carState = CAR_TURN_RIGHT;

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    for (uint8_t i = 0; i < 2; i++)
    {
        timeout = CHECK_LINE_SENSOR_TIMEOUT;
        while (--timeout > 0 && !RIGHT_SENSOR.diRead())
        {
            delay(1);
        }
        if (timeout)
        {
            while (--timeout > 0 && !MIDDLE_SENSOR.diRead())
            {
                delay(1);
            }
            if (timeout)
                detected = CARKIT_OK;
        }
        if (detected == CARKIT_ERROR)
            break;
    }

    m_carkit.carState = CAR_RUN;
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return detected;
}

/**
 * @brief TimerInterrupt intializes
 * 
 * @return int8_t 
 */
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
    static uint8_t m_leftSpeed = 0;
    static uint8_t m_ritghtSpeed = 0;

    m_leftSpeed = carkit->m_carkit.leftMotorSpeed;
    m_ritghtSpeed = carkit->m_carkit.rightMotorSpeed;

#define M_SENSOR (carkit->m_lineSensor[0])
#define L_SENSOR (carkit->m_lineSensor[1])
#define R_SENSOR (carkit->m_lineSensor[2])

    if (carkit->m_carkit.carStatus != CAR_STARTED)
        return;

    // carkit is out of line
    // shift left
    // if left motor speed is not equal to max value, then increase it 5 value
    // else
    // decerease right motr speed 5
    if (!M_SENSOR.diRead() && L_SENSOR.diRead() && !R_SENSOR.diRead())
    {

        if (carkit->m_carkit.leftMotorSpeed < (MAX_MOTOR_SPEED - 5))
        {
            carkit->m_carkit.leftMotorSpeed += 5;
        }
        else
        {
            carkit->m_carkit.rightMotorSpeed -= 5;
        }
    }
    else if (!M_SENSOR.diRead() && R_SENSOR.diRead() && !L_SENSOR.diRead())
    {
        if (carkit->m_carkit.leftMotorSpeed < (MAX_MOTOR_SPEED - 5))
        {
            carkit->m_carkit.leftMotorSpeed += 2;
        }
        else
        {
            carkit->m_carkit.rightMotorSpeed -= 2;
        }
    }

    // carkit is out of line
    // shift right
    if (M_SENSOR.diRead() && R_SENSOR.diRead() && !L_SENSOR.diRead())
    {
        if (carkit->m_carkit.rightMotorSpeed < (MAX_MOTOR_SPEED - 5))
        {
            carkit->m_carkit.rightMotorSpeed += 5;
        }
        else
        {
            carkit->m_carkit.leftMotorSpeed -= 5;
        }
    }
    else if (M_SENSOR.diRead() && R_SENSOR.diRead() && !L_SENSOR.diRead())
    {
        if (carkit->m_carkit.leftMotorSpeed < (MAX_MOTOR_SPEED - 5))
        {
            carkit->m_carkit.leftMotorSpeed += 2;
        }
        else
        {
            carkit->m_carkit.rightMotorSpeed -= 2;
        }
    }

    if (carkit->m_carkit.carState != CAR_TURN_LEFT && carkit->m_carkit.carState != CAR_TURN_RIGHT)
    {
        if (m_leftSpeed != carkit->m_carkit.leftMotorSpeed)
            carkit->m_leftMotor->setMotorSpeed(carkit->m_carkit.leftMotorSpeed);

        if (m_ritghtSpeed != carkit->m_carkit.rightMotorSpeed)
            carkit->m_rightMotor->setMotorSpeed(carkit->m_carkit.rightMotorSpeed);
    }
}