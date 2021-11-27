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

#if USE_SD_CARD
    // if no sd card support/ or exsit then load the map from EEPROM
    if (m_carkit.map.GetMapFromSDCard() != CARKIT_OK)
    {
        m_carkit.map.LoadMapFromEEPROM();
    }
#else
    m_carkit.map.LoadMapFromEEPROM();
#endif
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

    goStraight();
    timerInit();
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
    FLOG_I(F("Car finished\n"), NULL);
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
    // FLOG_I(F("Sensor:  L(%d) M(%d) R(%d)\n"), LEFT_SENSOR.diRead(), MIDDLE_SENSOR.diRead(), RIGHT_SENSOR.diRead());
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
        if (LEFT_SENSOR.diRead() && RIGHT_SENSOR.diRead() && m_carkit.waitTick < 0)
        {
            timeout = 100;
            while (!MIDDLE_SENSOR.diRead() && timeout-- > 0)
            {
                delay(1);
            }
            if (timeout <= 0)
                return 0;
            m_carkit.newCheckPointDetected = true;

            // each tick == SAMPLE_INTERVAL_TIME (50ms)
            m_carkit.waitTick = 20; // 500
            // increase the current node position
            m_carkit.currentNode++;
            FLOG_I(F("Current node: (%d, %d)\n"), m_carkit.currentNode.x, m_carkit.currentNode.y);

            // start new move between two nodes
            if (m_carkit.currentNode == *m_carkit.nextNode)
            {
                FLOG_I(F("-----> Check point\n"), NULL);
                m_carkit.currentNode = *m_carkit.nextNode;
                m_carkit.nextNode = m_carkit.map.GetNextPoint();

                // carkit finished the map
                if (m_carkit.nextNode == NULL)
                {
                    this->Stop();
                    return 0;
                }

                FLOG_I(F("Car prepared for turnning\n"), NULL);
                switch (m_carkit.currentNode.turn)
                {
                case GO_STRAIGHT:
                    goStraight();
                    break;
                case GO_LEFT:
                    turnLeft();
                    delay(10);
                    goStraight();
                    break;
                case GO_RIGHT:
                    turnRight();
                    delay(10);
                    goStraight();
                    break;
                default:
                    goStraight();
                    break;
                }
            }
        }
        m_carkit.waitTick--;
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

    FLOG_I(F("turnLeft()\n"), NULL);

    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_RUN);
    delay(200);
    while (--timeout > 0 && !LEFT_SENSOR.diRead())
    {
        delay(1);
    }

    while (--timeout > 0 && !MIDDLE_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
        detected = CARKIT_OK;

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    delay(200);
    while (--timeout > 0 && (MIDDLE_SENSOR.diRead() || RIGHT_SENSOR.diRead()))
    {
        delay(1);
    }
    if (timeout)
        detected = CARKIT_OK;

    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_STOP);
    return detected;
}

int8_t Carkit::turnRight()
{
    int8_t detected = CARKIT_ERROR;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_RIGHT;

    FLOG_I(F("turnRight()\n"), NULL);

    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_STOP);
    delay(200);
    while (--timeout > 0 && !RIGHT_SENSOR.diRead())
    {
        delay(1);
    }

    while (--timeout > 0 && !MIDDLE_SENSOR.diRead())
    {
        delay(1);
    }
    if (timeout)
        detected = CARKIT_OK;

    m_rightMotor->setMotorState(MOTOR_RUN);
    m_leftMotor->setMotorState(MOTOR_STOP);
    delay(200);
    while (--timeout > 0 && (MIDDLE_SENSOR.diRead() || LEFT_SENSOR.diRead()))
    {
        delay(1);
    }
    if (timeout)
        detected = CARKIT_OK;

    m_leftMotor->setMotorState(MOTOR_STOP);
    m_rightMotor->setMotorState(MOTOR_STOP);
    return detected;
}

int8_t Carkit::turnAround()
{
    int8_t detected = CARKIT_ERROR;
    m_carkit.carState = CAR_TURN_RIGHT;

    FLOG_I(F("turnAround()\n"), NULL);

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
    FLOG_I(F("goStraight()\n"), NULL);
    m_leftMotor->setMotorState(MOTOR_RUN);
    m_rightMotor->setMotorState(MOTOR_RUN);
    return 0;
}

#define ON_MIDDLE 1
#define ON_HALF_LEFT 2
#define ON_LEFT 3
#define ON_HALF_RIGHT 4
#define ON_RIGHT 5
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
    static uint8_t m_carDir = 0;
    uint8_t subSpeed = 0;

    m_leftSpeed = carkit->m_carkit.leftMotorSpeed;
    m_ritghtSpeed = carkit->m_carkit.rightMotorSpeed;

#define M_SENSOR (carkit->m_lineSensor[0])
#define L_SENSOR (carkit->m_lineSensor[1])
#define R_SENSOR (carkit->m_lineSensor[2])

    if (carkit->m_carkit.carStatus != CAR_STARTED)
        return;

    if (M_SENSOR.diRead() && !L_SENSOR.diRead() && !R_SENSOR.diRead())
    {
        m_carDir = ON_MIDDLE;
    }
    if (M_SENSOR.diRead() && L_SENSOR.diRead() && R_SENSOR.diRead())
    {
        m_carDir = ON_MIDDLE;
    }

    if (!M_SENSOR.diRead() && !L_SENSOR.diRead() && R_SENSOR.diRead())
    {
        // carkit is out of line
        // shift left
        // if left motor speed is not equal to max value, then increase it 5 value
        // else
        // decerease right motr speed 5
        m_carDir = ON_LEFT;
        subSpeed = 10;
    }
    else if (M_SENSOR.diRead() && R_SENSOR.diRead() && !L_SENSOR.diRead())
    {
        m_carDir = ON_HALF_LEFT;
        subSpeed = 5;
    }
    else if (!M_SENSOR.diRead() && !R_SENSOR.diRead() && L_SENSOR.diRead())
    {
        // carkit is out of line
        // shift right
        m_carDir = ON_RIGHT;
        subSpeed = 10;
    }
    else if (M_SENSOR.diRead() && !R_SENSOR.diRead() && L_SENSOR.diRead())
    {
        m_carDir = ON_HALF_RIGHT;
        subSpeed = 5;
    }
    else
    {
        ;
    }

    if (carkit->m_carkit.carState == CAR_RUN)
    {
        switch (m_carDir)
        {
        case ON_MIDDLE:
            // FLOG_I(F("Car Dir: ON MIDDLE\n"), NULL);
            carkit->m_carkit.leftMotorSpeed = carkit->m_carkit.rightMotorSpeed = DEFAULT_PWM;
            break;
        case ON_LEFT:
        case ON_HALF_LEFT:
            // FLOG_I(F("Car Dir: ON LEFT\n"), NULL);
            if (carkit->m_carkit.rightMotorSpeed >= (MIN_SPEED + subSpeed))
            {
                carkit->m_carkit.rightMotorSpeed -= subSpeed;
            }
            break;
        case ON_RIGHT:
        case ON_HALF_RIGHT:
            // FLOG_I(F("Car Dir: ON RIGHT\n"), NULL);
            if (carkit->m_carkit.rightMotorSpeed <= (MAX_SPEED + subSpeed))
            {
                carkit->m_carkit.rightMotorSpeed += subSpeed;
            }
            break;
        default:
            break;
        }

        carkit->m_leftMotor->setMotorSpeed(carkit->m_carkit.leftMotorSpeed);
        carkit->m_rightMotor->setMotorSpeed(carkit->m_carkit.rightMotorSpeed);
    }
}