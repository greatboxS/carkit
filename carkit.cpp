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
    m_carkit.leftMotorSpeed = DEFAULT_SPEED;
    m_carkit.rightMotorSpeed = DEFAULT_SPEED;
    // setMotorDirection(DIR_FORWARD, DIR_FORWARD);
    // setMotorSpeed(m_carkit.leftMotorSpeed, m_carkit.rightMotorSpeed);
    m_carkit.carState = CAR_RUN;
    m_carkit.carStatus = CAR_STARTED;
    // goStraight();
    // timerInit();

    GoStraight(115, 122, DIR_FORWARD);
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
    // FLOG_I(F("Sensor:  L(%d) M(%d) R(%d)\n"), LEFT_SENSOR.diRead(), MIDDLE_SENSOR.diRead(), RIGHT_SENSOR.diRead());
    // Check for carkit state every SAMPLE_INTERVAL_TIME (50ms) time
    if (millis() - m_tick > SAMPLE_INTERVAL_TIME)
    {
        if (m_carkit.waitTick >= 0)
            m_carkit.waitTick--;

        m_tick = millis();
    }

    if (m_carkit.carStatus == CAR_FINISHED)
        return 0;

    if (LEFT_SENSOR.diRead() && RIGHT_SENSOR.diRead() && MIDDLE_SENSOR.diRead() && m_carkit.waitTick < 0)
    {
        m_carkit.newCheckPointDetected = true;

        // each tick == SAMPLE_INTERVAL_TIME (10ms)
        m_carkit.waitTick = 100; // 500
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
                FLOG_I(F("Car arrived\n"), NULL);
                this->Stop();
                return 0;
            }
            this->Stop();

            FLOG_I(F("Car prepared for turnning\n"), NULL);
            switch (m_carkit.currentNode.turn)
            {
            case GO_STRAIGHT:
                // goStraight();
                GoStraight(115, 122, DIR_FORWARD);
                break;
            case GO_LEFT:
                delay(200);
                setMotorState(MOTOR_STOP, MOTOR_STOP);
                GoStraight(115, 122, DIR_BACKWARD);
                delay(50);
                GoLeft(120, 120, 250);
                // GoRight(130,130, 300);
                GoStraight(115, 122, DIR_FORWARD);

                // turnLeft(TURN_SPEED);
                // setMotorSpeed();
                // goStraight();
                break;
            case GO_RIGHT:
                delay(200);
                setMotorState(MOTOR_STOP, MOTOR_STOP);
                GoStraight(115, 122, DIR_BACKWARD);
                delay(50);
                GoRight(120, 120, 250);
                // GoLeft(130,130, 300);
                GoStraight(115, 122, DIR_FORWARD);
                // turnRight(TURN_SPEED);
                // setMotorSpeed();
                // goStraight();
                break;
            default:
                GoStraight(115, 122, DIR_FORWARD);
                // goStraight();
                break;
            }

            this->ReStart();
        }
    }

    // if get a map from serial port successfully, save it to EEPROM
    if (m_carkit.map.GetMapFromSerialPort() == CARKIT_OK)
    {
        m_carkit.map.SaveMapToEEPROM();
    }

    return 0;
}

int8_t Carkit::GoStraight(uint8_t lSpeed, uint8_t rSpeed, uint8_t direction)
{
    setMotorDirection(direction, direction);
    setMotorSpeed(lSpeed, rSpeed);
    setMotorState(MOTOR_RUN, MOTOR_RUN);
}

int8_t Carkit::GoLeft(uint8_t lSpeed, uint8_t rSpeed, uint32_t duration)
{
    setMotorState(MOTOR_STOP, MOTOR_STOP);
    turn(lSpeed, DIR_BACKWARD, rSpeed, DIR_FORWARD, duration);
    setMotorState(MOTOR_STOP, MOTOR_STOP);
    setMotorDirection(DIR_FORWARD, DIR_FORWARD);
}

int8_t Carkit::GoRight(uint8_t lSpeed, uint8_t rSpeed, uint32_t duration)
{
    setMotorState(MOTOR_STOP, MOTOR_STOP);
    turn(lSpeed, DIR_FORWARD, rSpeed, DIR_BACKWARD, duration);
    setMotorState(MOTOR_STOP, MOTOR_STOP);
    setMotorDirection(DIR_FORWARD, DIR_FORWARD);
}

void Carkit::turn(uint8_t lSpeed, uint8_t lDir, uint8_t rSpeed, uint8_t rDir, uint32_t duration)
{
    setMotorDirection(lDir, rDir);
    setMotorSpeed(lSpeed, rSpeed);
    setMotorState(MOTOR_RUN, MOTOR_RUN);
    delay(duration);
}

inline void Carkit::setMotorSpeed(uint8_t lSpeed, uint8_t rSpeed)
{
    m_leftMotor->setMotorSpeed(lSpeed);
    m_rightMotor->setMotorSpeed(rSpeed);
}

inline void Carkit::setMotorDirection(uint8_t lDir, uint8_t rDir)
{
    m_leftMotor->setMotorDir(lDir);
    m_rightMotor->setMotorDir(rDir);
}

inline void Carkit::setMotorState(uint8_t lState, uint8_t rState)
{
    m_leftMotor->setMotorState(lState);
    m_rightMotor->setMotorState(rState);
}

inline int8_t Carkit::turnLeft()
{
    int8_t detected = CARKIT_ERROR;
    int8_t carState = m_carkit.carState;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_LEFT;

    FLOG_I(F("turnLeft()\n"), NULL);

    setMotorSpeed(TURN_SPEED, TURN_SPEED);
    setMotorState(MOTOR_STOP, MOTOR_RUN);

    delay(300);

    while (!LEFT_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    setMotorState(MOTOR_RUN, MOTOR_STOP);

    delay(500);
    while (!MIDDLE_SENSOR.diRead() && LEFT_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    setMotorState(MOTOR_STOP, MOTOR_STOP);
    m_carkit.carState = carState;
    setMotorSpeed(m_carkit.leftMotorSpeed, m_carkit.rightMotorSpeed);
    return detected;
}

inline int8_t Carkit::turnRight()
{
    int8_t detected = CARKIT_ERROR;
    int8_t carState = m_carkit.carState;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_RIGHT;

    FLOG_I(F("turnRight()\n"), NULL);
    setMotorSpeed(TURN_SPEED, TURN_SPEED);
    setMotorState(MOTOR_RUN, MOTOR_STOP);

    delay(300);

    while (!RIGHT_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    setMotorState(MOTOR_STOP, MOTOR_RUN);

    delay(500);
    while (!MIDDLE_SENSOR.diRead() && !RIGHT_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    setMotorState(MOTOR_STOP, MOTOR_STOP);

    m_carkit.carState = carState;
    setMotorSpeed(m_carkit.leftMotorSpeed, m_carkit.rightMotorSpeed);
    return detected;
}

inline int8_t Carkit::turnAround()
{
    int8_t detected = CARKIT_ERROR;
    int carState = m_carkit.carState;
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
    setMotorState(MOTOR_RUN, MOTOR_RUN);
    m_carkit.carState = carState;
    return detected;
}

inline int8_t Carkit::goStraight()
{
    int8_t carState = m_carkit.carState;
    m_carkit.carState = CAR_GO_STRAIGHT;
    FLOG_I(F("goStraight()\n"), NULL);
    setMotorState(MOTOR_RUN, MOTOR_RUN);
    m_carkit.carState = carState;
    return 0;
}

inline int8_t Carkit::turnLeft(uint8_t speed)
{
    int8_t detected = CARKIT_ERROR;
    int8_t carState = m_carkit.carState;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_LEFT;

    FLOG_I(F("turnLeft()\n"), NULL);

    setMotorSpeed(speed, speed);
    setMotorDirection(DIR_BACKWARD, DIR_FORWARD);
    setMotorState(MOTOR_RUN, MOTOR_RUN);
    delay(300);

    while (!MIDDLE_SENSOR.diRead())
    {
        timeout -= 1;
        delay(1);
        if (timeout <= 0)
            break;
    }

    // setMotorSpeed(speed, speed);
    setMotorDirection(DIR_FORWARD, DIR_FORWARD);
    setMotorState(MOTOR_RUN, MOTOR_STOP);
    delay(100);

    while (!MIDDLE_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    // setMotorState(MOTOR_STOP, MOTOR_STOP);
    m_carkit.carState = carState;
    return detected;
}

inline int8_t Carkit::turnRight(uint8_t speed)
{
    int8_t detected = CARKIT_ERROR;
    int8_t carState = m_carkit.carState;
    timeout = CHECK_LINE_SENSOR_TIMEOUT;
    m_carkit.carState = CAR_TURN_LEFT;

    FLOG_I(F("turnLeft()\n"), NULL);

    setMotorSpeed(speed, speed);
    setMotorDirection(DIR_FORWARD, DIR_BACKWARD);
    setMotorState(MOTOR_RUN, MOTOR_RUN);
    delay(300);

    while (!MIDDLE_SENSOR.diRead())
    {
        timeout -= 1;
        delay(1);
        if (timeout <= 0)
            break;
    }

    // setMotorSpeed(speed, speed);
    setMotorDirection(DIR_FORWARD, DIR_FORWARD);
    setMotorState(MOTOR_STOP, MOTOR_RUN);
    delay(100);

    while (!MIDDLE_SENSOR.diRead())
    {
        timeout -= 10;
        delay(10);
        if (timeout <= 0)
            break;
    }

    // setMotorState(MOTOR_STOP, MOTOR_STOP);
    m_carkit.carState = carState;
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

#define ON_MIDDLE 1U
#define ON_HALF_LEFT 2U
#define ON_LEFT 3U
#define ON_HALF_RIGHT 4U
#define ON_RIGHT 5U
#define ON_FULL_LEFT 6U
#define ON_FULL_RIGHT 7U
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
    static uint8_t subSpeed = 0;

#define M_SENSOR (carkit->m_lineSensor[0])
#define L_SENSOR (carkit->m_lineSensor[1])
#define R_SENSOR (carkit->m_lineSensor[2])

    if (carkit->m_carkit.carState == CAR_TURN_LEFT || carkit->m_carkit.carState == CAR_TURN_RIGHT)
    {
        carkit->m_carkit.carDir = carkit->m_carkit.carState == CAR_TURN_LEFT ? ON_LEFT : ON_RIGHT;
    }
    else
    {
        if (M_SENSOR.diRead())
        {
            subSpeed = 0;
            carkit->m_carkit.carDir = ON_MIDDLE;
        }
        else if (!M_SENSOR.diRead() && !L_SENSOR.diRead() && R_SENSOR.diRead())
        {
            carkit->m_carkit.carDir = ON_LEFT;
            subSpeed = 5;
        }
        else if (!M_SENSOR.diRead() && !R_SENSOR.diRead() && L_SENSOR.diRead())
        {
            // carkit is out of line
            // shift right
            carkit->m_carkit.carDir = ON_RIGHT;
            subSpeed = 5;
        }
    }

    m_leftSpeed = carkit->m_carkit.leftMotorSpeed;
    m_ritghtSpeed = carkit->m_carkit.rightMotorSpeed;

    switch (carkit->m_carkit.carDir)
    {
    case ON_MIDDLE:
        // FLOG_I(F("Car Dir: ON MIDDLE\n"), NULL);
        // m_ritghtSpeed = m_leftSpeed = DEFAULT_SPEED;
        break;
    case ON_LEFT:
    case ON_HALF_LEFT:
        if (m_ritghtSpeed >= (MIN_SPEED + subSpeed))
        {
            m_ritghtSpeed -= subSpeed;
        }
        // else
        // {
        //     if (m_leftSpeed <= (MAX_SPEED - subSpeed))
        //     {
        //         m_leftSpeed += subSpeed;
        //     }
        // }
        FLOG_I(F("Car Dir: ON LEFT\n"), NULL);
        break;
    case ON_RIGHT:
    case ON_HALF_RIGHT:
        if (m_ritghtSpeed <= (MAX_SPEED - subSpeed))
        {
            m_ritghtSpeed += subSpeed;
        }
        // else
        // {
        //     if (m_leftSpeed > (MIN_SPEED + subSpeed))
        //     {
        //         m_leftSpeed -= subSpeed;
        //     }
        // }
        FLOG_I(F("Car Dir: ON RIGHT\n"), NULL);
        break;
    default:
        break;
    }

    if (carkit->m_carkit.carState == CAR_RUN && carkit->m_carkit.waitTick < 0)
    {
        FLOG_I(F("Set car speed (l,r): %u, %u\n"), m_leftSpeed, m_ritghtSpeed);
        carkit->m_carkit.leftMotorSpeed = m_leftSpeed;
        carkit->m_carkit.rightMotorSpeed = m_ritghtSpeed;
        carkit->setMotorSpeed(m_leftSpeed, m_ritghtSpeed);
    }
}
