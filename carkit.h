#ifndef __CARKIT_H__
#define __CARKIT_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "motor_ctrl.h"
#include "carkit_map.h"
#include "log.h"

#define USE_TIMER_1 true
#include "TimerInterrupt.h"

#define CARKIT_OK 0
#define CARKIT_ERROR -1

#define USE_DEBUG 1
/**
 * TimerInterrupt macros
 *
 */
#define TIMER_INTERVAL_MS 10
#define TIMER_DURATION_MS 0

/**
 * Digital input pin for 3 line detection sensors
 */
#define DI_MIDDL_PIN A4
#define DI_LEFT_PIN A3
#define DI_RIGHT_PIN A5

/**
 * Analog input pin for 3 line detection sensors
 */
#define AI_MIDDLE_PIN A0
#define AI_LEFT_PIN A1
#define AI_RIGHT_PIN A2

#define MAX_IO_LINE_SENSOR 3
#define SAMPLE_INTERVAL_TIME 10

#define USE_SD_CARD 0
/**
 * TIMEOUT
 */
#define DEFAULT_SERIAL_TIMEOUT (100)

#define CHECK_LINE_SENSOR_TIMEOUT (5000)

#define DEFAULT_SPEED 130U
#define MAX_SPEED 180U
#define MIN_SPEED 110U
#define TURN_SPEED 110U
/**
 * @brief Cakit class
 *
 */
class Carkit
{
    enum CarState_e
    {
        CAR_STOP,
        CAR_RUN,
        CAR_GO_STRAIGHT,
        CAR_TURN_LEFT,
        CAR_TURN_RIGHT,
    };
    enum CarRunningState_e
    {
        CAR_IDLE,
        CAR_STARTED,
        CAR_FINISHED,
    };

public:
    Carkit();
    ~Carkit(){};

    int8_t Init();
    int8_t Start();
    int8_t ReStart();
    int8_t Stop();
    int8_t Reset();

    int8_t loop();
    static void CarkitTimerHandler(Carkit *carkit);

    int8_t GoStraight(uint8_t lSpeed, uint8_t rSpeed, uint8_t direction);
    int8_t GoLeft(uint8_t lSpeed, uint8_t rSpeed, uint32_t duration);
    int8_t GoRight(uint8_t lSpeed, uint8_t rSpeed, uint32_t duration);

private:
    void turn(uint8_t lSpeed, uint8_t lDir, uint8_t rSpeed, uint8_t rDir, uint32_t duration);

private:
    CarkitMotor *m_leftMotor;
    CarkitMotor *m_rightMotor;
    unsigned long m_tick;
    int32_t timeout;
    uint8_t *m_buffer;

    struct Car_t
    {
        CPoint_t currentNode; // current position
        CPoint_t *nextNode;
        volatile int8_t carStatus;
        volatile int8_t carState;
        volatile uint8_t carDir;
        volatile bool newCheckPointDetected;
        uint8_t nextTurn;
        CarkitMap map;
        int32_t waitTick;
        volatile uint8_t leftMotorSpeed;
        volatile uint8_t rightMotorSpeed;
        uint8_t leftDir, rightDir;

    } m_carkit;

    struct LineSensor_t
    {
    private:
        uint8_t di_pin;
        uint8_t ai_pin;
        uint8_t side;
        volatile bool detected;

    public:
        void init(int _di, int _ai, uint8_t _side)
        {
            di_pin = _di;
            ai_pin = _ai;
            side = _side;
            pinMode(di_pin, INPUT);
        }

        inline uint8_t diRead()
        {
            return digitalRead(di_pin);
        }

        inline uint16_t aiRead()
        {
            return analogRead(ai_pin);
        }
    } m_lineSensor[MAX_IO_LINE_SENSOR];

    void setMotorSpeed(uint8_t lSpeed = DEFAULT_PWM, uint8_t rSpeed = DEFAULT_PWM);
    void setMotorDirection(uint8_t lDir = DIR_FORWARD, uint8_t rDir = DIR_FORWARD);
    void setMotorState(uint8_t lState, uint8_t rState);
    int8_t turnLeft();
    int8_t turnRight();
    int8_t turnAround();
    int8_t goStraight();
    int8_t turnLeft(uint8_t speed);
    int8_t turnRight(uint8_t speed);
    int8_t timerInit();
};

#endif // __CARKIT_H__