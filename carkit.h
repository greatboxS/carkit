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
#define TIMER_INTERVAL_MS 50
#define TIMER_DURATION_MS 0

/**
 * Digital input pin for 3 line detection sensors
 */
#define DI_MIDDL_PIN 6
#define DI_LEFT_PIN 7
#define DI_RIGHT_PIN 8

/**
 * Analog input pin for 3 line detection sensors
 */
#define AI_MIDDLE_PIN A0
#define AI_LEFT_PIN A1
#define AI_RIGHT_PIN A2

#define MAX_IO_LINE_SENSOR 3
#define SAMPLE_INTERVAL_TIME 50

/**
 * TIMEOUT
 */
#define DEFAULT_SERIAL_TIMEOUT (100)

#define CHECK_LINE_SENSOR_TIMEOUT (10000)
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
    };

    enum CarTurn_e
    {
        CAR_STRAIGHT,
        CAR_TURN_LEFT,
        CAR_TURN_RIGHT,
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

private:
    uint8_t m_carState;
    CarkitMotor *m_leftMotor;
    CarkitMotor *m_rightMotor;
    unsigned long m_tick;
    int32_t timeout;
    CarkitMap m_carkitMap;
    uint8_t *m_buffer;

    struct Car_t
    {
        CPoint_t *position;  // current position
        uint8_t nodeIndex;   // current node index of node list
        uint8_t nextDir;     // next direction
        uint8_t curDir;      // current direction
        uint8_t nextTurn;    // next turning
        int8_t newNodeFound; // indicates if new node is found
    } m_car;

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

    int8_t turnLeft();
    int8_t turnRight();
    int8_t turnAround();
    int8_t timerInit();
    int8_t goStraight();
};

#endif // __CARKIT_H__