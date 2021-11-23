/**
 * @file main.ino
 * @author greatboxs <https://github.com/greatboxs>
 * @brief 
 * @version 0.1
 * @date 2021-11-21
 * 
 * @libraries:  - TimerInterrupt v1.7.0
 *              - Adafruit-Motor-Shield-library v1.0.1
 *              - NewPing v1.9.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <arduino.h>
#include "main.h"
#include "carkit.h"

Carkit carkit;

void setup()
{
    carkit.Init();
}

void loop()
{
    carkit.loop();
}