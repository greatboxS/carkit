#ifndef __CARKIT_MAP_H__
#define __CARKIT_MAP_H__

#include <stdint.h>
#include <stdbool.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include "log.h"
#define EEPROM_BASE_ADDRESS 0x0F

#define MAP_FILE_NAME "map.txt"
#define SD_CHIP_SELECT_PIN 4
#define MAX_ROAD_POINT 200

/**             |north
 *              |
 * west ----------------- east
 *              |
 *              |south
 */

#define VIEW_NORTH 0
#define VIEW_EAST 1
#define VIEW_SOUTH 2
#define VIEW_WEST 3

#define GO_STRAIGHT 0
#define GO_LEFT 1
#define GO_RIGHT 2

struct _CPoint_t
{
    int8_t x;
    int8_t y;
    uint8_t view;  // south / north / west / east
    uint8_t state; // finished or not
    uint8_t turn;  // got straight /  left / right

    void init(uint8_t _view, uint8_t _turn)
    {
        state = 0;
        view = _view;
        turn = _turn;
    }
    void clear()
    {
        x = 0;
        y = 0;
    }

    _CPoint_t &operator=(const _CPoint_t &point)
    {
        memcpy(this, &point, sizeof(_CPoint_t));
        return *this;
    }

    bool operator>(const _CPoint_t &point)
    {
        if (this->x == point.y)
            return (this->y > point.y) ? true : false;
        else if (this->y == point.y)
            return (this->x > point.x) ? true : false;
        else
            return false;
    }

    _CPoint_t &operator++()
    {
        switch (this->view)
        {
        case VIEW_NORTH:
            if (this->turn != GO_STRAIGHT)
            {
                if (this->turn == GO_LEFT)
                    this->x--;
                else
                    this->x++;
            }
            else
                this->y++;
            break;
        case VIEW_EAST:
            if (this->turn != GO_STRAIGHT)
            {
                if (this->turn == GO_LEFT)
                    this->y++;
                else
                    this->y--;
            }
            else
                this->x++;
            break;
        case VIEW_SOUTH:
            if (this->turn != GO_STRAIGHT)
            {
                if (this->turn == GO_LEFT)
                    this->x--;
                else
                    this->x++;
            }
            else
                this->y--;
            break;
        case VIEW_WEST:
            if (this->turn != GO_STRAIGHT)
            {
                if (this->turn == GO_LEFT)
                    this->y--;
                else
                    this->y++;
            }
            else
                this->x--;
            break;
        default:
            this->x++;
            this->y++;
            break;
        }
        return *this;
    }

    bool operator==(const _CPoint_t &point)
    {
        return ((this->x == point.x) && (this->y == point.y));
    }
};

typedef struct _CPoint_t CPoint_t;

class CarkitMap
{
private:
    CPoint_t m_startP;
    CPoint_t m_endP;
    uint8_t m_roadPointsSize;
    uint8_t m_index;
    CPoint_t *m_roadPoints;
    uint8_t *m_buffer;
    File *m_mapFile;
    int8_t _X_, _Y_;
    uint8_t _SIZE_;
    int32_t m_timeout;

    int8_t fileReadLine(File *file, uint8_t *buff, uint8_t len);
    void setStartP(int8_t x, int8_t y);
    void setEndP(int8_t x, int8_t y);
    void clear();
    void freeRoadPoints();
    void *createRoadPoints(uint8_t size);
    void addPoint(int8_t x, int8_t y);
    int8_t preProcessMap();

public:
    CarkitMap();
    ~CarkitMap(){};

    int8_t GetMapFromSDCard();
    int8_t GetMapFromSerialPort();
    void SaveMapToEEPROM();
    void LoadMapFromEEPROM();
    CPoint_t &StartP();
    CPoint_t &EndP();
    uint8_t &Size();
    CPoint_t *PointList();
    CPoint_t *GetNextPoint();
};

#endif // __CARKIT_MAP_H__