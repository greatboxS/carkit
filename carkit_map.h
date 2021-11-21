#ifndef __CARKIT_MAP_H__
#define __CARKIT_MAP_H__

#include <stdint.h>
#include <stdbool.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>

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

typedef struct _CPoint_t
{
    int8_t x;
    int8_t y;
    uint8_t view;
    void clear()
    {
        x = 0;
        y = 0;
    }
} CPoint_t;

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

public:
    CarkitMap();
    ~CarkitMap(){};

    CPoint_t &StartP();
    CPoint_t &EndP();
    uint8_t &Size();
    CPoint_t *PointList();
    int8_t GetMapFromSDCard();
    void GetMapFromSerialPort();
};

#endif // __CARKIT_MAP_H__