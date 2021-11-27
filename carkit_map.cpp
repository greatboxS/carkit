#include <stddef.h>
#include "carkit_map.h"
#include "log.h"

#define LOG Serial
#define DEFAULT_BUFFER_SIZE (256)
#define GET_MAP_TIMEOUT (20000)
#ifndef USE_DEBUG
#define USE_DEBUG 1
#endif

#define NODE(x) (m_roadPoints[x])

void CarkitMap::setStartP(int8_t x, int8_t y)
{
    m_startP.x = x;
    m_startP.y = y;
}

void CarkitMap::setEndP(int8_t x, int8_t y)
{
    m_endP.x = x;
    m_endP.y = y;
}

void CarkitMap::clear()
{
    m_index = 0;
    m_startP.clear();
    m_endP.clear();
    freeRoadPoints();
}

void CarkitMap::freeRoadPoints()
{
    if (m_roadPoints)
    {
        free(m_roadPoints);
        m_roadPoints = NULL;
    }
}

void *CarkitMap::createRoadPoints(uint8_t size)
{
    m_roadPointsSize = size;
    freeRoadPoints();
    m_roadPoints = (CPoint_t *)calloc(size, sizeof(CPoint_t));
    return m_roadPoints;
}

void CarkitMap::addPoint(int8_t x, int8_t y)
{
    if ((m_index < m_roadPointsSize) && m_roadPoints)
    {
        m_roadPoints[m_index].x = x;
        m_roadPoints[m_index].y = y;
        m_index++;
    }
}

int8_t CarkitMap::preProcessMap()
{
    FLOG_I(F("preProcessMap\n"), NULL);
    if (m_roadPointsSize > 0)
    {
        for (uint8_t i = 0; i < m_roadPointsSize; i++)
        {
            if (i == m_roadPointsSize - 1)
                break;

            // In the vertical line
            if (NODE(i).x == NODE(i + 1).x)
            {
                // if the first, then init the view of start point
                // point ahead to next node
                if (i == 0)
                {
                    if (NODE(i).y < NODE(i + 1).y)
                        NODE(i).view = VIEW_NORTH;
                    else
                        NODE(i).view = VIEW_SOUTH;
                }

                switch (NODE(i).view)
                {
                case VIEW_NORTH:
                    NODE(i + 1).init(VIEW_NORTH, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;
                    break;

                case VIEW_SOUTH:
                    NODE(i + 1).init(VIEW_SOUTH, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;
                    break;

                case VIEW_EAST:
                    if (NODE(i + 1).y > NODE(i).y)
                    {
                        NODE(i + 1).init(VIEW_NORTH, GO_LEFT);
                        NODE(i).turn = GO_LEFT;
                    }
                    else
                    {
                        NODE(i + 1).init(VIEW_SOUTH, GO_RIGHT);
                        NODE(i).turn = GO_RIGHT;
                    }
                    break;

                case VIEW_WEST:
                    if (NODE(i + 1).y > NODE(i).y)
                    {
                        NODE(i + 1).init(VIEW_NORTH, GO_RIGHT);
                        NODE(i).turn = GO_RIGHT;
                    }
                    else
                    {
                        NODE(i + 1).init(VIEW_SOUTH, GO_LEFT);
                        NODE(i).turn = GO_LEFT;
                    }
                    break;

                default:
                    NODE(i + 1).init(NODE(i).view, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;
                    break;
                }
            }
            // in the horizontal line
            else if (NODE(i).y == NODE(i + 1).y)
            {
                if (i == 0)
                {
                    if (NODE(i + 1).x > NODE(i).x)
                        NODE(i).view = VIEW_EAST;
                    else
                        NODE(i).view = VIEW_WEST;
                }

                switch (NODE(i).view)
                {
                case VIEW_NORTH:
                    if (NODE(i + 1).x > NODE(i).x)
                    {
                        NODE(i + 1).init(VIEW_EAST, GO_RIGHT);
                        NODE(i).turn = GO_RIGHT;
                    }
                    else
                    {
                        NODE(i + 1).init(VIEW_WEST, GO_LEFT);
                        NODE(i).turn = GO_LEFT;
                    }
                    break;

                case VIEW_SOUTH:
                    if (NODE(i + 1).x > NODE(i).x)
                    {
                        NODE(i + 1).init(VIEW_EAST, GO_LEFT);
                        NODE(i).turn = GO_LEFT;
                    }
                    else
                    {
                        NODE(i + 1).init(VIEW_WEST, GO_RIGHT);
                        NODE(i).turn = GO_RIGHT;
                    }
                    break;

                case VIEW_EAST:
                    NODE(i + 1).init(VIEW_EAST, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;

                    break;

                case VIEW_WEST:
                    NODE(i + 1).init(VIEW_WEST, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;
                    break;

                default:
                    NODE(i + 1).init(NODE(i).view, GO_STRAIGHT);
                    NODE(i).turn = GO_STRAIGHT;
                    break;
                }
            }
            else
            {
            }

            FLOG_I(F("Node [%d]: (%d,%d), view: %d, turn: %d\n"), i, NODE(i).x, NODE(i).y, NODE(i).view, NODE(i).turn);
        }
    }
    return 0;
}

int8_t CarkitMap::fileReadLine(File *file, uint8_t *buff, uint8_t len)
{
    uint8_t index = 0;
    int val = -1;
    if (!buff)
        return -1;

    if (!file)
        return -1;

    memset(m_buffer, 0, len);

    while (m_mapFile->available())
    {
        val = m_mapFile->read();
        if (index >= len || (char)val == '\n' || val == -1)
            break;
        buff[index++] = (uint8_t)val;
    }
    return index;
}

/**`
 * @brief Public function
 */
CarkitMap::CarkitMap()
{
}

/**
 * Get the map from serial port
 * MAP definition:
 * 
 * start[%d,%d], end[%d,%d], len[%d]\n
 * eg: start[0,1], end[5,7], len[5]\n
 */
int8_t CarkitMap::GetMapFromSDCard()
{
    // see if the card is present and can be initialized:
    if (!SD.begin(SD_CHIP_SELECT_PIN))
    {
        FLOG_I(F("Card failed, or not present\n"), NULL);
        return -1;
    }

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    m_mapFile = (File *)calloc(1, sizeof(File));
    if (!m_mapFile)
        return -1;
    *m_mapFile = SD.open(MAP_FILE_NAME);

    // if the file is available, write to it:
    if (m_mapFile)
    {
        m_buffer = new uint8_t[DEFAULT_BUFFER_SIZE];
        while (m_mapFile->available())
        {
            if (fileReadLine(m_mapFile, m_buffer, DEFAULT_BUFFER_SIZE))
            {
                /**
                 * start of transaction
                 * query all header information and start to loop all the points of map
                 */
                if (strlen((const char *)m_buffer) > 0)
                {
                    sscanf((const char *)m_buffer, "%u\n", &m_roadPointsSize);
#if USE_DEBUG
                    FLOG_I(F("Total point = %d\n"), m_roadPointsSize);
#endif
                    for (uint8_t i = 0; i < m_roadPointsSize; i++)
                    {
                        if (fileReadLine(m_mapFile, m_buffer, DEFAULT_BUFFER_SIZE))
                        {
                            char *begin = strchr((const char *)m_buffer, '(');
                            if (begin)
                            {
                                sscanf((const char *)begin, "(%d, %d)\n", &_X_, &_Y_);
                                addPoint(_X_, _Y_);
#if USE_DEBUG
                                FLOG_I(F("Add new point: (%d, %d)\n"), _X_, _Y_);
                            }
#endif
                        }
                        else
                            break;
                    }
                }
            }
        }
        m_mapFile->close();
    }
    else
        return -1;

    preProcessMap();

    delete m_buffer;
    m_buffer = NULL;
    free(m_mapFile);
    m_mapFile = NULL;

    return m_roadPointsSize;
}

int8_t CarkitMap::GetMapFromSerialPort()
{
    int8_t ret = -1;
    uint8_t counter = 0;
    if (LOG.available() > 0)
    {
        m_buffer = new uint8_t[DEFAULT_BUFFER_SIZE];

        LOG.readBytesUntil('\n', m_buffer, DEFAULT_BUFFER_SIZE);

        /**
         * start of transaction
         * query all header information and start to loop all the points of map
         */
        if (strlen((const char *)m_buffer) > 0)
        {
            sscanf((const char *)m_buffer, "%u\n", &m_roadPointsSize);
#if USE_DEBUG
            FLOG_I(F("Total point = %d\n"), m_roadPointsSize);
#endif
            if (m_roadPointsSize > 0)
            {
                if (createRoadPoints(m_roadPointsSize))
                {
                    m_timeout = GET_MAP_TIMEOUT;
                    while (--m_timeout > 0)
                    {
                        memset(m_buffer, 0, DEFAULT_BUFFER_SIZE);
                        _SIZE_ = LOG.readBytesUntil('\n', m_buffer, DEFAULT_BUFFER_SIZE);

                        if (strstr((char *)m_buffer, "quit") != NULL)
                            break;

                        if (_SIZE_ > 0)
                        {
                            char *begin = strchr((const char *)m_buffer, '(');
                            if (begin)
                            {
                                sscanf((const char *)begin, "(%d, %d)\n", &_X_, &_Y_);
                                addPoint(_X_, _Y_);

                                counter++;
                                // get done
                                if (counter == m_roadPointsSize)
                                {
                                    ret = 0;
                                    FLOG_I(F("Get map: success\n"), NULL);
                                    break;
                                }
#if USE_DEBUG
                                FLOG_I(F("Add new point: (%d, %d)\n"), _X_, _Y_);
                            }
#endif
                        }
                        else
                            m_timeout -= 100;
                    }
#if USE_DEBUG
                    if (m_timeout < 0)
                        FLOG_I(F("Get map: timeout\n"), NULL);
#endif
                }
            }
        }
        delete m_buffer;
        m_buffer = NULL;
    }

    if (ret == 0)
        preProcessMap();

    return ret;
}

void CarkitMap::SaveMapToEEPROM()
{
    FLOG_I(F("Save the Map to EEPROM\n"), NULL);
    EEPROM.write(EEPROM_BASE_ADDRESS, m_roadPointsSize);

    for (uint16_t i = 0; i < m_roadPointsSize; i++)
    {
        EEPROM.put<CPoint_t>(EEPROM_BASE_ADDRESS + 1 + i * sizeof(CPoint_t), m_roadPoints[i]);
    }
}

void CarkitMap::LoadMapFromEEPROM()
{
    clear();
    uint8_t size = EEPROM.read(EEPROM_BASE_ADDRESS);
    if (size > MAX_ROAD_POINT || size == 0)
    {
        FLOG_I(F("Load Map from EEPROM failed\n"), NULL);
        return;
    }
    m_roadPointsSize = size;
    FLOG_I(F("Total point has loaded = %u\n"), size);
    if (createRoadPoints(m_roadPointsSize))
    {
        for (uint16_t i = 0; i < m_roadPointsSize; i++)
        {
            EEPROM.get<CPoint_t>(EEPROM_BASE_ADDRESS + 1 + i * sizeof(CPoint_t), m_roadPoints[i]);
        }
    }
}

CPoint_t &CarkitMap::StartP()
{
    return m_startP;
}

CPoint_t &CarkitMap::EndP()
{
    return m_endP;
}

uint8_t &CarkitMap::Size()
{
    return m_roadPointsSize;
}

CPoint_t *CarkitMap::PointList()
{
    return m_roadPoints;
}

CPoint_t *CarkitMap::GetNextPoint()
{
    static int8_t index = 0;
    if (m_roadPoints)
    {
        if (index >= m_roadPointsSize)
            return NULL;

        FLOG_I(F("Get next point: (%d, %d)\n"), m_roadPoints[index].x, m_roadPoints[index].y);
        return &m_roadPoints[index++];
    }
    else
        return NULL;
}