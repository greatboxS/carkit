#include "carkit_map.h"

#define LOG Serial
#define DEFAULT_BUFFER_SIZE (256)
#define GET_MAP_TIMEOUT (10000)
#ifndef USE_DEBUG
#define USE_DEBUG 1
#endif

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

/**
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
        LOG.println("Card failed, or not present");
        return -1;
    }

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    m_mapFile = new File;
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
                    sscanf((const char *)m_buffer, "%d\n", &m_roadPointsSize);
#if USE_DEBUG
                    LOG.print("Total point = ");
                    LOG.println(m_roadPointsSize);
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
                                LOG.print(F("Add new point: ("));
                                LOG.print(_X_);
                                LOG.print(", ");
                                LOG.print(_Y_);
                                LOG.println(")");
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

    delete m_buffer;
    m_buffer = NULL;
    delete m_mapFile;
    m_mapFile = NULL;

    return m_roadPointsSize;
}

void CarkitMap::GetMapFromSerialPort()
{
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
            sscanf((const char *)m_buffer, "%d\n", &m_roadPointsSize);
#if USE_DEBUG
            LOG.print("Total point = ");
            LOG.println(m_roadPointsSize);
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
#if USE_DEBUG
                                LOG.print(F("Add new point: ("));
                                LOG.print(_X_);
                                LOG.print(", ");
                                LOG.print(_Y_);
                                LOG.println(")");
                            }
#endif
                        }
                        else
                            m_timeout -= 100;
                    }

#if USE_DEBUG
                    LOG.println(F("Get map: quit"));
#endif
                }
            }
        }
        delete m_buffer;
        m_buffer = NULL;
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
