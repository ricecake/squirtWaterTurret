#include <stdint.h>
/*
 *	An Arduino library for the Hi-Link LD2450 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based off the https://github.com/0ingchun/arduino-lib_HLK-LD2450_Radar.
 *
 *	https://github.com/ncmreynolds/ld2410
 *
 *
 */
#ifndef LD2450_cpp
#define LD2450_cpp

#include "LD2450.h"

LD2450::LD2450() // Constructor function
{
}

LD2450::~LD2450() // Destructor function
{
}

void LD2450::begin(Stream &radarStream)
{
    LD2450::radar_uart = &radarStream;
}

void LD2450::begin(HardwareSerial &radarStream, bool already_initialized)
{
    if (!already_initialized)
    {
        radarStream.begin(LD2450_SERIAL_SPEED);
    }

    LD2450::radar_uart = &radarStream;
}

#ifdef ENABLE_SOFTWARESERIAL_SUPPORT
void LD2450::begin(SoftwareSerial &radarStream, bool already_initialized)
{
    if (!already_initialized)
    {
        radarStream.begin(LD2450_SERIAL_SPEED);
    }

    LD2450::radar_uart = &radarStream;
}
#endif

void LD2450::setNumberOfTargets(uint16_t _numTargets)
{
    if (_numTargets > LD2450_MAX_SENSOR_TARGETS)
    {
        _numTargets = LD2450_MAX_SENSOR_TARGETS;
    }

    LD2450::numTargets = _numTargets;
}

bool LD2450::waitForSensorMessage(bool wait_forever)
{

    uint8_t read_result = 0;
    for (long i = 0; i < LD2450_DEFAULT_RETRY_COUNT_FOR_WAIT_FOR_MSG; i++)
    {
        read_result = LD2450::read();
        if (read_result >= 0)
        {
            return true;
        }
        delay(1);

        //.... :)
        if (wait_forever)
        {
            i = 0;
        }
    }
    return false;
}

int LD2450::read()
{
    if (LD2450::radar_uart == nullptr)
    {
        return -2;
    }

    unsigned int available = LD2450::radar_uart->available();
    if (available >= 30)
    {

        byte rec_buf[LD2450_SERIAL_BUFFER] = "";
        const int len = LD2450::radar_uart->readBytes(rec_buf, min(available, sizeof(rec_buf)));
        // IF WE GOT DATA PARSE THEM
        if (len > 0)
        {
            return LD2450::ProcessSerialDataIntoRadarData(rec_buf, len);
        }
    }
    return -1;
}

uint16_t LD2450::getSensorSupportedTargetCount()
{
    if (LD2450::numTargets < LD2450_MAX_SENSOR_TARGETS)
    {
        return LD2450::numTargets;
    }

    return LD2450_MAX_SENSOR_TARGETS;
}

LD2450::RadarTarget LD2450::getTarget(uint16_t _target_id)
{
    if (_target_id >= LD2450_MAX_SENSOR_TARGETS)
    {
        LD2450::RadarTarget tmp;
        tmp.valid = false;
        return tmp;
    }
    return LD2450::radarTargets[_target_id];
}
int LD2450::ProcessSerialDataIntoRadarData(byte rec_buf[], int len)
{
    int redreshed_targets = 0;
    for (int i = 0; i < len; i++)
    {
        // Checking the header and footer
        if (rec_buf[i] == 0xAA && rec_buf[i + 1] == 0xFF && rec_buf[i + 2] == 0x03 && rec_buf[i + 3] == 0x00 && rec_buf[i + 28] == 0x55 && rec_buf[i + 29] == 0xCC)
        {
            int index = i + 4; // Skip header and in-frame data length fields

            for (uint16_t targetCounter = 0; targetCounter < LD2450_MAX_SENSOR_TARGETS; targetCounter++)
            {
                if (index + 7 < len)
                {
                    LD2450::radarTargets[targetCounter].id = targetCounter + 1;
                    LD2450::radarTargets[targetCounter].x = (int16_t)(rec_buf[index] | (rec_buf[index + 1] << 8));
                    LD2450::radarTargets[targetCounter].y = (int16_t)(rec_buf[index + 2] | (rec_buf[index + 3] << 8));
                    LD2450::radarTargets[targetCounter].speed = (int16_t)(rec_buf[index + 4] | (rec_buf[index + 5] << 8));
                    LD2450::radarTargets[targetCounter].resolution = (uint16_t)(rec_buf[index + 6] | (rec_buf[index + 7] << 8));

                    // Check the highest bit of x and y. Adjust the sign
                    if (rec_buf[index + 1] & 0x80)
                        LD2450::radarTargets[targetCounter].x -= 0x8000;
                    else
                        LD2450::radarTargets[targetCounter].x = -LD2450::radarTargets[targetCounter].x;
                    if (rec_buf[index + 3] & 0x80)
                        LD2450::radarTargets[targetCounter].y -= 0x8000;
                    else
                        LD2450::radarTargets[targetCounter].y = -LD2450::radarTargets[targetCounter].y;
                    if (rec_buf[index + 5] & 0x80)
                        LD2450::radarTargets[targetCounter].speed -= 0x8000;
                    else
                        LD2450::radarTargets[targetCounter].speed = -LD2450::radarTargets[targetCounter].speed;


                    // IF A RESOLUTION IS PRESENT THEN WE CAN ASSUME THAT A TARGET WAS FOUND
                    LD2450::radarTargets[targetCounter].valid = LD2450::radarTargets[targetCounter].resolution != 0;

                    index += 8; // Move to the start of the next target data

                    redreshed_targets++;

                    // SKIP IF USER ONLY REQUESTED X VALID TARGETS
                    if (redreshed_targets >= LD2450::numTargets)
                    {
                        break;
                    }
                }
                else
                {
                    LD2450::radarTargets[targetCounter].valid = false;
                }
            }
            i = index; // Updating the index of an external loop
        }
    }
    return redreshed_targets;
}

#endif
