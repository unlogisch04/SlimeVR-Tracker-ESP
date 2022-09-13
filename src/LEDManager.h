/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#ifndef SLIMEVR_LEDMANAGER_H
#define SLIMEVR_LEDMANAGER_H

#include <Arduino.h>
#include "globals.h"
#include "logging/Logger.h"
#include "status/Status.h"

/*
#define DEFAULT_LENGTH 200
#define DEFAULT_GAP 300
#define DEFAULT_INTERVAL 2000

#define STANDBUY_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_LENGTH DEFAULT_LENGTH
#define IMU_ERROR_INTERVAL 2000
#define IMU_ERROR_COUNT 5
#define LOW_BATTERY_LENGTH DEFAULT_LENGTH
#define LOW_BATTERY_INTERVAL 2000
#define LOW_BATTERY_COUNT 1
#define WIFI_CONNECTING_LENGTH DEFAULT_LENGTH
#define WIFI_CONNECTING_INTERVAL 2000
#define WIFI_CONNECTING_COUNT 3
#define SERVER_CONNECTING_LENGTH DEFAULT_LENGTH
#define SERVER_CONNECTING_INTERVAL 2000
#define SERVER_CONNECTING_COUNT 2
*/

namespace SlimeVR
{
    enum LEDStage
    {
        OFF,
        ON,
        GAP,
        INTERVAL
    };

    struct LEDConfig {
        SlimeVR::Status::TrackerStatus Status;
        uint16_t Length;
        uint16_t GAP;
        uint16_t Interval;
        uint8_t Count;
        LEDConfig() {}

        LEDConfig(SlimeVR::Status::TrackerStatus Status_, uint16_t Length_, uint16_t GAP_, uint16_t Interval_, uint8_t Count_)
        {
            Status = Status_;
            Length = Length_;
            GAP = GAP_;
            Interval = Interval_;
            Count = Count_;
        }
    };

    class LEDManager
    {
    public:
        LEDManager(uint8_t pin) : m_Pin(pin) {}

        void setup();

        /*!
         *  @brief Turns the LED on
         */
        void on();

        /*!
         *  @brief Turns the LED off
         */
        void off();

        /*!
         *  @brief Blink the LED for [time]ms. *Can* cause lag
         *  @param time Amount of ms to turn the LED on
         */
        void blink(unsigned long time);

        /*!
         *  @brief Show a pattern on the LED. *Can* cause lag
         *  @param timeon Amount of ms to turn the LED on
         *  @param timeoff Amount of ms to turn the LED off
         *  @param times Amount of times to display the pattern
         */
        void pattern(unsigned long timeon, unsigned long timeoff, int times);

        void update();

    private:
        uint8_t m_LEDConfigsize = 6;
        bool m_active = false; 
        uint8_t m_CurrentCount = 0;
        uint8_t m_CurrentError = 0;
        unsigned int length = 0;
        unsigned long m_Timer = 0;
        LEDStage m_CurrentStage = OFF;
        unsigned long m_LastUpdate = millis();
        LEDConfig *pCurLEDConf;

        uint8_t m_Pin;

        Logging::Logger m_Logger = Logging::Logger("LEDManager");
    };
}

#endif
