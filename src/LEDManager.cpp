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

#include "LEDManager.h"
#include "GlobalVars.h"
//#include "status/Status.h"

namespace SlimeVR
{
    LEDConfig LEDBlinks[]{
#if defined(LED_INTERVAL_STANDBY) && LED_INTERVAL_STANDBY > 0
            LEDConfig(SlimeVR::Status::TrackerStatus::NONE,                100, 300, LED_INTERVAL_STANDBY, 1),
#endif
            LEDConfig(SlimeVR::Status::TrackerStatus::LOADING,             50, 50, 1000, 20),
            LEDConfig(SlimeVR::Status::TrackerStatus::LOW_BATTERY,         100, 300, 2000, 1),
            LEDConfig(SlimeVR::Status::TrackerStatus::IMU_ERROR ,          100, 300, 2000, 5),
            LEDConfig(SlimeVR::Status::TrackerStatus::WIFI_CONNECTING ,    100, 300, 2000, 3),
            LEDConfig(SlimeVR::Status::TrackerStatus::SERVER_CONNECTING ,  100, 300, 2000, 2)
    };

    void LEDManager::setup()
    {
#if ENABLE_LEDS
        pinMode(m_Pin, OUTPUT);
#endif
        this->m_LEDConfigsize = sizeof(LEDBlinks)/sizeof(LEDConfig);
        this->m_CurrentError = 0;
        this->pCurLEDConf = &LEDBlinks[m_CurrentError];
        // Do the initial pull of the state
        update();
    }

    void LEDManager::on()
    {
#if ENABLE_LEDS
        digitalWrite(m_Pin, LED__ON);
#endif
    }

    void LEDManager::off()
    {
#if ENABLE_LEDS
        digitalWrite(m_Pin, LED__OFF);
#endif
    }

    void LEDManager::blink(unsigned long time)
    {
        on();
        delay(time);
        off();
    }

    void LEDManager::pattern(unsigned long timeon, unsigned long timeoff, int times)
    {
        for (int i = 0; i < times; i++)
        {
            blink(timeon);
            delay(timeoff);
        }
    }

    void LEDManager::update()
    {
        unsigned long time = millis();
        unsigned long diff = time - m_LastUpdate;

        // Don't tick the LEDManager *too* often
        if (diff < 10)
        {
            return;
        }

        this->m_LastUpdate = time;

        if (!statusManager.hasStatus(SlimeVR::Status::TrackerStatus::NONE))
        {
            // What do when there is a Error
            if (!this->m_active)
            {
                for (uint8_t i = 0; i < this->m_LEDConfigsize; i++) 
                {
                    this->m_CurrentError++;
                    if (this->m_CurrentError >= this->m_LEDConfigsize)
                    {
                        this->m_CurrentError = 0;
                    }
                    if (statusManager.hasStatus(LEDBlinks[m_CurrentError].Status)) 
                    {
                        this->pCurLEDConf = &LEDBlinks[m_CurrentError];
                        this->m_active = true;
                        this->m_CurrentStage = OFF;
                    }
                }
            }    
        }
        else
        {
            // What can we ignore when we have no Error / no Status to show?
#if defined(LED_INTERVAL_STANDBY) && LED_INTERVAL_STANDBY > 0
            this->pCurLEDConf = &LEDBlinks[0];
            if (!this->m_active) 
            {
                this->m_active = true;
                this->m_CurrentStage = OFF;
            }
#endif
        }
        
        if (this->m_active)
        {
            switch (this->m_CurrentStage)
            {
                case ON:
//                  Serial.printf("LEDManager Stage ON\n");
                    break;
                case OFF:
                    length = (pCurLEDConf)->Length;
//                  Serial.printf("LEDManager Stage OFF\n");
                    break;
                case GAP:
                    length = (pCurLEDConf)->GAP;
//                  Serial.printf("LEDManager Stage GAP\n");
                    break;
                case INTERVAL:
                    length = (pCurLEDConf)->Interval;
//                  Serial.printf("LEDManager Stage INTERVAL\n");
                break;
            }
        }       

//        if (m_CurrentStage == OFF || m_Timer + diff >= length)
        if ( m_Timer + diff >= length)
        {
            m_Timer = 0;
            // Advance stage
            switch (m_CurrentStage)
            {
                case OFF:
                    on();
                    m_CurrentStage = ON;
                    m_CurrentCount = 0;
                    break;
                case ON:
                    off();
                    m_CurrentCount++;
                    if (this->m_CurrentCount >= (pCurLEDConf)->Count)
                    {
                        this->m_CurrentCount = 0;
                        this->m_CurrentStage = INTERVAL;
                    }
                    else
                    {
                        this->m_CurrentStage = GAP;
                    }
                    break;
                case GAP:
                    this->m_CurrentStage = OFF;
                    break;
                case INTERVAL:
                    this->m_active = false;
                    break;
            }
        }
        else
        {
            m_Timer += diff;
        }
    }
}
