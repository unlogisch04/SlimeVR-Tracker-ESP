#ifndef STATUS_STATUS_H
#define STATUS_STATUS_H

namespace SlimeVR
{
    namespace Status
    {
        enum TrackerStatus
        {
            NONE = 0,
            LOADING = 1 << 0,
            LOW_BATTERY = 1 << 1,
            IMU_ERROR = 1 << 2,
            WIFI_CONNECTING = 1 << 3,
            SERVER_CONNECTING = 1 << 4
        };

        const char *statusToString(TrackerStatus status);
    }
}

#endif
