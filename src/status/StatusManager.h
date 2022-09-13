#ifndef STATUS_STATUSMANAGER_H
#define STATUS_STATUSMANAGER_H

#include "Status.h"
#include "logging/Logger.h"

namespace SlimeVR
{
    namespace Status
    {
        class StatusManager
        {
        public:
            void setStatus(TrackerStatus status, bool value);
            bool hasStatus(TrackerStatus status);
            bool noStatus();
            uint32_t getStatus();

        private:
            uint32_t m_Status;

            Logging::Logger m_Logger = Logging::Logger("StatusManager");
        };
    }
}

#endif
