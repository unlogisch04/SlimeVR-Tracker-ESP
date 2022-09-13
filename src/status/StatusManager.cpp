#include "StatusManager.h"

namespace SlimeVR
{
    namespace Status
    {
        void StatusManager::setStatus(TrackerStatus status, bool value)
        {
            if (value)
            {
                if (m_Status & status)
                {
                    return;
                }

                m_Logger.trace("Added status %s", statusToString(status));

                m_Status |= status;
            }
            else
            {
                if (!(m_Status & status))
                {
                    return;
                }

                m_Logger.trace("Removed status %s", statusToString(status));

                m_Status &= ~status;
            }
        }

        bool StatusManager::hasStatus(TrackerStatus status)
        {
            if (status == Status::NONE)
            {
                return m_Status == 0;
            }
            return (m_Status & status) == status;
        }

        uint32_t StatusManager::getStatus()
        {
            return m_Status;
        }
    }
}
