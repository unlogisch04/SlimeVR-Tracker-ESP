#ifndef LOGGING_LOGBUFFER_H
#define LOGGING_LOGBUFFER_H

#include <Arduino.h>

namespace SlimeVR::Logging {

class LogBuffer {
public:
	static constexpr size_t MAX_BUFFER_SIZE = 4096;  // Total buffer size
	static constexpr size_t MAX_BYTES_PER_CYCLE
		= 128;  // Max bytes to output per cycle (match with Serial buffer size)

	static LogBuffer& getInstance() {
		static LogBuffer instance;
		return instance;
	}

	// Add a log message to the buffer
	bool addLog(const char* message);

	// Process buffered logs up to MAX_BYTES_PER_CYCLE
	void processCycle();

	// Check if there are pending logs
	bool hasPendingLogs() const { return m_WritePos != m_ReadPos; }

	// Get number of bytes pending
	size_t getPendingBytes() const;

	// Force flush all pending logs (for critical errors or shutdown)
	void flushAll();

private:
	LogBuffer()
		: m_Buffer{0}
		, m_WritePos(0)
		, m_ReadPos(0)
		, m_HasActiveMessage(false)
		, m_RemainingMessageBytes(0) {}

	char m_Buffer[MAX_BUFFER_SIZE];
	volatile size_t m_WritePos;
	volatile size_t m_ReadPos;
	bool m_HasActiveMessage;
	size_t m_RemainingMessageBytes;

	size_t getAvailableSpace() const;
	size_t getNextMessageLength() const;
};

}  // namespace SlimeVR::Logging

#endif
