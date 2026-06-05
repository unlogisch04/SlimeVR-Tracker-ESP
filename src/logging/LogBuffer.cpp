#include "LogBuffer.h"

namespace SlimeVR::Logging {

bool LogBuffer::addLog(const char* message) {
	size_t len = strlen(message);

	// Need space for message + null terminator + length prefix (2 bytes)
	size_t required = len + 3;

	if (required > getAvailableSpace()) {
		// Buffer full, drop the message or overwrite oldest
		return false;
	}

	// Write length as 2-byte prefix (little endian)
	m_Buffer[m_WritePos] = (len >> 8) & 0xFF;
	m_WritePos = (m_WritePos + 1) % MAX_BUFFER_SIZE;
	m_Buffer[m_WritePos] = len & 0xFF;
	m_WritePos = (m_WritePos + 1) % MAX_BUFFER_SIZE;

	// Write message
	for (size_t i = 0; i < len; i++) {
		m_Buffer[m_WritePos] = message[i];
		m_WritePos = (m_WritePos + 1) % MAX_BUFFER_SIZE;
	}

	// Write null terminator
	m_Buffer[m_WritePos] = '\0';
	m_WritePos = (m_WritePos + 1) % MAX_BUFFER_SIZE;
	return true;
}

void LogBuffer::processCycle() {
	size_t bytesWritten = 0;

	size_t freefifo = Serial.availableForWrite();

	if (freefifo > MAX_BYTES_PER_CYCLE) {
		freefifo = MAX_BYTES_PER_CYCLE;
	}
	if (freefifo == 0) {
		return;  // Can't write anything right now
	}

	while (bytesWritten < freefifo) {
		if (!m_HasActiveMessage) {
			if (m_WritePos == m_ReadPos) {
				break;
			}

			size_t msgLen = getNextMessageLength();
			if (msgLen == 0) {
				// Corrupted length prefix, stop processing and try again next cycle
				break;
			}

			// Skip length prefix and start sending this message
			m_ReadPos = (m_ReadPos + 2) % MAX_BUFFER_SIZE;
			m_RemainingMessageBytes = msgLen;
			m_HasActiveMessage = true;
		}

		size_t remainingBudget = freefifo - bytesWritten;
		size_t toWrite = m_RemainingMessageBytes < remainingBudget
						   ? m_RemainingMessageBytes
						   : remainingBudget;

		for (size_t i = 0; i < toWrite; i++) {
			Serial.write(m_Buffer[m_ReadPos]);
			m_ReadPos = (m_ReadPos + 1) % MAX_BUFFER_SIZE;
		}

		bytesWritten += toWrite;
		m_RemainingMessageBytes -= toWrite;

		if (m_RemainingMessageBytes == 0) {
			// Message complete, skip null terminator and continue with next message
			m_ReadPos = (m_ReadPos + 1) % MAX_BUFFER_SIZE;
			m_HasActiveMessage = false;
		}
	}
}

size_t LogBuffer::getPendingBytes() const {
	if (m_WritePos >= m_ReadPos) {
		return m_WritePos - m_ReadPos;
	} else {
		return MAX_BUFFER_SIZE - m_ReadPos + m_WritePos;
	}
}

void LogBuffer::flushAll() {
	while (true) {
		if (!m_HasActiveMessage) {
			if (m_WritePos == m_ReadPos) {
				break;
			}

			size_t msgLen = getNextMessageLength();
			if (msgLen == 0) {
				break;  // Corrupted buffer
			}

			// Skip length prefix and flush this message fully
			m_ReadPos = (m_ReadPos + 2) % MAX_BUFFER_SIZE;
			m_RemainingMessageBytes = msgLen;
			m_HasActiveMessage = true;
		}

		while (m_RemainingMessageBytes > 0) {
			Serial.write(m_Buffer[m_ReadPos]);
			m_ReadPos = (m_ReadPos + 1) % MAX_BUFFER_SIZE;
			m_RemainingMessageBytes--;
		}

		// Message complete, skip null terminator and continue
		m_ReadPos = (m_ReadPos + 1) % MAX_BUFFER_SIZE;
		m_HasActiveMessage = false;
	}
}

size_t LogBuffer::getAvailableSpace() const {
	size_t used = getPendingBytes();
	return MAX_BUFFER_SIZE - used - 1;  // -1 to distinguish full from empty
}

size_t LogBuffer::getNextMessageLength() const {
	if (m_WritePos == m_ReadPos) {
		return 0;
	}

	size_t pos = m_ReadPos;
	uint8_t highByte = m_Buffer[pos];
	pos = (pos + 1) % MAX_BUFFER_SIZE;
	uint8_t lowByte = m_Buffer[pos];

	return (highByte << 8) | lowByte;
}

}  // namespace SlimeVR::Logging
