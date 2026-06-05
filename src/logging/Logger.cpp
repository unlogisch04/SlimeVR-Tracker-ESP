#include "Logger.h"

namespace SlimeVR::Logging {
void Logger::setTag(const char* tag) {
	m_Tag = (char*)malloc(strlen(tag) + 1);
	strcpy(m_Tag, tag);
}

void Logger::trace(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(TRACE, format, args);
	va_end(args);
}

void Logger::debug(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(DEBUG, format, args);
	va_end(args);
}

void Logger::info(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(INFO, format, args);
	va_end(args);
}

void Logger::warn(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(WARN, format, args);
	va_end(args);
}

void Logger::error(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(ERROR, format, args);
	va_end(args);
}

void Logger::fatal(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(FATAL, format, args);
	va_end(args);
}

void Logger::log(Level level, const char* format, va_list args) const {
	if (level < LOG_LEVEL) {
		return;
	}

	char buffer[256];
	vsnprintf(buffer, 256, format, args);

	char buf[strlen(m_Prefix) + (m_Tag == nullptr ? 0 : strlen(m_Tag)) + 2];
	strcpy(buf, m_Prefix);
	if (m_Tag != nullptr) {
		strcat(buf, ":");
		strcat(buf, m_Tag);
	}
	char fullMessage[strlen(levelToString(level)) +strlen(buf) + strlen(buffer) + 10];
	snprintf(fullMessage, sizeof(fullMessage), "[%-5s] [%s] %s\n", levelToString(level), buf, buffer);

	if (!LogBuffer::getInstance().addLog(fullMessage)) {
		// Buffer full, try to make space
		LogBuffer::getInstance().processCycle();
		// Try again (most messages are less than 127 bytes)
		if (!LogBuffer::getInstance().addLog(fullMessage)) {
			// Still cant add force flush and add
			LogBuffer::getInstance().flushAll();
			Serial.println("Buffer Flushed Due to Log Overflow");
			LogBuffer::getInstance().addLog(fullMessage);
		}
	}
}
}  // namespace SlimeVR::Logging
