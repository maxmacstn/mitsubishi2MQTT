#include "logger.h"

#include <stdio.h>
#include <string.h>

Logging &Logging::getInstance() {
  static Logging instance;
  return instance;
}

void Logging::storeLog(const char *log, size_t size) {
  if (log == NULL || size == 0)
    return;

  if (size > LOG_LINE_MAX)
    size = LOG_LINE_MAX;

  // Make room: drop whole lines from the front until the new data fits.
  if (logBuffSize + size > LOG_SIZE) {
    size_t needed = logBuffSize + size - LOG_SIZE;
    // Drop at least LOG_LINE_MAX bytes at a time so we do not memmove on every line.
    size_t drop = needed < LOG_LINE_MAX ? LOG_LINE_MAX : needed;
    if (drop > logBuffSize)
      drop = logBuffSize;
    // Extend to the end of the line so the buffer always starts at a line boundary.
    while (drop > 0 && drop < logBuffSize && logBuffPTR[drop - 1] != '\n')
      drop++;
    memmove(logBuffPTR, logBuffPTR + drop, logBuffSize - drop);
    logBuffSize -= drop;
  }

  memcpy(logBuffPTR + logBuffSize, log, size);
  logBuffSize += size;
}

void Logging::writeLine(const char *tag, const char *message, bool newline) {
  char buff[LOG_LINE_MAX];
  int prefix = snprintf(buff, sizeof(buff), "[%s:%08lu]\t", tag ? tag : "", (unsigned long)(millis() / 1000));
  if (prefix < 0)
    return;
  size_t size = (size_t)prefix < sizeof(buff) ? (size_t)prefix : sizeof(buff) - 1;

  // Reserve one byte for the optional newline.
  size_t room = sizeof(buff) - 1 - size;
  size_t msgLen = message ? strlen(message) : 0;
  if (msgLen > room)
    msgLen = room;
  if (msgLen)
    memcpy(buff + size, message, msgLen);
  size += msgLen;

  if (newline && (size == 0 || buff[size - 1] != '\n'))
    buff[size++] = '\n';

#ifdef ESP32
  Serial.write((const uint8_t *)buff, size);
#endif

  storeLog(buff, size);
}

void Logging::vwrite(const char *tag, bool newline, const char *format, va_list args) {
  char msg[LOG_LINE_MAX];
  int n = vsnprintf(msg, sizeof(msg), format ? format : "", args);
  if (n < 0)
    return;
  writeLine(tag, msg, newline); // vsnprintf always NUL-terminates within sizeof(msg)
}

void Logging::f(const char *tag, const char *format, ...) {
  va_list args;
  va_start(args, format);
  vwrite(tag, false, format, args);
  va_end(args);
}

void Logging::f(const char *tag, const String &string) {
  writeLine(tag, string.c_str(), false);
}

void Logging::ln(const char *tag, const String &string) {
  writeLine(tag, string.c_str(), true);
}

void Logging::ln(const char *tag, const char *format, ...) {
  va_list args;
  va_start(args, format);
  vwrite(tag, true, format, args);
  va_end(args);
}

String Logging::getLogs() {
  String logs;
  logs.reserve(logBuffSize + 1);
  // Copy in chunks through a NUL-terminated scratch buffer.
  char chunk[129];
  size_t pos = 0;
  while (pos < logBuffSize) {
    size_t n = logBuffSize - pos;
    if (n > sizeof(chunk) - 1)
      n = sizeof(chunk) - 1;
    memcpy(chunk, logBuffPTR + pos, n);
    chunk[n] = '\0';
    logs += chunk;
    pos += n;
  }
  return logs;
}

Logging &Log = Log.getInstance();
