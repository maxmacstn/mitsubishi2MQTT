#pragma once

#include <Arduino.h>
#include <stdarg.h>

#ifndef LOG_SIZE
#ifdef ESP32
    #define LOG_SIZE 50000
#else
    #define LOG_SIZE 10000
#endif
#endif

// Maximum length of one formatted log line (including the "[tag:time]\t" prefix).
#define LOG_LINE_MAX 512

class Logging{

    private:
        Logging() = default;
        char logBuffPTR [LOG_SIZE];
        size_t logBuffSize = 0;

        // Formats "[tag:seconds]\t" + message (+ optional '\n') into a bounded line buffer.
        void writeLine(const char *tag, const char *message, bool newline);
        void vwrite(const char *tag, bool newline, const char *format, va_list args);

    public:
        static Logging &getInstance();
        Logging(const Logging &) = delete; // no copying
        Logging &operator=(const Logging &) = delete;

        // Appends raw bytes to the ring buffer, dropping the oldest whole lines when full.
        void storeLog(const char *log, size_t size);

        // printf-style. Only pass trusted, literal format strings here.
        void f(const char* tag, const char *format, ...);
        // Logs the string verbatim; '%' in the text is never interpreted.
        void f(const char* tag, const String &string);
        void ln(const char* tag, const String &string);
        void ln(const char* tag, const char *format, ...);

        String getLogs();
        size_t size() const { return logBuffSize; }
        void clear() { logBuffSize = 0; }
};

extern Logging &Log;
