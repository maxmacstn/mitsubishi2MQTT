// Host shim for HardwareSerial: records written bytes and replays queued
// input bytes so the CN105 protocol code can be exercised without hardware.
#pragma once

#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>

class String;

#define SERIAL_8E1 0x1e

class HardwareSerial
{
public:
  void begin(unsigned long baud, int config = 0, int rx = -1, int tx = -1)
  {
    lastBaud = baud;
    lastRx = rx;
    lastTx = tx;
    beginCalls++;
  }
  void end() {}
  void setTimeout(unsigned long) {}
  int available() { return (int)rxQueue.size(); }
  int read()
  {
    if (rxQueue.empty())
      return -1;
    uint8_t c = rxQueue.front();
    rxQueue.pop_front();
    return c;
  }
  void flush() {}
  size_t write(uint8_t c)
  {
    written.push_back(c);
    return 1;
  }
  size_t write(const uint8_t *data, size_t len)
  {
    written.insert(written.end(), data, data + len);
    return len;
  }

  // Console-style output used by the library's debug prints: discarded.
  template <typename T>
  size_t print(const T &) { return 0; }
  template <typename T>
  size_t println(const T &) { return 0; }
  size_t println() { return 0; }
  int printf(const char *, ...) { return 0; }

  // Test helpers
  void queue(const std::vector<uint8_t> &bytes) { rxQueue.insert(rxQueue.end(), bytes.begin(), bytes.end()); }

  std::deque<uint8_t> rxQueue;
  std::vector<uint8_t> written;
  unsigned long lastBaud = 0;
  int lastRx = -1;
  int lastTx = -1;
  int beginCalls = 0;
};

extern HardwareSerial Serial;
