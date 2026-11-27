// Minimal Arduino API shim so firmware helpers can be compiled and tested on
// the host (see test/run_host_tests.sh). Only what the tested code uses.
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

typedef uint8_t byte;
typedef bool boolean;

#define PROGMEM
#define IRAM_ATTR
#define F(x) (x)
#define FPSTR(x) (x)
#define HEX 16

// Fake clock. Every millis() call advances it by 1 ms so that busy-wait
// loops in the library terminate; delay() advances it by the given amount.
extern unsigned long g_fake_millis;
inline unsigned long millis() { return g_fake_millis++; }
inline void delay(unsigned long ms) { g_fake_millis += ms; }
inline void yield() {}

class String
{
public:
  String() {}
  String(const char *c) : s_(c ? c : "") {}
  String(const std::string &x) : s_(x) {}
  explicit String(char c) : s_(1, c) {}
  explicit String(int v) : s_(std::to_string(v)) {}
  explicit String(unsigned int v) : s_(std::to_string(v)) {}
  explicit String(long v) : s_(std::to_string(v)) {}
  explicit String(unsigned long v) : s_(std::to_string(v)) {}
  explicit String(float v, unsigned char decimals = 2) { fmt(v, decimals); }
  explicit String(double v, unsigned char decimals = 2) { fmt(v, decimals); }

  const char *c_str() const { return s_.c_str(); }
  size_t length() const { return s_.length(); }
  bool isEmpty() const { return s_.empty(); }
  void reserve(size_t n) { s_.reserve(n); }
  void clear() { s_.clear(); }

  bool concat(const char *c)
  {
    if (c)
      s_ += c;
    return true;
  }
  bool concat(char c)
  {
    s_ += c;
    return true;
  }
  String &operator=(const char *c)
  {
    s_ = c ? c : "";
    return *this;
  }
  String &operator+=(const String &o)
  {
    s_ += o.s_;
    return *this;
  }
  String &operator+=(const char *c)
  {
    if (c)
      s_ += c;
    return *this;
  }
  String &operator+=(char c)
  {
    s_ += c;
    return *this;
  }
  char operator[](size_t i) const { return i < s_.size() ? s_[i] : '\0'; }

  bool operator==(const String &o) const { return s_ == o.s_; }
  bool operator==(const char *c) const { return s_ == (c ? c : ""); }
  bool operator!=(const String &o) const { return !(*this == o); }
  bool operator!=(const char *c) const { return !(*this == c); }

  long toInt() const { return strtol(s_.c_str(), nullptr, 10); }
  float toFloat() const { return strtof(s_.c_str(), nullptr); }
  void toLowerCase()
  {
    for (auto &ch : s_)
      ch = (char)tolower((unsigned char)ch);
  }
  int indexOf(const char *needle) const
  {
    size_t p = s_.find(needle);
    return p == std::string::npos ? -1 : (int)p;
  }

  const std::string &str() const { return s_; }

private:
  void fmt(double v, unsigned char decimals)
  {
    char b[64];
    snprintf(b, sizeof(b), "%.*f", (int)decimals, v);
    s_ = b;
  }
  std::string s_;
};

inline String operator+(const String &a, const String &b)
{
  String r(a);
  r += b;
  return r;
}
inline String operator+(const String &a, const char *b)
{
  String r(a);
  r += b;
  return r;
}
inline String operator+(const char *a, const String &b)
{
  String r(a);
  r += b;
  return r;
}

#include "HardwareSerial.h"
