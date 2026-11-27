/*
  stability.h - small, host-testable helpers used by mitsubishi2mqtt for
  rollover-safe timing, safe JSON parsing, atomic SPIFFS config storage,
  HTML escaping and session handling.

  Everything here is header-only and depends only on ArduinoJson 6,
  Arduino's String and a filesystem object with the fs::FS interface
  (open/exists/remove/rename), so it can be compiled on the host for tests.
*/
#pragma once

#include <ArduinoJson.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>
#include <new>

namespace m2m
{

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

// True once `interval` ms have passed since `start`. Correct across the
// 49.7-day millis() rollover because it uses unsigned subtraction.
inline bool intervalElapsed(unsigned long now, unsigned long start, unsigned long interval)
{
  return (unsigned long)(now - start) >= interval;
}

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------

// Reads an integer stored either as a JSON number or as a numeric string
// ("15", "16.00"). Returns `def` when the value is missing, not numeric or
// outside [minValue, maxValue].
inline long readJsonIntOrDefault(JsonVariantConst v, long def, long minValue, long maxValue)
{
  long value = def;
  bool ok = false;
  if (v.isNull())
  {
    return def;
  }
  if (v.is<long>())
  {
    value = v.as<long>();
    ok = true;
  }
  else if (v.is<double>())
  {
    double d = v.as<double>();
    if (isfinite(d) && d > -2147483648.0 && d < 2147483647.0)
    {
      value = lround(d);
      ok = true;
    }
  }
  else if (v.is<const char *>())
  {
    const char *s = v.as<const char *>();
    if (s != NULL)
    {
      char *end = NULL;
      double d = strtod(s, &end);
      if (end != s)
      {
        while (*end == ' ')
          end++;
        if (*end == '\0' && isfinite(d) && d > -2147483648.0 && d < 2147483647.0)
        {
          value = lround(d);
          ok = true;
        }
      }
    }
  }
  if (!ok || value < minValue || value > maxValue)
  {
    return def;
  }
  return value;
}

// Reads a string value, returning `def` when missing or not a string.
inline String readJsonStringOrDefault(JsonVariantConst v, const char *def)
{
  if (v.is<const char *>())
  {
    const char *s = v.as<const char *>();
    if (s != NULL)
    {
      return String(s);
    }
  }
  return String(def);
}

// ---------------------------------------------------------------------------
// Atomic config storage
// ---------------------------------------------------------------------------

static const size_t CONFIG_FILE_MAX_SIZE = 2048;

enum LoadSource
{
  LOAD_NONE = 0,
  LOAD_PRIMARY,
  LOAD_TMP,
  LOAD_BACKUP
};

inline bool makeSiblingPath(char *out, size_t outSize, const char *path, const char *suffix)
{
  int n = snprintf(out, outSize, "%s%s", path, suffix);
  return n > 0 && (size_t)n < outSize;
}

// Reads exactly one file into `doc`. The buffer is NUL-terminated and its
// length is passed to deserializeJson, so a missing terminator can never
// cause an out-of-bounds read.
template <typename FSType>
bool readJsonFileSingle(FSType &fs, const char *path, JsonDocument &doc, size_t maxSize = CONFIG_FILE_MAX_SIZE)
{
  doc.clear();
  if (!fs.exists(path))
  {
    return false;
  }
  auto file = fs.open(path, "r");
  if (!file)
  {
    return false;
  }
  size_t size = file.size();
  if (size == 0 || size > maxSize)
  {
    file.close();
    return false;
  }
  std::unique_ptr<char[]> buf(new (std::nothrow) char[size + 1]);
  if (!buf)
  {
    file.close();
    return false;
  }
  size_t got = file.readBytes(buf.get(), size);
  file.close();
  if (got != size)
  {
    return false;
  }
  buf[size] = '\0';
  // Pass a const pointer: with a mutable char* ArduinoJson 6 runs in
  // zero-copy mode and keeps pointers into `buf`, which is freed on return.
  const char *input = buf.get();
  DeserializationError err = deserializeJson(doc, input, size);
  if (err || doc.overflowed() || !doc.is<JsonObject>())
  {
    doc.clear();
    return false;
  }
  return true;
}

// Writes `doc` to `path` without ever leaving `path` half-written:
//   1. serialize to <path>.tmp and check the byte count,
//   2. rotate the current <path> to <path>.bak,
//   3. rename <path>.tmp to <path>.
// Every rename target is removed first because ESP32 SPIFFS.rename() fails
// when the destination exists.
template <typename FSType>
bool saveJsonAtomic(FSType &fs, const char *path, const JsonDocument &doc)
{
  if (doc.overflowed())
  {
    return false;
  }
  char tmpPath[48];
  char bakPath[48];
  if (!makeSiblingPath(tmpPath, sizeof(tmpPath), path, ".tmp") ||
      !makeSiblingPath(bakPath, sizeof(bakPath), path, ".bak"))
  {
    return false;
  }

  size_t len = measureJson(doc);
  if (len == 0 || len > CONFIG_FILE_MAX_SIZE)
  {
    return false;
  }
  std::unique_ptr<char[]> buf(new (std::nothrow) char[len + 1]);
  if (!buf)
  {
    return false;
  }
  if (serializeJson(doc, buf.get(), len + 1) != len)
  {
    return false;
  }

  if (fs.exists(tmpPath))
  {
    fs.remove(tmpPath);
  }
  auto file = fs.open(tmpPath, "w");
  if (!file)
  {
    return false;
  }
  size_t written = file.write((const uint8_t *)buf.get(), len);
  file.close();
  if (written != len)
  {
    fs.remove(tmpPath);
    return false;
  }
  // Verify what actually reached flash.
  {
    auto check = fs.open(tmpPath, "r");
    if (!check)
    {
      return false;
    }
    size_t size = check.size();
    check.close();
    if (size != len)
    {
      fs.remove(tmpPath);
      return false;
    }
  }

  if (fs.exists(bakPath))
  {
    fs.remove(bakPath);
  }
  bool hadPrimary = fs.exists(path);
  if (hadPrimary && !fs.rename(path, bakPath))
  {
    // Could not rotate: fall back to replacing in place.
    fs.remove(path);
  }
  if (!fs.rename(tmpPath, path))
  {
    // Put the previous version back so the device still boots with it.
    if (hadPrimary && fs.exists(bakPath) && !fs.exists(path))
    {
      fs.rename(bakPath, path);
    }
    return false;
  }
  return true;
}

// Loads `path`, falling back to <path>.tmp (a complete write interrupted
// before the final rename) and then <path>.bak (the previous version).
template <typename FSType>
LoadSource loadJsonWithBackup(FSType &fs, const char *path, JsonDocument &doc)
{
  if (readJsonFileSingle(fs, path, doc))
  {
    return LOAD_PRIMARY;
  }
  char sibling[48];
  if (makeSiblingPath(sibling, sizeof(sibling), path, ".tmp") && readJsonFileSingle(fs, sibling, doc))
  {
    return LOAD_TMP;
  }
  if (makeSiblingPath(sibling, sizeof(sibling), path, ".bak") && readJsonFileSingle(fs, sibling, doc))
  {
    return LOAD_BACKUP;
  }
  doc.clear();
  return LOAD_NONE;
}

// ---------------------------------------------------------------------------
// Energy
// ---------------------------------------------------------------------------

// kWh consumed at `powerW` watts over `elapsedMs` milliseconds.
inline float energyIncrementKWh(int powerW, unsigned long elapsedMs)
{
  if (powerW <= 0)
  {
    return 0.0f;
  }
  return (float)((double)powerW * (double)elapsedMs / 3600000000.0);
}

// Whether the energy counter should be persisted. Uses the absolute change
// so that a reset to a lower value (Energy Reset) is saved as well.
inline bool shouldSaveEnergy(float energy, float lastSavedValue, unsigned long now,
                             unsigned long lastSavedTime, float threshold, unsigned long intervalMs)
{
  return fabsf(energy - lastSavedValue) > threshold && intervalElapsed(now, lastSavedTime, intervalMs);
}

// ---------------------------------------------------------------------------
// HTML / auth helpers
// ---------------------------------------------------------------------------

inline String htmlEscape(const String &in)
{
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++)
  {
    char c = in[i];
    switch (c)
    {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    case '"':
      out += "&quot;";
      break;
    case '\'':
      out += "&#39;";
      break;
    default:
      out += c;
      break;
    }
  }
  return out;
}

// Compares two byte strings in time that depends only on their lengths.
inline bool constantTimeEquals(const char *a, size_t aLen, const char *b, size_t bLen)
{
  size_t n = aLen > bLen ? aLen : bLen;
  unsigned char diff = (aLen != bLen) ? 1 : 0;
  for (size_t i = 0; i < n; i++)
  {
    unsigned char ca = i < aLen ? (unsigned char)a[i] : 0;
    unsigned char cb = i < bLen ? (unsigned char)b[i] : 0;
    diff |= (unsigned char)(ca ^ cb);
  }
  return diff == 0;
}

// Extracts the value of cookie `name` from a Cookie header. Returns an empty
// string when the cookie is absent. Matches whole cookie names only.
inline String extractCookie(const String &header, const char *name)
{
  const char *h = header.c_str();
  size_t nameLen = strlen(name);
  const char *p = h;
  while (*p)
  {
    while (*p == ' ' || *p == ';')
      p++;
    if (strncmp(p, name, nameLen) == 0 && p[nameLen] == '=')
    {
      const char *v = p + nameLen + 1;
      const char *e = v;
      while (*e && *e != ';')
        e++;
      String out;
      out.reserve((size_t)(e - v));
      for (const char *q = v; q < e; q++)
        out += *q;
      return out;
    }
    while (*p && *p != ';')
      p++;
  }
  return String();
}

// Formats `len` random bytes as lowercase hex.
inline String bytesToHex(const uint8_t *bytes, size_t len)
{
  static const char hex[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++)
  {
    out += hex[bytes[i] >> 4];
    out += hex[bytes[i] & 0x0f];
  }
  return out;
}

// Returns the host[:port] part of an Origin/Referer URL ("http://a.b:80/x" -> "a.b:80").
inline String urlHost(const String &url)
{
  const char *s = url.c_str();
  const char *p = strstr(s, "://");
  if (p == NULL)
  {
    return String();
  }
  p += 3;
  const char *e = p;
  while (*e && *e != '/' && *e != '?' && *e != '#')
    e++;
  String out;
  for (const char *q = p; q < e; q++)
    out += (char)tolower((unsigned char)*q);
  return out;
}

// SoftAP passwords must be 8-63 characters or the AP refuses to start.
inline bool isValidApPassword(const String &pwd)
{
  return pwd.length() >= 8 && pwd.length() <= 63;
}

} // namespace m2m
