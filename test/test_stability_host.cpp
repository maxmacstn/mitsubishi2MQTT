// Host-side stability tests for mitsubishi2mqtt.
//
// Build and run with test/run_host_tests.sh (C++17, -fsanitize=address,undefined).
// Covers: logger stress/rotation and format-string safety, sendCustomPacket
// bounds, CN105 status power sync, UART pin retention, atomic config storage
// with power-loss recovery, HA discovery payload sizes, millis() rollover,
// energy accumulation / reset, and the session / HTML helpers.

#include <climits>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "Arduino.h"
#include <ArduinoJson.h>

#include "logger.h"
#include "stability.h"
#include "ha_discovery.h"

// Expose HeatPump internals (readPacket, currentStatus) to the tests only.
#define private public
#define protected public
#include "HeatPump.h"
#undef private
#undef protected

unsigned long g_fake_millis = 1000000;
HardwareSerial Serial;

// ---------------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------------

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                              \
  do                                                                             \
  {                                                                              \
    g_checks++;                                                                  \
    if (!(cond))                                                                 \
    {                                                                            \
      g_failures++;                                                              \
      fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
    }                                                                            \
  } while (0)

#define RUN(test)                    \
  do                                 \
  {                                  \
    int before = g_failures;         \
    test();                          \
    printf("%s %s\n", g_failures == before ? "PASS" : "FAIL", #test); \
  } while (0)

// ---------------------------------------------------------------------------
// In-memory filesystem with power-loss injection
// ---------------------------------------------------------------------------

struct PowerLoss
{
};

struct MockFS;

struct MockFile
{
  MockFS *fs = nullptr;
  std::string path;
  bool writing = false;
  size_t pos = 0;
  bool valid = false;

  explicit operator bool() const { return valid; }
  size_t size() const;
  size_t readBytes(char *out, size_t n);
  size_t write(const uint8_t *data, size_t n);
  void close() { valid = false; }
};

struct MockFS
{
  std::map<std::string, std::string> files;
  // Number of mutating operations allowed before simulated power loss
  // (-1 = never). A write that hits the limit persists half its bytes.
  long opsLeft = -1;

  void tick()
  {
    if (opsLeft == 0)
      throw PowerLoss();
    if (opsLeft > 0)
      opsLeft--;
  }

  bool exists(const char *p) const { return files.count(p) != 0; }

  MockFile open(const char *p, const char *mode)
  {
    MockFile f;
    f.fs = this;
    f.path = p;
    if (strcmp(mode, "w") == 0)
    {
      tick();
      files[p] = ""; // truncate on open, like SPIFFS
      f.writing = true;
      f.valid = true;
    }
    else
    {
      f.valid = exists(p);
    }
    return f;
  }

  bool remove(const char *p)
  {
    tick();
    return files.erase(p) != 0;
  }

  // ESP32 SPIFFS semantics: fails when the destination already exists.
  bool rename(const char *from, const char *to)
  {
    tick();
    if (!exists(from) || exists(to))
      return false;
    files[to] = files[from];
    files.erase(from);
    return true;
  }
};

size_t MockFile::size() const { return fs->files[path].size(); }

size_t MockFile::readBytes(char *out, size_t n)
{
  const std::string &d = fs->files[path];
  size_t avail = pos < d.size() ? d.size() - pos : 0;
  size_t k = std::min(n, avail);
  memcpy(out, d.data() + pos, k);
  pos += k;
  return k;
}

size_t MockFile::write(const uint8_t *data, size_t n)
{
  if (fs->opsLeft == 0)
  {
    fs->files[path].append((const char *)data, n / 2); // torn write
    throw PowerLoss();
  }
  fs->tick();
  fs->files[path].append((const char *)data, n);
  return n;
}

// ---------------------------------------------------------------------------
// Logger
// ---------------------------------------------------------------------------

static void test_logger_format_string_safety()
{
  Log.clear();
  String evil = "ssid %s %n %x %%";
  Log.ln("T", "Connected to " + evil);
  String logs = Log.getLogs();
  CHECK(logs.indexOf("Connected to ssid %s %n %x %%") >= 0);

  Log.clear();
  Log.ln("T", "value=%d name=%s", 42, "abc");
  CHECK(Log.getLogs().indexOf("value=42 name=abc") >= 0);
}

static void test_logger_long_lines_are_bounded()
{
  Log.clear();
  std::string big(5000, 'A');
  Log.ln("T", String(big.c_str()));
  CHECK(Log.size() <= LOG_LINE_MAX);
  String logs = Log.getLogs();
  CHECK(logs.length() == Log.size());
  CHECK(logs.str().back() == '\n');

  Log.clear();
  Log.ln("T", "%s", big.c_str()); // printf path, same bound
  CHECK(Log.size() <= LOG_LINE_MAX);
}

static void test_logger_rotation_stress()
{
  Log.clear();
  uint32_t seed = 12345;
  std::string last;
  for (int i = 0; i < 20000; i++)
  {
    seed = seed * 1103515245u + 12345u;
    size_t len = 1 + (seed >> 16) % 700; // includes lines longer than LOG_LINE_MAX
    std::string msg = "#" + std::to_string(i) + " " + std::string(len, (char)('a' + i % 26));
    Log.ln("STRESS", String(msg.c_str()));
    CHECK(Log.size() <= LOG_SIZE);
    last = "#" + std::to_string(i) + " ";
  }
  String logs = Log.getLogs();
  // Starts at a line boundary and keeps the newest line.
  CHECK(logs.str().rfind("[STRESS:", 0) == 0);
  CHECK(logs.indexOf(last.c_str()) >= 0);
  CHECK(logs.str().back() == '\n');
}

// ---------------------------------------------------------------------------
// HeatPump
// ---------------------------------------------------------------------------

static void test_send_custom_packet_bounds()
{
  HeatPump hp;
  HardwareSerial port;
  hp._HardSerial = &port;

  byte data[40];
  for (int i = 0; i < 40; i++)
    data[i] = (byte)(0x10 + i);

  // Normal 20-byte payload -> 22-byte packet: header, payload, checksum.
  port.written.clear();
  hp.sendCustomPacket(data, 20);
  CHECK(port.written.size() == 22);
  CHECK(port.written[0] == 0xfc);
  CHECK(port.written[1] == data[0] && port.written[20] == data[19]);
  byte sum = 0;
  for (size_t i = 0; i < 21; i++)
    sum += port.written[i];
  CHECK(port.written[21] == (byte)((0xfc - sum) & 0xff));

  // Oversized payload is truncated instead of overflowing the stack buffer.
  port.written.clear();
  hp.sendCustomPacket(data, 40);
  CHECK(port.written.size() == (size_t)HeatPump::PACKET_LEN);

  // Short payload only reads what it was given.
  port.written.clear();
  hp.sendCustomPacket(data, 3);
  CHECK(port.written.size() == 5);

  port.written.clear();
  hp.sendCustomPacket(data, 0);
  hp.sendCustomPacket(nullptr, 5);
  CHECK(port.written.empty());
}

static std::vector<uint8_t> makeStatusPacket(bool operating, uint8_t freq, uint16_t powerW)
{
  std::vector<uint8_t> p = {0xfc, 0x62, 0x01, 0x30, 0x10};
  std::vector<uint8_t> data(16, 0);
  data[0] = 0x06;
  data[3] = freq;
  data[4] = operating ? 1 : 0;
  data[5] = (uint8_t)(powerW >> 8);
  data[6] = (uint8_t)(powerW & 0xff);
  p.insert(p.end(), data.begin(), data.end());
  int sum = 0;
  for (uint8_t b : p)
    sum += b;
  p.push_back((uint8_t)((0xfc - sum) & 0xff));
  return p;
}

static void test_status_power_synced_on_callback()
{
  HeatPump hp;
  HardwareSerial port;
  hp._HardSerial = &port;

  int callbackPower = -1;
  hp.setStatusChangedCallback([&](heatpumpStatus st) { callbackPower = st.power; });

  // operating changes false -> true: callback branch.
  port.queue(makeStatusPacket(true, 40, 1234));
  int type = hp.readPacket(true);
  CHECK(type == hp.RCVD_PKT_STATUS);
  CHECK(hp.getStatus().power == 1234);
  CHECK(callbackPower == 1234);

  // No change in operating: else branch, power still tracked.
  port.queue(makeStatusPacket(true, 41, 987));
  hp.readPacket(true);
  CHECK(hp.getStatus().power == 987);
}

static void test_connect_keeps_uart_pins()
{
  HeatPump hp;
  HardwareSerial port;
  // No response queued: 2400 bps fails and falls back to 9600 bps.
  hp.connect(&port, 44, 43);
  CHECK(port.lastBaud == 9600);
  CHECK(port.lastRx == 44 && port.lastTx == 43);

  // Reconnect from sync() passes NULL and no pins.
  port.lastRx = port.lastTx = -1;
  hp.connect(NULL);
  CHECK(port.lastRx == 44 && port.lastTx == 43);
}

// ---------------------------------------------------------------------------
// Atomic storage
// ---------------------------------------------------------------------------

static void makeUnitDoc(JsonDocument &doc, const char *version)
{
  doc.clear();
  doc["unit_tempUnit"] = "cel";
  doc["min_temp"] = "16";
  doc["max_temp"] = "31";
  doc["temp_step"] = "1";
  doc["update_int"] = "300";
  doc["support_mode"] = "all";
  doc["beep"] = "1";
  doc["ledEnabled"] = "0";
  doc["login_password"] = version;
}

static void test_unit_doc_fits_and_old_size_overflowed()
{
  // The old StaticJsonDocument<128> could not hold the 9 unit fields
  // (worse on the device: 16-byte slots, Strings copied into the pool).
  StaticJsonDocument<128> small;
  makeUnitDoc(small, "a-long-enough-password");
  CHECK(small.overflowed());

  DynamicJsonDocument doc(512);
  makeUnitDoc(doc, "a-long-enough-password");
  CHECK(!doc.overflowed());
}

static void test_atomic_save_and_load()
{
  MockFS fs;
  DynamicJsonDocument doc(512), out(1024);
  makeUnitDoc(doc, "v1");
  CHECK(m2m::saveJsonAtomic(fs, "/unit.json", doc));
  CHECK(fs.exists("/unit.json"));
  CHECK(!fs.exists("/unit.json.tmp"));
  CHECK(m2m::loadJsonWithBackup(fs, "/unit.json", out) == m2m::LOAD_PRIMARY);
  CHECK(strcmp(out["login_password"] | "", "v1") == 0);
  CHECK(m2m::readJsonIntOrDefault(out["update_int"], 15, 1, 3600) == 300);

  makeUnitDoc(doc, "v2");
  CHECK(m2m::saveJsonAtomic(fs, "/unit.json", doc));
  CHECK(fs.exists("/unit.json.bak"));
  CHECK(m2m::loadJsonWithBackup(fs, "/unit.json", out) == m2m::LOAD_PRIMARY);
  CHECK(strcmp(out["login_password"] | "", "v2") == 0);

  // Overflowed documents are never written.
  StaticJsonDocument<16> tiny;
  makeUnitDoc(tiny, "v3");
  CHECK(!m2m::saveJsonAtomic(fs, "/unit.json", tiny));
  CHECK(m2m::loadJsonWithBackup(fs, "/unit.json", out) == m2m::LOAD_PRIMARY);
  CHECK(strcmp(out["login_password"] | "", "v2") == 0);
}

static void test_atomic_power_loss_recovery()
{
  // Interrupt the save of v2 after every possible operation; after the
  // "reboot" the loader must always return a complete v1 or v2.
  int recoveredOld = 0, recoveredNew = 0;
  for (long k = 0; k < 20; k++)
  {
    MockFS fs;
    DynamicJsonDocument doc(512), out(1024);
    makeUnitDoc(doc, "v1");
    CHECK(m2m::saveJsonAtomic(fs, "/unit.json", doc));
    makeUnitDoc(doc, "v1b"); // creates a .bak too
    CHECK(m2m::saveJsonAtomic(fs, "/unit.json", doc));

    makeUnitDoc(doc, "v2");
    fs.opsLeft = k;
    bool crashed = false;
    try
    {
      m2m::saveJsonAtomic(fs, "/unit.json", doc);
    }
    catch (const PowerLoss &)
    {
      crashed = true;
    }
    fs.opsLeft = -1;

    m2m::LoadSource src = m2m::loadJsonWithBackup(fs, "/unit.json", out);
    CHECK(src != m2m::LOAD_NONE);
    const char *pwd = out["login_password"] | "";
    bool ok = strcmp(pwd, "v1b") == 0 || strcmp(pwd, "v2") == 0 || strcmp(pwd, "v1") == 0;
    CHECK(ok);
    CHECK(strcmp(out["beep"] | "", "1") == 0); // fields never silently dropped
    if (strcmp(pwd, "v2") == 0)
      recoveredNew++;
    else
      recoveredOld++;
    if (!crashed)
    {
      CHECK(strcmp(pwd, "v2") == 0);
      break;
    }
  }
  CHECK(recoveredOld > 0);
  CHECK(recoveredNew > 0);
}

static void test_load_rejects_garbage_and_bounds()
{
  MockFS fs;
  DynamicJsonDocument out(256);
  fs.files["/wifi.json"] = "{\"ap_ssid\":\"abc\""; // truncated JSON, no NUL anywhere
  CHECK(m2m::loadJsonWithBackup(fs, "/wifi.json", out) == m2m::LOAD_NONE);

  fs.files["/wifi.json.bak"] = "{\"ap_ssid\":\"good\"}";
  CHECK(m2m::loadJsonWithBackup(fs, "/wifi.json", out) == m2m::LOAD_BACKUP);
  CHECK(strcmp(out["ap_ssid"] | "", "good") == 0);

  fs.files["/big.json"] = std::string(5000, ' ');
  CHECK(!m2m::readJsonFileSingle(fs, "/big.json", out));

  // Legacy files written by older firmware (same names) still load.
  fs.files["unit.json"] = "{\"unit_tempUnit\":\"fah\",\"update_int\":\"15\",\"min_temp\":\"16.00\"}";
  DynamicJsonDocument legacy(1024);
  CHECK(m2m::loadJsonWithBackup(fs, "unit.json", legacy) == m2m::LOAD_PRIMARY);
  CHECK(m2m::readJsonIntOrDefault(legacy["min_temp"], 0, 0, 60) == 16);
}

static void test_read_json_int()
{
  StaticJsonDocument<256> d;
  deserializeJson(d, "{\"a\":\"15\",\"b\":\"300\",\"c\":\"abc\",\"e\":16.4,\"f\":\"16.00\",\"g\":99999}");
  CHECK(m2m::readJsonIntOrDefault(d["a"], 1, 1, 3600) == 15);
  CHECK(m2m::readJsonIntOrDefault(d["b"], 1, 1, 3600) == 300); // no uint8_t wrap
  CHECK(m2m::readJsonIntOrDefault(d["c"], 7, 1, 3600) == 7);
  CHECK(m2m::readJsonIntOrDefault(d["missing"], 7, 1, 3600) == 7);
  CHECK(m2m::readJsonIntOrDefault(d["e"], 0, 0, 60) == 16);
  CHECK(m2m::readJsonIntOrDefault(d["f"], 0, 0, 60) == 16);
  CHECK(m2m::readJsonIntOrDefault(d["g"], 15, 1, 3600) == 15); // out of range
}

// ---------------------------------------------------------------------------
// HA discovery
// ---------------------------------------------------------------------------

static m2m::HaDiscoveryContext makeContext(size_t nameLen, size_t topicLen, bool fahrenheit)
{
  m2m::HaDiscoveryContext c;
  std::string name(nameLen, 'n');
  std::string topic(topicLen, 't');
  String base = String(topic.c_str()) + "/" + name.c_str();
  c.id = "a1b2c3";
  c.deviceName = name.c_str();
  c.swVersion = "Mitsubishi2MQTT magi's edition (2025.1.0)";
  c.hwVersion = "WiFiKit Serial 1.2";
  c.configUrl = "http://192.168.100.200";
  c.availabilityReport = true;
  c.availabilityTopic = base + "/availability";
  c.useFahrenheit = fahrenheit;
  c.tempStep = "0.5";
  c.powerSetTopic = base + "/power/set";
  c.modeSetTopic = base + "/mode/set";
  c.tempSetTopic = base + "/temp/set";
  c.fanSetTopic = base + "/fan/set";
  c.vaneSetTopic = base + "/vane/set";
  c.wideVaneSetTopic = base + "/wideVane/set";
  c.stateTopic = base + "/state";
  c.energySetTopic = base + "/energy/set";
  c.ledSetTopic = base + "/led/set";
  c.beepSetTopic = base + "/beep/set";
  c.unitSettingsTopic = base + "/unitSettings";
  return c;
}

typedef void (*FillFn)(JsonDocument &, const m2m::HaDiscoveryContext &, const char *);

static void test_ha_discovery_fits()
{
  const FillFn fills[] = {m2m::haFillClimate, m2m::haFillRoomTemp, m2m::haFillPower,
                          m2m::haFillEnergy, m2m::haFillEnergyReset, m2m::haFillVaneVertical,
                          m2m::haFillVaneHorizontal, m2m::haFillLedSwitch, m2m::haFillBeepSwitch};
  // Host slots are 32 bytes (device: 16), so passing here leaves headroom on the device.
  const size_t sizes[][2] = {{15, 15}, {32, 32}};
  for (auto &sz : sizes)
  {
    for (int f = 0; f < 2; f++)
    {
      m2m::HaDiscoveryContext c = makeContext(sz[0], sz[1], f == 1);
      DynamicJsonDocument doc(m2m::HA_DISCOVERY_DOC_SIZE);
      for (FillFn fill : fills)
      {
        fill(doc, c, "mdi:test");
        CHECK(!doc.overflowed());
        CHECK(doc["device"]["name"] == c.deviceName.c_str());
        CHECK(doc["avty_t"] == c.availabilityTopic.c_str());
        CHECK(doc["pl_avail"] == "online");
        CHECK(doc["pl_not_avail"] == "offline");
      }
    }
  }

  // Absurd lengths are detected (and skipped by publishHADiscovery) rather
  // than producing a truncated payload.
  m2m::HaDiscoveryContext huge = makeContext(400, 400, false);
  DynamicJsonDocument doc(m2m::HA_DISCOVERY_DOC_SIZE);
  m2m::haFillClimate(doc, huge, "mdi:test");
  CHECK(doc.overflowed());
}

static void test_ha_discovery_content()
{
  m2m::HaDiscoveryContext c = makeContext(10, 15, false);
  DynamicJsonDocument doc(m2m::HA_DISCOVERY_DOC_SIZE);

  m2m::haFillPower(doc, c, "i");
  CHECK(doc["state_class"] == "measurement");
  CHECK(doc["device_class"] == "power");

  m2m::haFillRoomTemp(doc, c, "i");
  CHECK(doc["device_class"] == "temperature");
  CHECK(doc["state_class"] == "measurement");

  m2m::haFillEnergy(doc, c, "i");
  CHECK(doc["state_class"] == "total_increasing");

  c.availabilityReport = false;
  m2m::haFillPower(doc, c, "i");
  CHECK(doc["avty_t"].isNull());

  c.supportHeatMode = false;
  m2m::haFillClimate(doc, c, "i");
  bool hasHeat = false;
  for (JsonVariant m : doc["modes"].as<JsonArray>())
    hasHeat |= (m == "heat");
  CHECK(!hasHeat);
}

// ---------------------------------------------------------------------------
// Timing / energy
// ---------------------------------------------------------------------------

static void test_millis_rollover()
{
  const unsigned long MAX = ULONG_MAX;
  // 16 ms have passed across the wrap.
  CHECK(m2m::intervalElapsed(5, MAX - 10, 16));
  CHECK(!m2m::intervalElapsed(5, MAX - 10, 17));
  // The old "millis() > start + interval" form gets this wrong.
  unsigned long start = MAX - 100, interval = 300000;
  unsigned long deadline = start + interval; // wraps to a small number
  unsigned long now = start + 50;            // only 50 ms later
  CHECK(now > deadline);                     // old check: "timed out" (wrong)
  CHECK(!m2m::intervalElapsed(now, start, interval));

  // 32-bit device arithmetic, explicitly.
  uint32_t s32 = 0xFFFFFFF0u, n32 = 0x00000010u;
  CHECK((uint32_t)(n32 - s32) == 0x20u);
}

static void test_energy_accumulation_and_reset()
{
  CHECK(fabsf(m2m::energyIncrementKWh(1000, 3600000UL) - 1.0f) < 1e-6f);
  // 15.9 s at 2 kW: fractional seconds are kept.
  CHECK(fabsf(m2m::energyIncrementKWh(2000, 15900UL) - (2000.0f * 15.9f / 3600.0f / 1000.0f)) < 1e-7f);
  CHECK(m2m::energyIncrementKWh(-5, 1000) == 0.0f);
  // Corrupted CN105 readings (e.g. 0xFFFF = 65535 W) are ignored.
  CHECK(m2m::energyIncrementKWh(65535, 3600000UL) == 0.0f);
  CHECK(m2m::energyIncrementKWh(m2m::MAX_PLAUSIBLE_POWER_W, 3600000UL) > 0.0f);

  const unsigned long interval = 10 * 60000UL;
  // Normal growth.
  CHECK(m2m::shouldSaveEnergy(120.2f, 120.0f, 2 * interval, interval - 1, 0.1f, interval));
  CHECK(!m2m::shouldSaveEnergy(120.05f, 120.0f, 3 * interval, 0, 0.1f, interval));
  // After Energy Reset the value drops: must still be saved.
  CHECK(m2m::shouldSaveEnergy(0.5f, 120.0f, 3 * interval, 0, 0.1f, interval));
  // ...but not more often than the interval.
  CHECK(!m2m::shouldSaveEnergy(0.5f, 120.0f, 100, 0, 0.1f, interval));
}

// ---------------------------------------------------------------------------
// HTML / session helpers
// ---------------------------------------------------------------------------

static void test_html_and_session_helpers()
{
  CHECK(m2m::htmlEscape("a'b\"<c>&") == "a&#39;b&quot;&lt;c&gt;&amp;");

  CHECK(m2m::extractCookie("a=1; M2MSESSIONID=abc123; x=y", "M2MSESSIONID") == "abc123");
  CHECK(m2m::extractCookie("XM2MSESSIONID=bad", "M2MSESSIONID") == "");
  CHECK(m2m::extractCookie("M2MSESSIONID=1", "M2MSESSIONID") == "1");
  CHECK(m2m::extractCookie("", "M2MSESSIONID") == "");

  CHECK(m2m::constantTimeEquals("abc", 3, "abc", 3));
  CHECK(!m2m::constantTimeEquals("abc", 3, "abd", 3));
  CHECK(!m2m::constantTimeEquals("abc", 3, "abcd", 4));
  CHECK(!m2m::constantTimeEquals("", 0, "x", 1));
  // An empty session token must never match an empty cookie by accident:
  // is_authenticated() checks sessionToken.length() first.
  CHECK(m2m::constantTimeEquals("", 0, "", 0));

  uint8_t bytes[16];
  for (int i = 0; i < 16; i++)
    bytes[i] = (uint8_t)(i * 17);
  String hex = m2m::bytesToHex(bytes, 16);
  CHECK(hex.length() == 32);
  CHECK(hex == "00112233445566778899aabbccddeeff");

  CHECK(m2m::urlHost("http://192.168.1.5/control?x=1") == "192.168.1.5");
  CHECK(m2m::urlHost("HTTP://Hvac_1234.local:8080/") == "hvac_1234.local:8080");
  CHECK(m2m::urlHost("null") == "");

  CHECK(!m2m::isValidApPassword("1234567"));
  CHECK(m2m::isValidApPassword("12345678"));
  CHECK(!m2m::isValidApPassword(String(std::string(64, 'x').c_str())));
}

int main()
{
  RUN(test_logger_format_string_safety);
  RUN(test_logger_long_lines_are_bounded);
  RUN(test_logger_rotation_stress);
  RUN(test_send_custom_packet_bounds);
  RUN(test_status_power_synced_on_callback);
  RUN(test_connect_keeps_uart_pins);
  RUN(test_unit_doc_fits_and_old_size_overflowed);
  RUN(test_atomic_save_and_load);
  RUN(test_atomic_power_loss_recovery);
  RUN(test_load_rejects_garbage_and_bounds);
  RUN(test_read_json_int);
  RUN(test_ha_discovery_fits);
  RUN(test_ha_discovery_content);
  RUN(test_millis_rollover);
  RUN(test_energy_accumulation_and_reset);
  RUN(test_html_and_session_helpers);

  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
