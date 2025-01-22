/*
  HeatPump.cpp - Mitsubishi Heat Pump control library for Arduino
  Copyright (c) 2017 Al Betschart.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "HeatPump.h"

String getHEXformatted(uint8_t *bytes, size_t len)
{
  String res;
  char buf[5];
  for (size_t i = 0; i < len; i++)
  {
    if (i > 0)
      res += ':';
    sprintf(buf, "%02X", bytes[i]);
    res += buf;
  }
  return res;
}

// Structures //////////////////////////////////////////////////////////////////

bool operator==(const heatpumpSettings &lhs, const heatpumpSettings &rhs)
{
  return lhs.power == rhs.power &&
         lhs.mode == rhs.mode &&
         lhs.temperature == rhs.temperature &&
         lhs.fan == rhs.fan &&
         lhs.vane == rhs.vane &&
         lhs.wideVane == rhs.wideVane &&
         lhs.iSee == rhs.iSee;
}

bool operator!=(const heatpumpSettings &lhs, const heatpumpSettings &rhs)
{
  return lhs.power != rhs.power ||
         lhs.mode != rhs.mode ||
         lhs.temperature != rhs.temperature ||
         lhs.fan != rhs.fan ||
         lhs.vane != rhs.vane ||
         lhs.wideVane != rhs.wideVane ||
         lhs.iSee != rhs.iSee;
}

bool operator!(const heatpumpSettings &settings)
{
  return !settings.power &&
         !settings.mode &&
         !settings.temperature &&
         !settings.fan &&
         !settings.vane &&
         !settings.wideVane &&
         !settings.iSee;
}

bool operator==(const heatpumpTimers &lhs, const heatpumpTimers &rhs)
{
  return lhs.mode == rhs.mode &&
         lhs.onMinutesSet == rhs.onMinutesSet &&
         lhs.onMinutesRemaining == rhs.onMinutesRemaining &&
         lhs.offMinutesSet == rhs.offMinutesSet &&
         lhs.offMinutesRemaining == rhs.offMinutesRemaining;
}

bool operator!=(const heatpumpTimers &lhs, const heatpumpTimers &rhs)
{
  return lhs.mode != rhs.mode ||
         lhs.onMinutesSet != rhs.onMinutesSet ||
         lhs.onMinutesRemaining != rhs.onMinutesRemaining ||
         lhs.offMinutesSet != rhs.offMinutesSet ||
         lhs.offMinutesRemaining != rhs.offMinutesRemaining;
}

// Constructor /////////////////////////////////////////////////////////////////

HeatPump::HeatPump()
{
  lastSend = 0;
  lastSendUpdate = 0;
  infoMode = 0;
  lastRecv = millis() - (PACKET_SENT_INTERVAL_MS * 60);
  autoUpdate = false;
  firstRun = true;
  tempMode = false;
  // waitForRead = false;
  externalUpdate = false;
  wideVaneAdj = false;
  functions = heatpumpFunctions();
}

// Public Methods //////////////////////////////////////////////////////////////

#if defined(__WIFIKITSAMD__)

bool HeatPump::connect(Uart *serial, int bitrate)
{

  if (serial != NULL)
  {
    _HardSerial = serial;
  }
  _HardSerial->begin(bitrate, SERIAL_8E1);
  _HardSerial->setTimeout(PACKET_RESPONSE_WAIT_TIME);

  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  if (onConnectCallback)
  {
    onConnectCallback();
  }

  // settle before we start sending packets
  delay(2000);

  // send the CONNECT packet twice - need to copy the CONNECT packet locally
  byte packet[CONNECT_LEN];
  memcpy(packet, CONNECT, CONNECT_LEN);

  writePacket(packet, CONNECT_LEN); //Send connect command
  int packetType = readPacket(true);  //Read response packet (blocking)
  
  if (packetType != RCVD_PKT_CONNECT_SUCCESS && bitrate == 2400)
  {
    return connect(serial, 9600);
  }
  return packetType == RCVD_PKT_CONNECT_SUCCESS;
}

#else

bool HeatPump::connect(HardwareSerial *serial)
{
  return connect(serial, -1, -1);
}

bool HeatPump::connect(HardwareSerial *serial, int bitrate)
{
  return connect(serial, bitrate, -1, -1);
}

bool HeatPump::connect(HardwareSerial *serial, int rx, int tx)
{
  return connect(serial, 0, rx, tx);
}

bool HeatPump::connect(HardwareSerial *serial, int bitrate, int rx, int tx)
{
  if (serial != NULL)
  {
    _HardSerial = serial;
  }
  bool retry = false;
  if (bitrate == 0)
  {
    bitrate = 2400;
    retry = true;
  }
  connected = false;
  Serial.printf("Connecting at baud rate %d\n", bitrate);
  if (rx >= 0 && tx >= 0)
  {
    #if defined(ESP32)
    _HardSerial->begin(bitrate, SERIAL_8E1, rx, tx);
    #else
    _HardSerial->begin(bitrate, SERIAL_8E1);
    #endif
  }
  else
  {
    _HardSerial->begin(bitrate, SERIAL_8E1);
  }

  _HardSerial->setTimeout(PACKET_RESPONSE_WAIT_TIME);
  
  if (onConnectCallback)
  {
    onConnectCallback();
  }

  // settle before we start sending packets
  delay(2000);

  // send the CONNECT packet twice - need to copy the CONNECT packet locally
  byte packet[CONNECT_LEN];
  memcpy(packet, CONNECT, CONNECT_LEN);

  writePacket(packet, CONNECT_LEN); //Send connect command
  int packetType = readPacket(true);  //Read response packet (blocking)
  
  if (packetType != RCVD_PKT_CONNECT_SUCCESS && bitrate == 2400)
  {
    return connect(serial, 9600);
  }
  return packetType == RCVD_PKT_CONNECT_SUCCESS;

  
}

#endif

bool HeatPump::update()
{

  if (!canSend(false))
  {
    // SerialUSB.println("Could not send yet.");
    return false;
  }

  // SerialUSB.println("Send new command");

  //Set flag if the current update is a power setting (will need to add more delay to next command)
  powerSettingUpdate = wantedSettings.power != currentSettings.power;
  // SerialUSB.println("Power setting update = " + powerSettingUpdate);
  if (powerSettingUpdate){
    packet_sent_delay_interval_ms = PACKET_SENT_INTERVAL_MS + 10000;
  }else{
    packet_sent_delay_interval_ms = PACKET_SENT_INTERVAL_MS;
  }


  byte packet[PACKET_LEN] = {};
  createPacket(packet, wantedSettings);
  writePacket(packet, PACKET_LEN);

  currentSettings = wantedSettings;
  updating = true;
  lastSendUpdate = millis();
  return true;
}

// Default PACKET_TYPE_DEFAULT = 99;
void HeatPump::sync(byte packetType)
{
  if ((!connected) || (millis() - lastRecv > (PACKET_SENT_INTERVAL_MS * 12)))
  {
    connected = false;
    connect(NULL);
  }
  else if (sendPending()) // Command to send is pending.
  { 
    update();
  }
  else if (updating) // Previously send a command, then check for ack.
  {
      int packetType = readPacket();

      if (packetType == RCVD_PKT_UPDATE_SUCCESS) // 0xFC 0x61
      {
        // call sync() to get the latest settings from the heatpump for autoUpdate, which should now have the updated settings
        if (autoUpdate)
        { // this sync will happen regardless, but autoUpdate needs it sooner than later.
          while (!canSend(true))
          {
            delay(10);
          }
          sync(RQST_PKT_SETTINGS);
        }

        // SerialUSB.println("A/C update Success");
        return;
      }
      else
      {
        // SerialUSB.println("A/C update failed");
        return;
      }
  }
  else if (canRead())   //Periodic serial read for any incoming data.
  {
    readPacket();
  }
  else if (autoUpdate && !firstRun && wantedSettings != currentSettings && packetType == PACKET_TYPE_DEFAULT)
  {
    update();
  }

  
  if (canSend(true))//    Fetch new A/C status if possible
  {
    byte packet[PACKET_LEN] = {};
    createInfoPacket(packet, packetType);
    writePacket(packet, PACKET_LEN);
  }
  
}

bool HeatPump::sendPending()
{
  return wantedSettings != currentSettings;
}

void HeatPump::enableExternalUpdate()
{
  autoUpdate = true;
  externalUpdate = true;
}

void HeatPump::disableExternalUpdate()
{
  externalUpdate = false;
}

void HeatPump::enableAutoUpdate()
{
  autoUpdate = true;
}

void HeatPump::disableAutoUpdate()
{
  autoUpdate = false;
}

heatpumpSettings HeatPump::getSettings()
{
  return currentSettings;
}

bool HeatPump::isConnected()
{
  return connected;
}

void HeatPump::setSettings(heatpumpSettings settings)
{
  setPowerSetting(settings.power);
  setModeSetting(settings.mode);
  setTemperature(settings.temperature);
  setFanSpeed(settings.fan);
  setVaneSetting(settings.vane);
  setWideVaneSetting(settings.wideVane);

  
}

bool HeatPump::getPowerSettingBool()
{
  return currentSettings.power == POWER_MAP[1] ? true : false;
}

void HeatPump::setPowerSetting(bool setting)
{

  // if (strcmp(wantedSettings.power, lookupByteMapIndex(POWER_MAP, 2, POWER_MAP[setting ? 1 : 0]) > -1 ? POWER_MAP[setting ? 1 : 0] : POWER_MAP[0] == 0)){
  //   powerSettingUpdate = false;
  // }else{
  //   powerSettingUpdate = true;
  // }

  wantedSettings.power = lookupByteMapIndex(POWER_MAP, 2, POWER_MAP[setting ? 1 : 0]) > -1 ? POWER_MAP[setting ? 1 : 0] : POWER_MAP[0];
}

const char *HeatPump::getPowerSetting()
{
  return currentSettings.power;
}

void HeatPump::setPowerSetting(const char *setting)
{
  int index = lookupByteMapIndex(POWER_MAP, 2, setting);
  if (index < 0)
  {
    index = 0;
  }


  // if (strcmp(wantedSettings.power, POWER_MAP[index]) == 0){
  //     powerSettingUpdate = false;
  // }else{
  //   powerSettingUpdate = true;
  // }

  wantedSettings.power = POWER_MAP[index];

}

const char *HeatPump::getModeSetting()
{
  return currentSettings.mode;
}

void HeatPump::setModeSetting(const char *setting)
{
  int index = lookupByteMapIndex(MODE_MAP, 5, setting);
  if (index > -1)
  {
    wantedSettings.mode = MODE_MAP[index];
  }
  else
  {
    wantedSettings.mode = MODE_MAP[0];
  }
}

float HeatPump::getTemperature()
{
  return currentSettings.temperature;
}

void HeatPump::setTemperature(float setting)
{
  if (!tempMode)
  {
    wantedSettings.temperature = lookupByteMapIndex(TEMP_MAP, 16, (int)(setting + 0.5)) > -1 ? setting : TEMP_MAP[0];
  }
  else
  {
    setting = setting * 2;
    setting = round(setting);
    setting = setting / 2;
    wantedSettings.temperature = setting < 10 ? 10 : (setting > 31 ? 31 : setting);
  }
}

void HeatPump::setRemoteTemperature(float setting)
{
  byte packet[PACKET_LEN] = {};

  prepareSetPacket(packet, PACKET_LEN);

  packet[5] = 0x07;
  if (setting > 0)
  {
    packet[6] = 0x01;
    setting = setting * 2;
    setting = round(setting);
    setting = setting / 2;
    float temp1 = 3 + ((setting - 10) * 2);
    packet[7] = (int)temp1;
    float temp2 = (setting * 2) + 128;
    packet[8] = (int)temp2;
  }
  else
  {
    packet[6] = 0x00;
    packet[8] = 0x80; // MHK1 send 80, even though it could be 00, since ControlByte is 00
  }
  // add the checksum
  byte chkSum = checkSum(packet, 21);
  packet[21] = chkSum;
  while (!canSend(false))
  {
    delay(10);
  }
  writePacket(packet, PACKET_LEN);
}

const char *HeatPump::getFanSpeed()
{
  return currentSettings.fan;
}

void HeatPump::setFanSpeed(const char *setting)
{
  int index = lookupByteMapIndex(FAN_MAP, 6, setting);
  if (index > -1)
  {
    wantedSettings.fan = FAN_MAP[index];
  }
  else
  {
    wantedSettings.fan = FAN_MAP[0];
  }
}

const char *HeatPump::getVaneSetting()
{
  return currentSettings.vane;
}

void HeatPump::setVaneSetting(const char *setting)
{
  int index = lookupByteMapIndex(VANE_MAP, 7, setting);
  if (index > -1)
  {
    wantedSettings.vane = VANE_MAP[index];
  }
  else
  {
    wantedSettings.vane = VANE_MAP[0];
  }
}

const char *HeatPump::getWideVaneSetting()
{
  return currentSettings.wideVane;
}

void HeatPump::setWideVaneSetting(const char *setting)
{
  int index = lookupByteMapIndex(WIDEVANE_MAP, 7, setting);
  if (index > -1)
  {
    wantedSettings.wideVane = WIDEVANE_MAP[index];
  }
  else
  {
    wantedSettings.wideVane = WIDEVANE_MAP[0];
  }
}

bool HeatPump::getIseeBool()
{ // no setter yet
  return currentSettings.iSee;
}

heatpumpStatus HeatPump::getStatus()
{
  return currentStatus;
}

float HeatPump::getRoomTemperature()
{
  return currentStatus.roomTemperature;
}

bool HeatPump::getOperating()
{
  return currentStatus.operating;
}

float HeatPump::FahrenheitToCelsius(int tempF)
{
  float temp = (tempF - 32) / 1.8;
  return ((float)round(temp * 2)) / 2; // Round to nearest 0.5C
}

int HeatPump::CelsiusToFahrenheit(float tempC)
{
  float temp = (tempC * 1.8) + 32; // round up if heat, down if cool or any other mode
  return (int)(temp + 0.5);
}

void HeatPump::setOnConnectCallback(ON_CONNECT_CALLBACK_SIGNATURE)
{
  this->onConnectCallback = onConnectCallback;
}

void HeatPump::setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE)
{
  this->settingsChangedCallback = settingsChangedCallback;
}

void HeatPump::setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE)
{
  this->statusChangedCallback = statusChangedCallback;
}

void HeatPump::setPacketCallback(PACKET_CALLBACK_SIGNATURE)
{
  this->packetCallback = packetCallback;
}

void HeatPump::setRoomTempChangedCallback(ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE)
{
  this->roomTempChangedCallback = roomTempChangedCallback;
}

// #### WARNING, THE FOLLOWING METHOD CAN F--K YOUR HP UP, USE WISELY ####
void HeatPump::sendCustomPacket(byte data[], int packetLength)
{
  while (!canSend(false))
  {
    delay(10);
  }

  packetLength += 2;                                                      // +2 for first header byte and checksum
  packetLength = (packetLength > PACKET_LEN) ? PACKET_LEN : packetLength; // ensure we are not exceeding PACKET_LEN
  byte packet[packetLength];
  packet[0] = HEADER[0]; // add first header byte

  // add data
  for (int i = 0; i < packetLength; i++)
  {
    packet[(i + 1)] = data[i];
  }

  // add checksum
  byte chkSum = checkSum(packet, (packetLength - 1));
  packet[(packetLength - 1)] = chkSum;

  writePacket(packet, packetLength);
}

// Private Methods //////////////////////////////////////////////////////////////

int HeatPump::lookupByteMapIndex(const int valuesMap[], int len, int lookupValue)
{
  for (int i = 0; i < len; i++)
  {
    if (valuesMap[i] == lookupValue)
    {
      return i;
    }
  }
  return -1;
}

int HeatPump::lookupByteMapIndex(const char *valuesMap[], int len, const char *lookupValue)
{
  for (int i = 0; i < len; i++)
  {
    if (strcasecmp(valuesMap[i], lookupValue) == 0)
    {
      return i;
    }
  }
  return -1;
}

const char *HeatPump::lookupByteMapValue(const char *valuesMap[], const byte byteMap[], int len, byte byteValue)
{
  for (int i = 0; i < len; i++)
  {
    if (byteMap[i] == byteValue)
    {
      return valuesMap[i];
    }
  }
  return valuesMap[0];
}

int HeatPump::lookupByteMapValue(const int valuesMap[], const byte byteMap[], int len, byte byteValue)
{
  for (int i = 0; i < len; i++)
  {
    if (byteMap[i] == byteValue)
    {
      return valuesMap[i];
    }
  }
  return valuesMap[0];
}

bool HeatPump::canSend(bool isInfo)

{
  // return (millis() - (isInfo ? PACKET_INFO_INTERVAL_MS : PACKET_SENT_INTERVAL_MS)) > lastSend;

  if (isInfo)
  {
     if (powerSettingUpdate  && millis() - lastSendUpdate < PACKET_SENT_INTERVAL_MS + 10000 ){
    //  if ( millis() - lastSendUpdate < 10000 ){
        return false;
     }
    // return millis() - lastSend > PACKET_INFO_INTERVAL_MS;
    return millis() - lastSend > PACKET_INFO_INTERVAL_MS;
  }
  else
  {
    return millis() - lastSendUpdate > packet_sent_delay_interval_ms && millis() - lastSend > PACKET_INFO_INTERVAL_MS;

  }
}

bool HeatPump::canRead()
{
  // return (waitForRead && (millis() - PACKET_SENT_INTERVAL_MS) > lastSend);
  // return (waitForRead && (millis() - PACKET_RESPONSE_WAIT_TIME) > lastSend);
  return (millis() - PACKET_RESPONSE_WAIT_TIME) > lastSend;
}

byte HeatPump::checkSum(byte bytes[], int len)
{
  byte sum = 0;
  for (int i = 0; i < len; i++)
  {
    sum += bytes[i];
  }
  return (0xfc - sum) & 0xff;
}


static void readHPsettings(heatpumpSettings hpSettings)
{

  #if !defined(CDC_DISABLED) && defined(__WIFIKITSAMD__)
  SerialUSB.println("\tPower: " + String(hpSettings.power));
  SerialUSB.println("\tMode: " + String(hpSettings.mode));
  float degc = hpSettings.temperature;
  SerialUSB.println("\tTarget: " + String(degc, 1));
  SerialUSB.println("\tFan: " + String(hpSettings.fan));
  SerialUSB.println("\tSwing: H:" + String(hpSettings.wideVane) + " V:" + String(hpSettings.vane));
  #endif
}

void HeatPump::createPacket(byte *packet, heatpumpSettings settings)
{
  prepareSetPacket(packet, PACKET_LEN);
  // SerialUSB.println("Local Settings");
  readHPsettings(currentSettings);
  // SerialUSB.println("Send Settings");
  readHPsettings(settings);

  if (settings.power != currentSettings.power)
  {
    packet[8] = POWER[lookupByteMapIndex(POWER_MAP, 2, settings.power)];
    packet[6] += CONTROL_PACKET_1[0];
  }

  if (settings.mode != currentSettings.mode)
  {
    packet[9] = MODE[lookupByteMapIndex(MODE_MAP, 5, settings.mode)];
    packet[6] += CONTROL_PACKET_1[1];
  }
  if (!tempMode && settings.temperature != currentSettings.temperature)
  {
    packet[10] = TEMP[lookupByteMapIndex(TEMP_MAP, 16, settings.temperature)];
    packet[6] += CONTROL_PACKET_1[2];
  }
  else if (tempMode && settings.temperature != currentSettings.temperature)
  {
    float temp = (settings.temperature * 2) + 128;
    packet[19] = (int)temp;
    packet[6] += CONTROL_PACKET_1[2];
  }
  if (settings.fan != currentSettings.fan)
  {
    packet[11] = FAN[lookupByteMapIndex(FAN_MAP, 6, settings.fan)];
    packet[6] += CONTROL_PACKET_1[3];
  }
  if (settings.vane != currentSettings.vane)
  {
    packet[12] = VANE[lookupByteMapIndex(VANE_MAP, 7, settings.vane)];
    packet[6] += CONTROL_PACKET_1[4];
  }
  if (settings.wideVane != currentSettings.wideVane)
  {
    packet[18] = WIDEVANE[lookupByteMapIndex(WIDEVANE_MAP, 7, settings.wideVane)] | (wideVaneAdj ? 0x80 : 0x00);
    packet[7] += CONTROL_PACKET_2[0];
  }
  // add the checksum
  byte chkSum = checkSum(packet, 21);
  packet[21] = chkSum;
}

void HeatPump::setInfoModeIndex(int index)
{
  if (index < INFOMODE_LEN)
  {
    infoMode = index;
  }
}

void HeatPump::createInfoPacket(byte *packet, byte packetType)
{
  // add the header to the packet
  for (int i = 0; i < INFOHEADER_LEN; i++)
  {
    packet[i] = INFOHEADER[i];
  }

  // set the mode - settings or room temperature
  if (packetType != PACKET_TYPE_DEFAULT)
  {
    packet[5] = INFOMODE[packetType];
  }
  else
  {
    // request current infoMode, and increment for the next request
    packet[5] = INFOMODE[infoMode];
    if (infoMode == (INFOMODE_LEN - 1))
    {
      infoMode = 0;
    }
    else
    {
      infoMode++;
    }
  }

  // packet[5] = infomodecmd++ ; //FOR DEBUG

  // pad the packet out
  for (int i = 0; i < 15; i++)
  {
    packet[i + 6] = 0x00;
  }

  // add the checksum
  byte chkSum = checkSum(packet, 21);
  packet[21] = chkSum;
}

void HeatPump::writePacket(byte *packet, int length)
{

#ifdef ESP32
  Serial.println("CN105 >> " + getHEXformatted(packet, length));
#elif __WIFIKITSAMD__ 
  #ifndef CDC_DISABLED
  SerialUSB.println("CN105 >> " + getHEXformatted(packet, length));
  #endif
#endif

  for (int i = 0; i < length; i++)
  {
    _HardSerial->write((uint8_t)packet[i]);
  }

  if (packetCallback)
  {
    packetCallback(packet, length, (char *)"packetSent");
  }
  // waitForRead = true;
  lastSend = millis();

  // readPacket();
}

int HeatPump::readPacket(bool waitForPacket)
{
  byte header[INFOHEADER_LEN] = {};
  byte data[PACKET_LEN] = {};
  int dataSum = 0;
  byte checksum = 0;
  byte dataLength = 0;
  unsigned long startTime = millis();
  uint8_t len = 0;
  bool receiveSuccess = false;


  //Has incoming message in buffer.
  if (_HardSerial->available() > 0 || waitForPacket)
  {
    while (millis() - startTime < PACKET_RESPONSE_WAIT_TIME)
    {
      if (_HardSerial->available() > 0)
      {
        char c = _HardSerial->read();

        // Check packet
        if (len < 4 && len != 1)
        {
          if (c != HEADER[len])
          {
            // Serial.println("Header invalid");
            _HardSerial->flush();
            return RCVD_PKT_FAIL;
          }
        }
        if (len == 4)
        {
          dataLength = c;
        }

        // Store packet
        if (len <= 4)
        {
          header[len] = c;
        }
        else
        { // Data bytes + checksum byte
          // Serial.printf("Data elem %d, %x\n", len-5, c);
          data[len - 5] = c;
        }

        // End condition
        if (len == dataLength + 5)
        {
          receiveSuccess = true;
          len++;
          break;
        }

        len++;
      }
    }
  }else{
    return RCVD_PKT_FAIL; //No response
  }


  _HardSerial->flush();
  if (!receiveSuccess)
  {
#ifdef ESP32
    Serial.println("Wait read timeout");
#elif __WIFIKITSAMD__
    #ifndef CDC_DISABLED
    SerialUSB.println("Wait read timeout");
    #endif
#endif
    return RCVD_PKT_FAIL;
  }

  // sum up the header bytes...
  for (int i = 0; i < INFOHEADER_LEN; i++)
  {
    dataSum += header[i];
  }

  // ...and add to that the sum of the data bytes
  for (int i = 0; i < dataLength; i++)
  {
    dataSum += data[i];
  }

#ifdef ESP32
  Serial.println("CN105 << " + getHEXformatted(header, INFOHEADER_LEN) + "|" + getHEXformatted(data, dataLength + 1));
#elif __WIFIKITSAMD__
  #ifndef CDC_DISABLED
  SerialUSB.println("CN105 << " + getHEXformatted(header, INFOHEADER_LEN) + "|" + getHEXformatted(data, dataLength + 1));
  #endif
#endif

  // calculate checksum
  checksum = (0xfc - dataSum) & 0xff;

  if (data[dataLength] == checksum)
  {
    lastRecv = millis();
    if (packetCallback)
    {
      byte packet[37]; // we are going to put header[5] and data[32] into this, so the whole packet is sent to the callback
      for (int i = 0; i < INFOHEADER_LEN; i++)
      {
        packet[i] = header[i];
      }
      for (int i = 0; i < (dataLength + 1); i++)
      { // must be dataLength+1 to pick up checksum byte
        packet[(i + 5)] = data[i];
      }
      packetCallback(packet, PACKET_LEN, (char *)"packetRecv");
    }

    if (header[1] == 0x62)
    {

      // uint8_t command = data[0];
      // static std::map<uint8_t, String> mp;
      // String lastData = "";
      // String currentData  = getHEXformatted(data, dataLength);
      // bool changed = false;
      // if(mp.find(command) != mp.end()){
      //   lastData  = mp.find(command)->second;
      //   // Serial.print("Command " );
      //   // Serial.println(command,HEX);
      //   // Serial.println("Current" + currentData);
      //   // Serial.println("Last" + lastData);
      //   if (currentData != lastData){
      //     changed = true;
      //   }
      // }
      // mp[command] = currentData;
      //   Serial.print(changed?"[Changed]\n":"\n");
      //   if (changed){
      //     Serial.println("OLDAT << " + getHEXformatted(header, INFOHEADER_LEN) + "|" + lastData);
      //   }
      //   Serial.println();

      switch (data[0])
      {
      case 0x02:
      { // setting information
        heatpumpSettings receivedSettings;
        receivedSettings.power = lookupByteMapValue(POWER_MAP, POWER, 2, data[3]);
        receivedSettings.iSee = data[4] > 0x08 ? true : false;
        receivedSettings.mode = lookupByteMapValue(MODE_MAP, MODE, 5, receivedSettings.iSee ? (data[4] - 0x08) : data[4]);

        if (data[11] != 0x00)
        {
          int temp = data[11];
          temp -= 128;
          receivedSettings.temperature = (float)temp / 2;
          tempMode = true;
        }
        else
        {
          receivedSettings.temperature = lookupByteMapValue(TEMP_MAP, TEMP, 16, data[5]);
        }

        receivedSettings.fan = lookupByteMapValue(FAN_MAP, FAN, 6, data[6]);
        receivedSettings.vane = lookupByteMapValue(VANE_MAP, VANE, 7, data[7]);
        receivedSettings.wideVane = lookupByteMapValue(WIDEVANE_MAP, WIDEVANE, 7, data[10] & 0x0F);
        wideVaneAdj = (data[10] & 0xF0) == 0x80 ? true : false;

        if (settingsChangedCallback && receivedSettings != currentSettings)
        {
          currentSettings = receivedSettings;
          settingsChangedCallback();
        }
        else
        {
          currentSettings = receivedSettings;
        }

        // // if this is the first time we have synced with the heatpump, set wantedSettings to receivedSettings
        // if (firstRun || (autoUpdate && externalUpdate))
        // {
        //   wantedSettings = currentSettings;
        //   firstRun = false;
        // }

        wantedSettings = currentSettings;

        return RCVD_PKT_SETTINGS;
      }

      case 0x03:
      { // Room temperature reading
        heatpumpStatus receivedStatus;
        // Serial.printf("Mystery val 1 : %d\n",data[13]);

        if (data[6] != 0x00)
        {
          int temp = data[6];
          temp -= 128;
          receivedStatus.roomTemperature = (float)temp / 2;
        }
        else
        {
          receivedStatus.roomTemperature = lookupByteMapValue(ROOM_TEMP_MAP, ROOM_TEMP, 32, data[3]);
        }

        if ((statusChangedCallback || roomTempChangedCallback) && currentStatus.roomTemperature != receivedStatus.roomTemperature)
        {
          currentStatus.roomTemperature = receivedStatus.roomTemperature;

          if (statusChangedCallback)
          {
            statusChangedCallback(currentStatus);
          }

          if (roomTempChangedCallback)
          { // this should be deprecated - statusChangedCallback covers it
            roomTempChangedCallback(currentStatus.roomTemperature);
          }
        }
        else
        {
          currentStatus.roomTemperature = receivedStatus.roomTemperature;
        }

        return RCVD_PKT_ROOM_TEMP;
      }

      case 0x04:
      { // unknown
        break;
      }

      case 0x05:
      { // timer packet
        heatpumpTimers receivedTimers;

        receivedTimers.mode = lookupByteMapValue(TIMER_MODE_MAP, TIMER_MODE, 4, data[3]);
        receivedTimers.onMinutesSet = data[4] * TIMER_INCREMENT_MINUTES;
        receivedTimers.onMinutesRemaining = data[6] * TIMER_INCREMENT_MINUTES;
        receivedTimers.offMinutesSet = data[5] * TIMER_INCREMENT_MINUTES;
        receivedTimers.offMinutesRemaining = data[7] * TIMER_INCREMENT_MINUTES;

        // callback for status change
        if (statusChangedCallback && currentStatus.timers != receivedTimers)
        {
          currentStatus.timers = receivedTimers;
          statusChangedCallback(currentStatus);
        }
        else
        {
          currentStatus.timers = receivedTimers;
        }

        return RCVD_PKT_TIMER;
      }

      case 0x06:
      { // status
        heatpumpStatus receivedStatus;
        receivedStatus.operating = data[4];
        receivedStatus.compressorFrequency = data[3];
        uint16_t power = data[5] << 8;
        power += data[6];
        receivedStatus.power = power;
        // Serial.printf("Mystery val 2 : %d\n",mVal2);

        // callback for status change -- not triggered for compressor frequency at the moment
        if (statusChangedCallback && currentStatus.operating != receivedStatus.operating)
        {
          currentStatus.operating = receivedStatus.operating;
          currentStatus.compressorFrequency = receivedStatus.compressorFrequency;
          statusChangedCallback(currentStatus);
        }
        else
        {
          currentStatus.operating = receivedStatus.operating;
          currentStatus.compressorFrequency = receivedStatus.compressorFrequency;
          currentStatus.power = receivedStatus.power;
        }

        return RCVD_PKT_STATUS;
      }

      case 0x09:
      { // standby mode maybe?
        break;
      }

      case 0x20:
      case 0x22:
      {
        if (dataLength == 0x10)
        {
          if (data[0] == 0x20)
          {
            functions.setData1(&data[1]);
          }
          else
          {
            functions.setData2(&data[1]);
          }

          return RCVD_PKT_FUNCTIONS;
        }
        break;
      }
      }
    }

    if (header[1] == 0x61)
    { // Last update was successful
      updating = false;
      return RCVD_PKT_UPDATE_SUCCESS;
    }
    else if (header[1] == 0x7a)
    { // Last update was successful
      connected = true;
      // return RCVD_PKT_CONNECT_SUCCESS;
      return 1;
    }
  }

  return RCVD_PKT_FAIL;
}

void HeatPump::prepareInfoPacket(byte *packet, int length)
{
  memset(packet, 0, length * sizeof(byte));

  for (int i = 0; i < INFOHEADER_LEN && i < length; i++)
  {
    packet[i] = INFOHEADER[i];
  }
}

void HeatPump::prepareSetPacket(byte *packet, int length)
{
  memset(packet, 0, length * sizeof(byte));

  for (int i = 0; i < HEADER_LEN && i < length; i++)
  {
    packet[i] = HEADER[i];
  }
}

heatpumpFunctions HeatPump::getFunctions()
{
  functions.clear();

  byte packet1[PACKET_LEN] = {};
  byte packet2[PACKET_LEN] = {};

  prepareInfoPacket(packet1, PACKET_LEN);
  packet1[5] = FUNCTIONS_GET_PART1;
  packet1[21] = checkSum(packet1, 21);

  prepareInfoPacket(packet2, PACKET_LEN);
  packet2[5] = FUNCTIONS_GET_PART2;
  packet2[21] = checkSum(packet2, 21);

  while (!canSend(false))
  {
    delay(10);
  }
  writePacket(packet1, PACKET_LEN);
  readPacket();

  while (!canSend(false))
  {
    delay(10);
  }
  writePacket(packet2, PACKET_LEN);
  readPacket();

  // retry reading a few times in case responses were related
  // to other requests
  for (int i = 0; i < 5 && !functions.isValid(); ++i)
  {
    delay(100);
    readPacket();
  }

  return functions;
}

bool HeatPump::setFunctions(heatpumpFunctions const &functions)
{
  if (!functions.isValid())
  {
    return false;
  }

  byte packet1[PACKET_LEN] = {};
  byte packet2[PACKET_LEN] = {};

  prepareSetPacket(packet1, PACKET_LEN);
  packet1[5] = FUNCTIONS_SET_PART1;

  prepareSetPacket(packet2, PACKET_LEN);
  packet2[5] = FUNCTIONS_SET_PART2;

  functions.getData1(&packet1[6]);
  functions.getData2(&packet2[6]);

  // sanity check, we expect data byte 15 (index 20) to be 0
  if (packet1[20] != 0 || packet2[20] != 0)
    return false;

  // make sure all the other data bytes are set
  for (int i = 6; i < 20; ++i)
  {
    if (packet1[i] == 0 || packet2[i] == 0)
      return false;
  }

  packet1[21] = checkSum(packet1, 21);
  packet2[21] = checkSum(packet2, 21);

  while (!canSend(false))
  {
    delay(10);
  }
  writePacket(packet1, PACKET_LEN);
  readPacket();

  while (!canSend(false))
  {
    delay(10);
  }
  writePacket(packet2, PACKET_LEN);
  readPacket();

  return true;
}

heatpumpFunctions::heatpumpFunctions()
{
  clear();
}

bool heatpumpFunctions::isValid() const
{
  return _isValid1 && _isValid2;
}

void heatpumpFunctions::setData1(byte *data)
{
  memcpy(raw, data, 15);
  _isValid1 = true;
}

void heatpumpFunctions::setData2(byte *data)
{
  memcpy(raw + 15, data, 15);
  _isValid2 = true;
}

void heatpumpFunctions::getData1(byte *data) const
{
  memcpy(data, raw, 15);
}

void heatpumpFunctions::getData2(byte *data) const
{
  memcpy(data, raw + 15, 15);
}

void heatpumpFunctions::clear()
{
  memset(raw, 0, sizeof(raw));
  _isValid1 = false;
  _isValid2 = false;
}

int heatpumpFunctions::getCode(byte b)
{
  return ((b >> 2) & 0xff) + 100;
}

int heatpumpFunctions::getValue(byte b)
{
  return b & 3;
}

int heatpumpFunctions::getValue(int code)
{
  if (code > 128 || code < 101)
    return 0;

  for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i)
  {
    if (getCode(raw[i]) == code)
      return getValue(raw[i]);
  }

  return 0;
}

bool heatpumpFunctions::setValue(int code, int value)
{
  if (code > 128 || code < 101)
    return false;

  if (value < 1 || value > 3)
    return false;

  for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i)
  {
    if (getCode(raw[i]) == code)
    {
      raw[i] = ((code - 100) << 2) + value;
      return true;
    }
  }

  return false;
}

heatpumpFunctionCodes heatpumpFunctions::getAllCodes()
{
  heatpumpFunctionCodes result;
  for (int i = 0; i < MAX_FUNCTION_CODE_COUNT; ++i)
  {
    int code = getCode(raw[i]);
    result.code[i] = code;
    result.valid[i] = (code >= 101 && code <= 128);
  }

  return result;
}

bool heatpumpFunctions::operator==(const heatpumpFunctions &rhs)
{
  return this->isValid() == rhs.isValid() && memcmp(this->raw, rhs.raw, MAX_FUNCTION_CODE_COUNT * sizeof(int)) == 0;
}

bool heatpumpFunctions::operator!=(const heatpumpFunctions &rhs)
{
  return !(*this == rhs);
}
