/*
   -------------------------------------------------------------------
   EmonESP Serial to Emoncms gateway
   -------------------------------------------------------------------
   Adaptation of Chris Howells OpenEVSE ESP Wifi
   by Trystan Lea, Glyn Hudson, OpenEnergyMonitor

   Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina
   Modified to use with EMS Workshop by dmendonca

   All adaptation GNU General Public License as below.

   -------------------------------------------------------------------

   This file is part of OpenEnergyMonitor.org project.
   EmonESP is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.
   EmonESP is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EmonESP; see the file COPYING.  If not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
*/

#include "mqtt_client.h"
#include <TimeLib.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <data_model.h>
#include <config.h>
#include <ArduinoJson.h>
#include <modbus_master.h>
#include <modbus_evse.h>
#include <sunspec_model_213.h>
#include "data_model.h"

WiFiClient transportClient;                 // the network client for MQTT (also works with EthernetLarge)
PubSubClient mqttclient(transportClient);   // the MQTT client

unsigned long mqtt_interval_ts = 0;
static char mqtt_data[128] = "";
static int mqtt_connection_error_count = 0;
String topic_device;          // StreetPoleEMS globally unique device topic (publish/subscribe under here)
String topic_cmd;             // command topic (for 'southbound' commands)

// Function prototype for mqtt_publish_json
void mqtt_publish_json(const char* subtopic, const JsonDocument* payload);

// --- EVSE Command Queue (deferred execution from MQTT callback) ---
struct PendingCmd {
  char     cmd[16];
  uint16_t reg;
  uint16_t value;
};

#define CMD_QUEUE_SIZE 4
static PendingCmd cmd_queue[CMD_QUEUE_SIZE];
static uint8_t cmd_queue_head = 0;
static uint8_t cmd_queue_tail = 0;

static bool enqueue_cmd(const char* cmd, uint16_t reg = 0, uint16_t value = 0) {
  uint8_t next_head = (cmd_queue_head + 1) % CMD_QUEUE_SIZE;
  if (next_head == cmd_queue_tail) {
    Serial.println("MQTT CMD: queue full, dropping command");
    return false;
  }
  strncpy(cmd_queue[cmd_queue_head].cmd, cmd, sizeof(cmd_queue[cmd_queue_head].cmd) - 1);
  cmd_queue[cmd_queue_head].cmd[sizeof(cmd_queue[cmd_queue_head].cmd) - 1] = '\0';
  cmd_queue[cmd_queue_head].reg = reg;
  cmd_queue[cmd_queue_head].value = value;
  cmd_queue_head = next_head;
  return true;
}

void generateTopics() {
  //the top-level device topic string, eg: OPENAMI_<streetpoleEMSid>
  topic_device = MQTT_TOPIC;
  topic_device.concat("/");
  topic_device.concat(getDeviceID());
  topic_device.concat("/");

  //the command topic we subscribe to, eg: OPENAMI_ECAE3D98/cmd
  
  topic_cmd = topic_device;
  topic_cmd.concat("cmd");
}

// -------------------------------------------------------------------
// MQTT Connect
// Called only when MQTT server field is populated
// -------------------------------------------------------------------
boolean mqtt_connect()
{
  Serial.printf("MQTT Connecting...timeout in:%d\r\n", transportClient.getTimeout());

  if (transportClient.connect(MQTT_SERVER, 1883) != 1) //8883 for TLS
  {
     Serial.println("MQTT connect timeout.");
     return (0);
  }

  //transportClient.setTimeout(60);//(MQTT_TIMEOUT);
  mqttclient.setSocketTimeout(6);//MQTT_TIMEOUT);
  mqttclient.setBufferSize(MAX_DATA_LEN + 200);
  mqttclient.setKeepAlive(180);

  if (strcmp(MQTT_USER, "") == 0) {
    //allows for anonymous connection
    mqttclient.connect(getDeviceID()); // Attempt to connect
  } else {
    mqttclient.connect(getDeviceID(), MQTT_USER, MQTT_PW); // Attempt to connect
  }

  if (mqttclient.state() == 0) {
    Serial.printf("MQTT connected: %s\r\n", MQTT_SERVER);
    
    //subscribe to command topic
    if (!mqttclient.subscribe(topic_cmd.c_str())) {
      delay(250);
      if (!mqttclient.subscribe(topic_cmd.c_str())) {
        delay(500);
        if (!mqttclient.subscribe(topic_cmd.c_str())) {
          Serial.printf("MQTT: FAILED TO SUBSCRIBE TO COMMAND TOPIC: %s\r\n", topic_cmd.c_str());
          return false;
        }
      }
    }
    Serial.printf("MQTT: SUBSCRIBED TO COMMAND TOPIC: %s\r\n", topic_cmd.c_str());
    //  mqttclient.publish(getDeviceTopic().c_str(), "connected"); // Once connected, publish an announcement..
  } else {
    Serial.println("MQTT failed: ");
    Serial.println(mqttclient.state());
    return (0);
  }
  return (1);
}

void mqtt_publish_evse_data() {
  StaticJsonDocument<2048> jsonDoc;

  jsonDoc["timestamp"] = millis();
  jsonDoc["poll_success"] = evse_poll_ok;

  // Core status
  jsonDoc["current_status"] = evse.getCurrentStatus();
  jsonDoc["status_string"] = evse.getStatusString();
  jsonDoc["is_charging"] = evse.isCharging();
  jsonDoc["is_connected"] = evse.isConnected();
  jsonDoc["remote_start_stop"] = evse.getRemoteStartStop();
  jsonDoc["software_version"] = evse.getSoftwareVersion();

  // Charging data (regs 147-149)
  jsonDoc["charging_current"] = evse.getChargingCurrent();
  jsonDoc["charging_voltage"] = evse.getChargingVoltage();
  jsonDoc["charging_power"] = evse.getChargingPower();

  // CP signal (regs 153-154)
  jsonDoc["cp_positive_voltage"] = evse.getCPPositiveVoltage();
  jsonDoc["cp_negative_voltage"] = evse.getCPNegativeVoltage();

  // PWM (regs 109, 142, 151, 152)
  jsonDoc["max_output_pwm_duty"] = evse.getMaxOutputPWMDuty();
  jsonDoc["cable_gauge_pwm"] = evse.getCableGaugePWM();
  jsonDoc["rotary_switch_pwm"] = evse.getRotarySwitchPWM();
  jsonDoc["current_output_pwm"] = evse.getCurrentOutputPWM();

  // Protection & safety (regs 143-145, 155, 157)
  jsonDoc["rcmu_status"] = evse.getRCMUStatus();
  jsonDoc["rfid_status"] = evse.getRFIDStatus();
  jsonDoc["lock_status"] = evse.getLockStatus();
  jsonDoc["overcurrent_count"] = evse.getOvercurrentCount();
  jsonDoc["current_temperature"] = evse.getCurrentTemperature();

  // Meter data (regs 159, 162-165)
  jsonDoc["meter_a_voltage"] = evse.getMeterAVoltage();
  jsonDoc["meter_current"] = evse.getMeterCurrent();
  jsonDoc["meter_total_power"] = evse.getMeterTotalPower();
  jsonDoc["meter_total_kwh"] = evse.getMeterTotalKWH();

  // Dial settings (regs 128-133)
  jsonDoc["dial_setting_1"] = evse.getDialSetting(1);
  jsonDoc["dial_setting_2"] = evse.getDialSetting(2);
  jsonDoc["dial_setting_3"] = evse.getDialSetting(3);
  jsonDoc["dial_setting_4"] = evse.getDialSetting(4);
  jsonDoc["dial_setting_5"] = evse.getDialSetting(5);
  jsonDoc["dial_setting_6"] = evse.getDialSetting(6);

  mqtt_publish_json("evse", &jsonDoc);
}



void mqtt_publish_json(const char* subtopic, const JsonDocument * payload) {
    String jsonString;
    if (measureJson(*payload) >= 2048) {
      Serial.println("MQTT publish: payload too large");
      return;
    }
    serializeJson(*payload, jsonString);
    String topicBuf = topic_device;
    topicBuf.concat(subtopic);
    if (!mqttclient.publish(topicBuf.c_str(), jsonString.c_str())) {
        Serial.println("MQTT publish: failed");
    }
#ifdef ENABLE_DEBUG_MQTT
    Serial.printf("topic: %s, data: %s\n", topicBuf.c_str(), jsonString.c_str());
#endif
}

//pull apart a comma-sep colon-delim name:value string and publish the name:value pairs under 'subtopic'
void mqtt_publish_comma_sep_colon_delim(const char* subtopic, const char * data) {
    String topicBuf;
    char buf[256];
    Serial.printf("MQTT publish: size:%d chars", strlen(data));
    do {
      int pos = strcspn(data, ":");
      strncpy(buf, data, pos);
      buf[pos] = 0;
      String st(subtopic);
      topicBuf = topic_device;
      topicBuf.concat(st+"/");
      topicBuf.concat(buf);
      //topic_ptr[pos] = 0;
      data += pos;
      if (*data++ == 0) {
        break;
      }

      pos = strcspn(data, ",");
      strncpy(mqtt_data, data, pos);
      mqtt_data[pos] = 0;
      data += pos;

      if (!mqttclient.publish(topicBuf.c_str(), mqtt_data)) {
       Serial.println("MQTT publish: failed");
      }
#ifdef ENABLE_DEBUG_MQTT
      Serial.printf("topic: %s, data: %s\n", topicBuf.c_str(), mqtt_data);
#endif
    } while (*data++ != 0);
}

// Subscriber callback -- parses JSON commands and enqueues for deferred execution.
// Commands are executed in process_pending_cmds() called from the main loop.
void subscriber_callback(char* topic, uint8_t* payload, unsigned int length) {
  if (length > 254) {
    Serial.printf("MQTT CALLBACK: payload too large: %d\n", length);
    return;
  }
  if (strcmp(topic, topic_cmd.c_str()) != 0) {
    return;
  }

  char payload_buf[256] = {0};
  strncpy(payload_buf, (char*)payload, length);
  payload_buf[length] = '\0';
  Serial.printf("\nMQTT CMD: topic '%s', payload '%s'\n", topic, payload_buf);

  StaticJsonDocument<256> cmdDoc;
  DeserializationError err = deserializeJson(cmdDoc, payload_buf);
  if (err) {
    Serial.printf("MQTT CMD: JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* cmd = cmdDoc["cmd"] | (const char*)nullptr;
  if (cmd == nullptr) {
    Serial.println("MQTT CMD: missing 'cmd' field");
    return;
  }

  uint16_t reg   = cmdDoc["reg"]   | (uint16_t)0;
  uint16_t value = cmdDoc["value"] | (uint16_t)0;

  if (strcmp(cmd, "start") == 0 ||
      strcmp(cmd, "stop") == 0 ||
      strcmp(cmd, "enable") == 0 ||
      strcmp(cmd, "disable") == 0 ||
      strcmp(cmd, "set_pwm") == 0 ||
      strcmp(cmd, "write_register") == 0) {
    if (!enqueue_cmd(cmd, reg, value)) {
      Serial.printf("MQTT CMD: failed to enqueue '%s'\n", cmd);
    }
  } else {
    Serial.printf("MQTT CMD: unknown command '%s'\n", cmd);
  }
}

// Process queued commands -- called from main loop, safe to do Modbus I/O here.
void process_pending_cmds() {
  while (cmd_queue_tail != cmd_queue_head) {
    PendingCmd& pc = cmd_queue[cmd_queue_tail];
    const char* cmd = pc.cmd;
    uint8_t result = 0xFF;

    if (strcmp(cmd, "start") == 0 || strcmp(cmd, "enable") == 0) {
      result = evse.startCharging();
    } else if (strcmp(cmd, "stop") == 0) {
      result = evse.stopCharging();
    } else if (strcmp(cmd, "disable") == 0) {
      result = evse.stopCharging();
      evse.setMaxOutputPWMDuty(0);
    } else if (strcmp(cmd, "set_pwm") == 0) {
      result = evse.setMaxOutputPWMDuty(pc.value);
    } else if (strcmp(cmd, "write_register") == 0) {
      result = evse.write_register(pc.reg, pc.value);
    }

    // Publish ack
    StaticJsonDocument<256> ackDoc;
    ackDoc["cmd"] = cmd;
    ackDoc["result"] = result;
    ackDoc["success"] = (result == 0x00);
    if (pc.reg != 0) ackDoc["reg"] = pc.reg;
    if (pc.value != 0) ackDoc["value"] = pc.value;
    ackDoc["timestamp"] = millis();
    mqtt_publish_json("cmd_ack", &ackDoc);

    Serial.printf("EVSE CMD: '%s' result=0x%02X %s\n",
                  cmd, result, result == 0 ? "OK" : "FAIL");

    cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_SIZE;
  }
}

// Fast MQTT loop -- call every main loop iteration for low-latency command processing
void loop_mqtt_fast() {
  mqttclient.loop();
  if (mqttclient.connected()) {
    process_pending_cmds();
  }
}

void setup_mqtt_client() {
  generateDeviceID(); // Generate device ID first
  generateTopics();
  mqttclient.setCallback(subscriber_callback);
  if (!mqtt_connect()) {
    delay(250);
    if (!mqtt_connect()) {
      delay(500);
      if (!mqtt_connect()) {
        Serial.println("MQTT: FAILED TO CONNECT");
        return;
      }
    }
  }
  mqtt_interval_ts = now();
}

void loop_mqtt() {
      // Reconnect if needed
      if (!mqttclient.connected()) {
        mqtt_connect();
      }

      if (mqttclient.connected()) {
        mqtt_publish_evse_data();
      }
      mqtt_interval_ts = millis();
}

void mqtt_restart()
{
  if (mqttclient.connected()) {
    mqttclient.disconnect();
  }
}

boolean mqtt_connected()
{
  return mqttclient.connected();
}

void mqtt_publish_door_opened() {
  char buf[32] = {0};
  sprintf(buf,"%s/door", topic_device);
  mqttclient.publish(buf, "open", 0);
}

void mqtt_publish_door_closed() {
  char buf[32] = {0};
  sprintf(buf,"%s/door", topic_device);
  mqttclient.publish(buf, "closed", 0);
}
