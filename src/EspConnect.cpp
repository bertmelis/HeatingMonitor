#include "EspConnect.h"

namespace EspConnect {

uint32_t lastStats = 0;
constexpr uint32_t statsInterval = STATS_INTERVAL;
espMqttClient mqttClient(espMqttClientTypes::UseInternalTask::NO);
Blinker blinker(RGB_BUILTIN, LED_COLOUR_ORDER);
Uptime uptime;
bool restart = false;
bool sessionSetup = false;
size_t sessionSetupStep = 0;
uint16_t subscribeId = 0;
bool webUpdateEnabled = false;
SetupHandler userSetupHandler = nullptr;
espMqttClientTypes::OnMessageCallback userMessageHandler = nullptr;

struct MqttEntity {
  const char* id;
  const char* name;
  const char* unitOfMeasurement;
  const char* icon;
};
const MqttEntity systemEntities[3] = {
  {"signal", "Signal strength", "dB", "mdi:wifi"},
  {"memory", "Free memory",     "b",  "mdi:memory"},
  {"uptime", "Uptime",          "s",  "mdi-timer-check-outline"}
};

// forward declares
void sendStats();
void onMqttConnected(bool sessionPresent);
void onMqttDisconnected(espMqttClientTypes::DisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode* returncodes, size_t len);
void onMqttMessage_internal(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total);
void handleUpdate(const uint8_t* payload, size_t length, size_t index, size_t total);
void setupSession();

void enableWebUpdate() {
  webUpdateEnabled = true;
}

void addSetupHandler(SetupHandler setupHandler) {
  userSetupHandler = setupHandler;
}

void addMessageHandler(espMqttClientTypes::OnMessageCallback messageHandler) {
  userMessageHandler = messageHandler;
}

void setup() {
  mqttClient.setServer(BROKER_IP, BROKER_PORT)
            #ifdef MQTT_USER
            .setCredentials(MQTT_USER, MQTT_PASS)
            #endif
            .setClientId(DEVICEID)
            .setWill(HAD_BASETOPIC DEVICEID "/$system/status", 1, true, "offline")
            .onConnect(onMqttConnected)
            .onDisconnect(onMqttDisconnected)
            .onSubscribe(onMqttSubscribe)
            .onMessage(onMqttMessage_internal)
            .setKeepAlive(MQTT_KEEPALIVE)
            .setCleanSession(true);
  WiFi.setHostname(DEVICEID);
  blinker.on(Blinker::red, 250);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    blinker.loop();
    delay(1);
  }
  blinker.on(Blinker::blue, 250);
  if (webUpdateEnabled) {
    initWebUpdate();
  }
  mqttClient.connect();
  while (!mqttClient.connected()) {
    mqttClient.loop();
    blinker.loop();
    delay(1);
  }
}

void loop(uint32_t currentMillis) {
  blinker.loop();
  mqttClient.loop();
  if (webUpdateEnabled) {
    loopWebupdate();
  }
  if (WiFi.status() != WL_CONNECTED) {
    blinker.on(Blinker::red, 250);
    if (!mqttClient.disconnected()) {
      // WiFi disconnected but MQTT still connected
      mqttClient.disconnect(true);
    } else {
      // WiFi and MQTT disconnected, wait for WiFi reconnect
    }
  } else if (mqttClient.disconnected()) {
    // WiFi connected, MQTT disconnected
    blinker.on(Blinker::orange, 250);
    mqttClient.connect();
  } else if (mqttClient.connected()) {
    // WiFi connected, MQTT connected
    blinker.off();
    if (!sessionSetup) setupSession();
    if (currentMillis - lastStats > statsInterval && isConnected()) {
      lastStats = currentMillis;
      sendStats();
    }
  } else {
    // WiFi connected, MQTT transitional state
    delay(1);
  }
}

bool isConnected() {
  return (mqttClient.connected() && !Update.isRunning() && sessionSetup);
}

void sendStats() {
  char payload[18] = {'\0'};  // uptime: 18446744073709551615 / 1000, signal: 100, heap: 250000
  snprintf(payload, sizeof(payload), "%" PRIu64, uptime.getUptime() / 1000);
  mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/uptime", 0, false, payload);
  mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/uptimereadable", 0, false, uptime.getUptimeStr());
  snprintf(payload, sizeof(payload), "%i", WiFi.RSSI());
  mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/signal", 0, false, payload);
  snprintf(payload, sizeof(payload), "%lu", ESP.getFreeHeap());
  mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/memory", 0, false, payload);
}

void onMqttConnected(bool sessionPresent) {
  mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/status", 1, true, "online");
  sendStats();
}

void onMqttDisconnected(espMqttClientTypes::DisconnectReason reason) {
  (void) reason;
  sessionSetup = false;  // we connected with clean session
  if (restart) {
    ESP.restart();
  }
}

void onMqttSubscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode* returncodes, size_t len) {
  (void) len;
  if (packetId == subscribeId && returncodes[0] != espMqttClientTypes::SubscribeReturncode::FAIL) {
    ++sessionSetupStep;
  } else {
    abort();  // this is not expected to happen
  }
}

void onMqttMessage_internal(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
  (void) properties;
  if (strcmp(topic, HAD_BASETOPIC DEVICEID "/$system/update") == 0) {
    handleUpdate(payload, len, index, total);
    return;
  }
  if (userMessageHandler) userMessageHandler(properties, topic, payload, len, index, total);
}

void handleUpdate(const uint8_t* payload, size_t length, size_t index, size_t total) {
  // The Updater class takes a non-const pointer to write data although it doesn't change the data
  uint8_t* data = const_cast<uint8_t*>(payload);
  static size_t written;
  if (index == 0) {
    if (Update.isRunning()) {
      Update.end();
      Update.clearError();
    }
    written = 0;
    Update.begin(total);
    written += Update.write(data, length);
  } else {
    if (!Update.isRunning()) return;
    written += Update.write(data, length);
  }
  if (Update.isFinished()) {
    if (Update.end()) {
      restart = true;
      mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/status", 1, true, "reboot");
      mqttClient.disconnect();
    } else {
      // Update.printError(Serial);
      Update.clearError();
    }
  }
}

void setupSession() {
  // don't proceed if we're shot on memory
  if (ESP.getMaxAllocHeap() < 16384UL) {
    return;
  }

  switch (sessionSetupStep) {
  case 0:
    // subscribe to update channel
    subscribeId = mqttClient.subscribe(HAD_BASETOPIC DEVICEID "/$system/update", 0);
    if (subscribeId > 0) {
      ++sessionSetupStep;
    }
    return;
  case 1:
    // wait for suback
    return;
  case 2:
    [[fallthrough]];
  case 3:
    [[fallthrough]];
  case 4: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.setSystem();
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = systemEntities[sessionSetupStep - 2].unitOfMeasurement;
    device.json()[HADISCOVERY_ICON] = systemEntities[sessionSetupStep - 2].icon;
    if (!device.create(systemEntities[sessionSetupStep - 2].id, systemEntities[sessionSetupStep - 2].name) ||
      !mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return;
    }
    ++sessionSetupStep;
    [[fallthrough]];
  }
  default:
    if (!userSetupHandler || (userSetupHandler && userSetupHandler())) {
      sessionSetup = true;
      sessionSetupStep = 0;
    }
    return;
  }
}

}  // end namespace EspConnect

