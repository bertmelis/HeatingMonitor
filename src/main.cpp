#include <Arduino.h>

#include <VitoWiFi.h>
#include <USBHostSerial.h>
#include "EspConnect.h"

VitoWiFi::Datapoint datapoints[] = {
  VitoWiFi::Datapoint("boilertemp", 0x0810, 2, VitoWiFi::div10),
  VitoWiFi::Datapoint("burnerhours", 0x08A7, 4, VitoWiFi::div3600),
  VitoWiFi::Datapoint("burnerstarts", 0x088A, 4, VitoWiFi::noconv),
  VitoWiFi::Datapoint("boilerpump", 0x7660, 1, VitoWiFi::noconv),
  VitoWiFi::Datapoint("watertemp", 0x0812, 2, VitoWiFi::div10),
  VitoWiFi::Datapoint("solarcollectortemp", 0x6564, 2, VitoWiFi::div10),
  VitoWiFi::Datapoint("solarcylindertemp", 0x6566, 2, VitoWiFi::div10),
  VitoWiFi::Datapoint("solarpump", 0x6552, 1, VitoWiFi::noconv)
};
uint8_t numberOfDatapoints = 8;

uint8_t setupStep = 0;
bool setupSession() {
  switch (setupStep) {
  case 0: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "°c";
    device.json()[HADISCOVERY_ICON] = "mdi:water-boiler";
    if (!device.create("boilertemp", "Boiler Temperature") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 1: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "h";
    device.json()[HADISCOVERY_ICON] = "mdi:timer-check-outline";
    device.json()[HADISCOVERY_STATE_CLASS] = "total_increasing";
    if (!device.create("burnerhours", "Burner hours run") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 2: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    // device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "";
    device.json()[HADISCOVERY_ICON] = "mdi:counter";
    device.json()[HADISCOVERY_STATE_CLASS] = "total_increasing";
    if (!device.create("burnerstarts", "Burner starts") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 3: {
    HassDiscovery::BinarySensor device(DEVICEID, DEVICENAME);
    // device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "";
    device.json()[HADISCOVERY_ICON] = "mdi:counter";
    if (!device.create("boilerpump", "Boiler internal pump") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 4: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "°c";
    device.json()[HADISCOVERY_ICON] = "mdi:water-thermometer";
    if (!device.create("watertemp", "DHW Cylinder temp") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 5: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "°c";
    device.json()[HADISCOVERY_ICON] = "mdi:solar-power-variant";
    if (!device.create("solarcollectortemp", "Solar Collector Temperature") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 6: {
    HassDiscovery::Sensor device(DEVICEID, DEVICENAME);
    device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "°c";
    device.json()[HADISCOVERY_ICON] = "mdi:water-boiler";
    if (!device.create("solarcylindertemp", "Solar Cylinder Temperature") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  case 7: {
    HassDiscovery::BinarySensor device(DEVICEID, DEVICENAME);
    // device.json()[HADISCOVERY_UNIT_OF_MEASUREMENT] = "";
    device.json()[HADISCOVERY_ICON] = "mdi:pump";
    if (!device.create("solarpump", "Solar Pump") ||
      !EspConnect::mqttClient.publish(device.topic(), 1, true, device.payload())) {
      return false;
    }
    ++setupStep;
    [[fallthrough]];
  }
  default:
    setupStep = 0;
    return true;
  }
}

static void logUSB(const char* msg) {
  if (EspConnect::isConnected()) {
    EspConnect::mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/log/usb", 0, false, msg);
  }
}

class USBInterface {
 public:
  bool begin() {
    _usbSerial.setLogger(logUSB);
    _usbSerial.flush();
    return _usbSerial.begin(4800, 2, 2, 8);
  }
  void end() {
    // not yet implemented!
    _usbSerial.end();
  }
  std::size_t write(const uint8_t* data, uint8_t length) {
    return _usbSerial.write(data, length);
  }
  uint8_t read() {
    return _usbSerial.read();
  }
  size_t available() {
    return _usbSerial.available();
  }

 private:
 USBHostSerial _usbSerial;
} usbInterface;

VitoWiFi::VitoWiFi<VitoWiFi::VS2> vitoWiFi(&usbInterface);
bool readValues = false;
uint8_t datapointIndex = 0;

void onResponse(const VitoWiFi::PacketVS2& response, const VitoWiFi::Datapoint& request) {
  char topic[100];
  char payload[18] = {'\0'};

  if (response.packetType() == VitoWiFi::PacketType::ERROR) {
    char buff[100];
    snprintf(buff, 100, "error resoonse for %s", request.name());
    EspConnect::mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/log", 0, false, buff);
  }

  snprintf(topic, sizeof(topic), HAD_BASETOPIC DEVICEID "/%s", request.name());
  if (request.converter() == VitoWiFi::div10 || request.converter() == VitoWiFi::div3600) {
    float value = request.decode(response);
    snprintf(payload, sizeof(payload), "%.1f", value);
  } else if (request.converter() == VitoWiFi::noconv) {
    if (strcmp(request.name(), "burnerstarts") == 0) {
      uint32_t value = request.decode(response);
      snprintf(payload, sizeof(payload), "%lu", value);
    } else {
      bool value = request.decode(response);
      snprintf(payload, sizeof(payload), "%s", value ? "1" : "0");
    }
  }
  EspConnect::mqttClient.publish(topic, 0, false, payload);
}

void onError(VitoWiFi::OptolinkResult error, const VitoWiFi::Datapoint& request) {
  char buff[100];
  snprintf(buff, 100, "response from %s NOK: %i", request.name(), static_cast<int>(error));
  EspConnect::mqttClient.publish(HAD_BASETOPIC DEVICEID "/$system/log", 0, false, buff);
}

void setup() {
  delay(2000);

  EspConnect::enableWebUpdate();
  EspConnect::addSetupHandler(setupSession);
  EspConnect::setup(); // blocking function, connect to WiFi and MQTT
  delay(5000);

  EspConnect::blinker.on({0, 0, 100});
  vitoWiFi.onResponse(onResponse);
  vitoWiFi.onError(onError);
  vitoWiFi.begin();
  delay(500);
  EspConnect::blinker.off();
}

void loop() {
  constexpr uint32_t valuesInterval = 60000UL;
  static uint32_t currentMillis = 0;
  static uint32_t lastValues = 0;
  currentMillis = millis();

  if (currentMillis - lastValues > valuesInterval) {
      lastValues = currentMillis;
      readValues = true;
      datapointIndex = 0;
    }

  if (EspConnect::isConnected()) {
    if (readValues) {
      if (vitoWiFi.read(datapoints[datapointIndex])) {
        ++datapointIndex;
      }
      if (datapointIndex == numberOfDatapoints) {
        readValues = false;
      }
    }
  }

  vitoWiFi.loop();
  EspConnect::loop(currentMillis);
}
