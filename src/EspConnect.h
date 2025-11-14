#pragma once

#include <WiFi.h>
#include <Update.h>

#include <espMqttClient.h>
#include <HassDiscovery.h>

#include "Config.h"
#include "Helpers/WebUpdate.h"
#include "Helpers/Blinker.h"
#include "Helpers/Uptime.h"

namespace EspConnect {

// MQTT client
extern espMqttClient mqttClient;

// LED blinker
extern Blinker blinker;

// enable a webserver on port 80 for firmware updates
void enableWebUpdate();

typedef std::function<bool()> SetupHandler;
void addSetupHandler(SetupHandler setupHandler);
void addMessageHandler(espMqttClientTypes::OnMessageCallback messageHandler);

// call setup once. this method blocks until WiFi and MQTT are connected
void setup();

// call loop frequent to keep things running
void loop(uint32_t currentMillis);

// method to check if device is connected to WiFi *and* MQTT. Also returns false when the device is being updated.
bool isConnected();

}  // end namespace EspConnect
