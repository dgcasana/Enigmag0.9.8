/**
  * @file EnigmaIOT-SmartSwitch-Controller.ino
  * @version 0.9.8
  * @date 15/07/2021
  * @author German Martin
  * @brief Node based on EnigmaIoT over ESP-NOW, in non sleeping mode
  *
  * Sensor reading code is mocked on this example. You can implement any other code you need for your specific need
  * 
  *   Actualizado a la version enigmaiot 0.9.8.
  * 	HomeAsistant Version
  *   * 
  */

#if !defined ESP8266 && !defined ESP32
#error Node only supports ESP8266 or ESP32 platform
#endif

#include <Arduino.h>
#include <EnigmaIOTNode.h>
#include <EnigmaIOTjsonController.h>
#include <FailSafe.h>
#include "SmartSwitchCaldera.h" // <-- Include here your controller class header

#include <EnigmaIOTNode.h>
#include <espnow_hal.h>
#include <CayenneLPP.h>
#include <ArduinoJson.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ESPAsyncTCP.h> // Comment to compile for ESP32
#include <Hash.h>
#elif defined ESP32
#include <WiFi.h>
#include <AsyncTCP.h> // Comment to compile for ESP8266
#include <Update.h>
#include <driver/adc.h>
#include "esp_wifi.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#endif
#include <ArduinoJson.h>
#include <Curve25519.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include <FS.h>

#define SLEEPY 0 // Set it to 1 if your node should sleep after sending data
#if SUPPORT_HA_DISCOVERY    
#include <haTrigger.h>
#include <haSwitch.h>
#endif

EnigmaIOTjsonController* controller; // Generic controller is refferenced here. You do not need to modify it

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // ESP32 boards normally have a LED in GPIO3 or GPIO5
#endif // !LED_BUILTIN

bool rBypass = true;


const time_t BOOT_FLAG_TIMEOUT = 10000; // Time in ms to reset flag
const int MAX_CONSECUTIVE_BOOT = 3; // Number of rapid boot cycles before enabling fail safe mode
const int LED = LED_BUILTIN; // Number of rapid boot cycles before enabling fail safe mode
const int FAILSAFE_RTC_ADDRESS = 0; // If you use RTC memory adjust offset to not overwrite other data

// Called when node is connected to gateway. You don't need to do anything here usually


//constexpr auto RESET_PIN = -1;

#define BLUE_LED LED_BUILTIN // You can set a different LED pin here. -1 means disabled

#define domoticz_idx 76
bool releRx,datoRx=false;

// Pines del Wemos
//#define RELE_PIN D1 // como siempre






void connectEventHandler () {
	controller->connectInform ();
	Serial.println ("Connected");
}

void disconnectEventHandler (nodeInvalidateReason_t reason) {
	Serial.printf ("Unregistered. Reason: %d\n", reason);
}

// Called to route messages to EnitmaIOTNode class. Do not modify
bool sendUplinkData (const uint8_t* data, size_t len, nodePayloadEncoding_t payloadEncoding, dataMessageType_t dataMsgType) {
    if (dataMsgType == DATA_TYPE) {
        return EnigmaIOTNode.sendData (data, len, payloadEncoding);
    } else if (dataMsgType == HA_DISC_TYPE) {
        return EnigmaIOTNode.sendHADiscoveryMessage (data, len);
    } else {
        return false;
    }
}

void sendMsgPack (DynamicJsonDocument json) {
  int len = measureMsgPack (json) + 1;
    uint8_t* buffer = (uint8_t*)malloc (len);
    len = serializeMsgPack (json, (char *)buffer, len);
    Serial.printf ("Trying to send: %s\n", printHexBuffer (buffer, len));
    // Send buffer data
    if (!EnigmaIOTNode.sendData (buffer, len, MSG_PACK)) {
      Serial.println ("---- Error sending data");
    } else {
      Serial.println ("---- Data sent");
    }
    free (buffer);
	//json.clear();
}

void processRxData (const uint8_t* mac, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	char macstr[ENIGMAIOT_ADDR_LEN * 3];
	String commandStr;
	uint8_t tempBuffer[MAX_MESSAGE_LENGTH];
	bool broadcast = false;
	uint8_t _command = command;
	const size_t capacity = JSON_OBJECT_SIZE (5);
	DynamicJsonDocument json (capacity);

	if (controller->processRxCommand (mac, buffer, length, command, payloadEncoding)) {
		DEBUG_INFO ("Command processed");
	} else {
		if (_command & 0x80)
			broadcast = true;

		_command = (_command & 0x7F);

		mac2str (mac, macstr);
		Serial.println ();
		Serial.printf ("Data from %s --> %s\n", macstr, printHexBuffer (buffer, length));
		if (_command == nodeMessageType_t::DOWNSTREAM_DATA_GET)
			commandStr = "GET";
		else if (_command == nodeMessageType_t::DOWNSTREAM_DATA_SET)
			commandStr = "SET";
		else
			return;

		Serial.printf ("%s Command %s\n", broadcast ? "Broadcast" : "Unicast", commandStr.c_str ());
		Serial.printf ("Data: %s\n", printHexBuffer (buffer, length));
		Serial.printf ("Encoding: 0x%02X\n", payloadEncoding);

		CayenneLPP lpp (MAX_DATA_PAYLOAD_SIZE);
		DynamicJsonDocument doc (1000);
		JsonArray root;
		memcpy (tempBuffer, buffer, length);

		switch (payloadEncoding) {
		case CAYENNELPP:
			root = doc.createNestedArray ();
			lpp.decode (tempBuffer, length, root);
			serializeJsonPretty (doc, Serial);
			break;
		case MSG_PACK:
			deserializeMsgPack (doc, tempBuffer, length);
			serializeJsonPretty (doc, Serial);
			if(doc ["idx"] == domoticz_idx){
				rBypass = doc["nvalue"]; // 1 actua en funcion del termostato, 0 nunca activa el rele
				json["bypass"] = rBypass;
				sendMsgPack(json);	
				
			}


			Serial.println ();
			break;
		default:
			DEBUG_WARN ("Payload encoding %d is not (yet) supported", payloadEncoding);
		}
	}
}

// Do not modify
void wifiManagerExit (boolean status) {
	controller->configManagerExit (status);
}

// Do not modify
void wifiManagerStarted () {
	controller->configManagerStart ();
}


void setup () {
	
	
  	Serial.begin (115200); Serial.println (); Serial.println ("enigmaiot_Caldera-v2.1.1 --  Enigmav0.9.8");
	Serial.println ("----Reset Pin D5----");

#ifdef ESP32
	// Turn-off the 'brownout detector' to avoid random restarts during wake up,
	// normally due to bad quality regulator on board
	WRITE_PERI_REG (RTC_CNTL_BROWN_OUT_REG, 0);
#endif
    FailSafe.checkBoot (MAX_CONSECUTIVE_BOOT, LED, FAILSAFE_RTC_ADDRESS); // Parameters are optional
    if (FailSafe.isActive ()) { // Skip all user setup if fail safe mode is activated
        return;
    }

	controller = (EnigmaIOTjsonController*)new CONTROLLER_CLASS_NAME (); // Use your class name here

	EnigmaIOTNode.setLed (BLUE_LED);
	EnigmaIOTNode.setResetPin (RESET_PIN);
	EnigmaIOTNode.onConnected (connectEventHandler);
	EnigmaIOTNode.onDisconnected (disconnectEventHandler);
	EnigmaIOTNode.onDataRx (processRxData);
	EnigmaIOTNode.enableClockSync (true); // Set to true if you need this node to get its clock syncronized with gateway
	//EnigmaIOTNode.onWiFiManagerStarted (wifiManagerStarted);
	EnigmaIOTNode.onWiFiManagerExit (wifiManagerExit);
	EnigmaIOTNode.enableBroadcast ();

	if (!controller->loadConfig ()) { // Trigger custom configuration loading
		DEBUG_WARN ("Error reading config file");
		if (FILESYSTEM.format ())
            DEBUG_WARN ("FILESYSTEM Formatted");
	}

	EnigmaIOTNode.begin (&Espnow_hal, NULL, NULL, true, SLEEPY == 1); // Start EnigmaIOT communication
    
	uint8_t macAddress[ENIGMAIOT_ADDR_LEN];
#ifdef ESP8266
	if (wifi_get_macaddr (STATION_IF, macAddress))
#elif defined ESP32
	if ((esp_wifi_get_mac (WIFI_IF_STA, macAddress) == ESP_OK))
#endif
	{
		EnigmaIOTNode.setNodeAddress (macAddress);
		DEBUG_WARN ("Node address set to %s", mac2str (macAddress));
	} else {
		DEBUG_WARN ("Node address error");
	}
	

	controller->sendDataCallback (sendUplinkData); // Listen for data from controller class
	controller->setup (&EnigmaIOTNode);			   // Start controller class

}







void loop () {
	FailSafe.loop (BOOT_FLAG_TIMEOUT); // Use always this line

    if (FailSafe.isActive ()) { // Skip all user loop code if Fail Safe mode is active
        return;
    }

    controller->loop (); // Loop controller class
#if SUPPORT_HA_DISCOVERY 
    controller->callHAdiscoveryCalls (); // Send HA registration messages
#endif // SUPPORT_HA_DISCOVERY 
    EnigmaIOTNode.handle (); // Mantain EnigmaIOT connection

	CayenneLPP msg (20);

	//DynamicJsonDocument json (capacity);

	
	
}
