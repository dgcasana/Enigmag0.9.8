/**
  * @file enigmaiot_SueloRad.ino
  * @version 0.9.8
  * @date 18/11/2021
  * @author German Martin && dgcasana
  * @brief Node based on EnigmaIoT over ESP-NOW
  *
  * Sensor reading code is mocked on this example. You can implement any other code you need for your specific need
  */

#include <Arduino.h>
#include <EnigmaIOTNode.h>
#include <espnow_hal.h>
#include <CayenneLPP.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
/*#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ESPAsyncTCP.h> // Comment to compile for ESP32
#include <Hash.h>*/
#elif defined ESP32
#include <WiFi.h>
#include <AsyncTCP.h> // Comment to compile for ESP8266
#include <SPIFFS.h>
#include <Update.h>
#include <driver/adc.h>
#include "esp_wifi.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#endif
#include <ArduinoJson.h>
/*#include <Curve25519.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include <FS.h>*/

#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // ESP32 boards normally have a LED in GPIO2 or GPIO5
#endif // !LED_BUILTIN

#define BLUE_LED LED_BUILTIN
constexpr auto RESET_PIN = -1;

#ifdef ESP8266
//ADC_MODE (ADC_VCC);
const int analogInPin = A0;
#endif

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS D3
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
//DeviceAddress sens1,sens2,sens3,sens4,sens5,sens6,sens7,sens8,sens9,sens10,sens11;

DeviceAddress sens1 = { 0x28,0x5B,0x9C,0x75,0xD0,0x01,0x3C,0xB7 };
DeviceAddress sens2 = { 0x28,0xB4,0x00,0x75,0xD0,0x01,0x3C,0x2C };
DeviceAddress sens3 = { 0x28,0x84,0xF1,0x75,0xD0,0x01,0x3C,0xAA };
DeviceAddress sens4 = { 0x28,0x6E,0x3E,0x75,0xD0,0x01,0x3C,0x01 };
DeviceAddress sens5 = { 0x28,0xB2,0x1B,0x75,0xD0,0x01,0x3C,0x92 };
DeviceAddress sens6 = { 0x28,0x3E,0xDB,0x75,0xD0,0x01,0x3C,0x27 };
DeviceAddress sens7 = { 0x28,0x2F,0x68,0x75,0xD0,0x01,0x3C,0x4A };
DeviceAddress sens8 = { 0x28,0xF7,0x3E,0x75,0xD0,0x01,0x3C,0x26 };
DeviceAddress sens9 = { 0x28,0x56,0xD1,0x75,0xD0,0x01,0x3C,0xF8 };
DeviceAddress sens10 = { 0x28,0x69,0x40,0x75,0xD0,0x01,0x3C,0x4B };
//DeviceAddress sens11 = { 0x28,0xFE,0x7C,0x75,0xD0,0x01,0x3C,0xDA };

void connectEventHandler () {
	Serial.println ("Registered");
}

void disconnectEventHandler (nodeInvalidateReason_t reason) {
	Serial.printf ("Unregistered. Reason: %d\n", reason);
}

void processRxData (const uint8_t* mac, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	char macstr[ENIGMAIOT_ADDR_LEN * 3];
	String commandStr;
	uint8_t tempBuffer[MAX_MESSAGE_LENGTH];

	mac2str (mac, macstr);
	Serial.println ();
	Serial.printf ("Data from %s\n", macstr);
	if (command == nodeMessageType_t::DOWNSTREAM_DATA_GET)
		commandStr = "GET";
	else if (command == nodeMessageType_t::DOWNSTREAM_DATA_SET)
		commandStr = "SET";
	else
		return;

	Serial.printf ("Command %s\n", commandStr.c_str ());
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
		break;
	default:
		DEBUG_WARN ("Payload encoding %d is not (yet) supported", payloadEncoding);
	}
}

void setup () {

	Serial.begin (115200); Serial.println ("EnigmaNode- SueloRadiante 0.9.8"); Serial.println ();
	time_t start = millis ();
 sensors.begin();
 Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
 sensors.setResolution(sens1, TEMPERATURE_PRECISION);
 sensors.setResolution(sens2, TEMPERATURE_PRECISION);
 sensors.setResolution(sens3, TEMPERATURE_PRECISION);
  sensors.setResolution(sens4, TEMPERATURE_PRECISION);
  sensors.setResolution(sens5, TEMPERATURE_PRECISION);
  sensors.setResolution(sens6, TEMPERATURE_PRECISION);
  sensors.setResolution(sens7, TEMPERATURE_PRECISION);
  sensors.setResolution(sens8, TEMPERATURE_PRECISION);
  sensors.setResolution(sens9, TEMPERATURE_PRECISION);
  sensors.setResolution(sens10, TEMPERATURE_PRECISION);
  //sensors.setResolution(sens11, TEMPERATURE_PRECISION);

	EnigmaIOTNode.setLed (BLUE_LED);
	EnigmaIOTNode.setResetPin (RESET_PIN);
	EnigmaIOTNode.onConnected (connectEventHandler);
	EnigmaIOTNode.onDisconnected (disconnectEventHandler);
	EnigmaIOTNode.onDataRx (processRxData);

	EnigmaIOTNode.begin (&Espnow_hal);


	uint8_t macAddress[ENIGMAIOT_ADDR_LEN];
#ifdef ESP8266
	if (wifi_get_macaddr (STATION_IF, macAddress))
#elif defined ESP32
	if ((esp_wifi_get_mac (WIFI_IF_STA, macAddress) == ESP_OK))
#endif
	{
		EnigmaIOTNode.setNodeAddress (macAddress);
		//char macStr[ENIGMAIOT_ADDR_LEN * 3];
		DEBUG_DBG ("Node address set to %s", mac2str (macAddress));
	} else {
		DEBUG_WARN ("Node address error");
	}

	// Put here your code to read sensor and compose buffer
	CayenneLPP msg (MAX_DATA_PAYLOAD_SIZE);
  sensors.requestTemperatures();

  float temp1 = sensors.getTempC(sens1);
  float temp2 = sensors.getTempC(sens2);
  float temp3 = sensors.getTempC(sens3);
  float temp4 = sensors.getTempC(sens4);
  float temp5 = sensors.getTempC(sens5);
  float temp6 = sensors.getTempC(sens6);
  float temp7 = sensors.getTempC(sens7);
  float temp8 = sensors.getTempC(sens8);
  float temp9 = sensors.getTempC(sens9);
  float temp10 = sensors.getTempC(sens10);
  //float temp11 = sensors.getTempC(sens11);


  int sensorValue = analogRead(analogInPin);
  float vccBatt = sensorValue/183.5;
	msg.addAnalogInput (0, vccBatt);

	msg.addTemperature (1, temp1);
	msg.addTemperature (2, temp2);
  msg.addTemperature (3, temp3);
  msg.addTemperature (4, temp4);
  msg.addTemperature (5, temp5);
  msg.addTemperature (6, temp6);
  msg.addTemperature (7, temp7);
  msg.addTemperature (8, temp8);
  msg.addTemperature (9, temp9);
  msg.addTemperature (10, temp10);
  //msg.addTemperature (11, temp11);



	Serial.printf ("Vcc: %i\n", sensorValue);
 Serial.printf ("Sensor 9: %f\n", temp9);
	// End of user code

	Serial.printf ("Trying to send: %s\n", printHexBuffer (msg.getBuffer (), msg.getSize ()));

	// Send buffer data
	if (!EnigmaIOTNode.sendData (msg.getBuffer (), msg.getSize ())) {
		Serial.println ("---- Error sending data");
	} else {
		Serial.println ("---- Data sent");
	}
	Serial.printf ("Total time: %lu ms\n", millis () - start);

	// Go to sleep
	EnigmaIOTNode.sleep ();
}

void loop () {

	EnigmaIOTNode.handle ();

}
