/**
  * @file enigmaiot_node_nonsleepy.ino
  * @version 0.9.8
  * @date 15/07/2021
  * @author German Martin
  * @brief Node based on EnigmaIoT over ESP-NOW, in non sleeping mode
  *
  * Sensor reading code is mocked on this example. You can implement any other code you need for your specific need
  * 
  *   Actualizado a la version enigmaiot 0.9.8.
  *   * 
  */

#if !defined ESP8266 && !defined ESP32
#error Node only supports ESP8266 or ESP32 platform
#endif

#include <Arduino.h>
#include <EnigmaIOTNode.h>
#include <espnow_hal.h>
#include <CayenneLPP.h>

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

//-------------------------------//
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewPing.h>

#define BLUE_LED LED_BUILTIN
constexpr auto RESET_PIN = -1;

#define domoticz_idx 76
bool releRx,datoRx=false;

// Pines del Wemos
#define RELE_PIN D1 // como siempre
#define ONE_WIRE_BUS D6
#define TERMOSTATO_PIN D7
#define TRIGGER_PIN  D3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     D2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


float tempCaldera, tempPBaja, tempPAlta;
int termost;
unsigned long timeout;
DeviceAddress  termCaldera= { 0x28, 0xFE, 0x7C, 0x75, 0xD0, 0x01, 0x3C, 0xDA };
DeviceAddress termPBaja   = { 0x28, 0xFF, 0x34, 0xB6, 0x51, 0x17, 0x04, 0x78 };
DeviceAddress termPAlta   = { 0x28, 0xFF, 0x95, 0x6C, 0x53, 0x17, 0x04, 0xD3 };

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
ADC_MODE (ADC_VCC);

bool bypass = true, unaVez = true;

const size_t capacity = JSON_OBJECT_SIZE (5);


void connectEventHandler () {
	Serial.println ("Connected");
}

void disconnectEventHandler (nodeInvalidateReason_t reason) {
	Serial.printf ("Unregistered. Reason: %d\n", reason);
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
	DynamicJsonDocument json (capacity);
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
      		bypass = doc["nvalue"]; // 1 actua en funcion del termostato, 0 nunca activa el rele
			json["bypass"] = bypass;
			sendMsgPack(json);	
    	}


		Serial.println ();
		break;
	default:
		DEBUG_WARN ("Payload encoding %d is not (yet) supported", payloadEncoding);
	}
}



void setup () {
	pinMode(TERMOSTATO_PIN,INPUT_PULLUP);
	pinMode(RELE_PIN,OUTPUT);
  	Serial.begin (115200); Serial.println (); Serial.println ("enigmaiot_Caldera-v2.0.3 --  Enigmav0.9.8");
	Serial.println ("----Reset Pin D5----");

#ifdef ESP32
	// Turn-off the 'brownout detector' to avoid random restarts during wake up,
	// normally due to bad quality regulator on board
	WRITE_PERI_REG (RTC_CNTL_BROWN_OUT_REG, 0);
#endif

	EnigmaIOTNode.setLed (BLUE_LED);
	//pinMode (BLUE_LED, OUTPUT);
	//digitalWrite (BLUE_LED, HIGH); // Turn on LED
	EnigmaIOTNode.setResetPin (RESET_PIN);
	EnigmaIOTNode.onConnected (connectEventHandler);
	EnigmaIOTNode.onDisconnected (disconnectEventHandler);
	EnigmaIOTNode.onDataRx (processRxData);
	EnigmaIOTNode.enableClockSync ();
	EnigmaIOTNode.enableBroadcast ();

	EnigmaIOTNode.begin (&Espnow_hal, NULL, NULL, true, false);

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
	sensors.begin();
	sensors.setResolution(termCaldera, 10);
	sensors.setResolution(termPBaja, 10);
	sensors.setResolution(termPAlta, 10);

}

void showTime () {
	//const int time_freq = 10000;


	if (EnigmaIOTNode.hasClockSync ()) {
        //static time_t displayTime;
        tm timeinfo;

		//displayTime = millis ();
		//time_t local_time_ms = EnigmaIOTNode.clock ();
		//local_time_ms /= 1000;
		time_t local_time = EnigmaIOTNode.unixtime ();
		localtime_r (&local_time, &timeinfo);
		//Serial.printf ("Timestamp ms: %lld\n", local_time_ms);
		//Serial.printf ("Timestamp sec: %ld\n", local_time);
		Serial.printf ("%02d/%02d/%04d %02d:%02d:%02d\n",
					   timeinfo.tm_mday,
					   timeinfo.tm_mon + 1,
					   timeinfo.tm_year + 1900,
					   timeinfo.tm_hour,
					   timeinfo.tm_min,
					   timeinfo.tm_sec);
	} else {
		Serial.printf ("Time not sync'ed\n");
	}

}

void userCode(){
	// Read sensor data
		// Put here your code to read sensor and compose buffer

		//---------------------------------------------------------------------------
		DynamicJsonDocument json (capacity);

		// locate devices on the bus
		Serial.print("Locating devices...");
		Serial.print("Found ");
		Serial.print(sensors.getDeviceCount(), DEC);
		Serial.println(" devices.");

		sensors.requestTemperatures();
		// print the device information
		float tempCaldera = sensors.getTempC(termCaldera);
		float tempPBaja = sensors.getTempC(termPBaja);
		float tempPAlta = sensors.getTempC(termPAlta);
		int lecturas=0,suma=0;
		float nivelPellets;
		int distancia;

		Serial.printf ("tempCaldera: %f\n", tempCaldera);
		Serial.printf ("tempPAlta: %f\n", tempPAlta);
		Serial.printf ("tempPBaja: %f\n", tempPBaja);

		// Nivel
		//msg.addAnalogInput (0, (float)(ESP.getVcc ()) / 1000);
		//msg.addTemperature (1, 20.34);

		//Serial.printf ("Vcc: %f\n", (float)(ESP.getVcc ()) / 1000);

		/*msg.addTemperature (8, tempCaldera);
		msg.addTemperature (9, tempPBaja);
		msg.addTemperature (10, tempPAlta);*/
		json["idx"] = 8;
		json["nvalue"] = 0;
		json["svalue"] = String (tempCaldera);

		sendMsgPack(json);
			//--
		json["idx"] = 9;
		json["nvalue"] = 0;
		json["svalue"] = String (tempPBaja);

		sendMsgPack(json);
		//--

		json["idx"] = 10;
		json["nvalue"] = 0;
		json["svalue"] = String (tempPAlta);

		sendMsgPack(json);
		//--

		for(int i=0;i<3;i++){
			delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
			Serial.print("Ping: ");
			Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
			Serial.println("cm");
			distancia= sonar.ping_cm();
		}

		if((distancia>3)&&(distancia<120)){
			Serial.println("Medida correcta,envia");
			nivelPellets=map(distancia,3,120,100,0);
			/*msg.addAnalogInput(11, nivelPellets);
			msg.addDistance(35,distancia);*/
			json["idx"] = 11;
			json["nvalue"] = 0;
			json["svalue"] = String (nivelPellets);
			sendMsgPack(json);
			//--
		json["idx"] = 35;
		json["nvalue"] = 0;
		json["svalue"] = String (distancia);
		sendMsgPack(json);
		//--
		Serial.printf ("Distancia: %i\n", distancia);
		Serial.printf ("Nivel pellets: %f\n", nivelPellets);
		}
		json["idx"] = 77;  //termostato
		json["nvalue"] = termost;
		json["svalue"] = "0";
		sendMsgPack(json);
		// End of user code
}

void loop () {

	EnigmaIOTNode.handle ();

	termost = !digitalRead(TERMOSTATO_PIN);

	CayenneLPP msg (20);

	DynamicJsonDocument json (capacity);

	static time_t lastSensorData;
	static const time_t SENSOR_PERIOD = 300000;
	static time_t lastStart;
	static const time_t START_PERIOD = 7200000;  // 2 horas
	if (millis () - lastSensorData > SENSOR_PERIOD) {
		lastSensorData = millis ();
		showTime ();
		userCode();
	}
	static int lastState = 1;
	static bool permisoTemp = true;
	if(termost == HIGH && unaVez){
		unaVez = false;

		Serial.println("Demanda de caldera");
	}
	if ((termost == HIGH) && bypass && permisoTemp && !lastState){
		lastState = true;
		json["idx"] = 78; // caldera
		json["nvalue"] = 1;
		json["svalue"] = "0";
		sendMsgPack(json);
		digitalWrite (RELE_PIN,1);
		Serial.println("Orden arranque caldera");

	}
	else if (termost == LOW && lastState == HIGH){
		lastStart = millis();
		permisoTemp = LOW;
		unaVez = true;
		digitalWrite (RELE_PIN,0);
		lastState = false;
		json["idx"] = 78; // caldera
		json["nvalue"] = 0;
		json["svalue"] = "0";
		sendMsgPack(json);
		Serial.println("Desactivado 2 horas");
	}
	if (millis () - lastStart > START_PERIOD && !permisoTemp) {
		permisoTemp = HIGH;
		Serial.println("Arranque permitido por tiempo");
	}

}