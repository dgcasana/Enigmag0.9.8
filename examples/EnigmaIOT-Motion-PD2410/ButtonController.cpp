// 
// 
// 

#include <functional>
#include "ButtonController.h"
#include <ld2410.h>
	#define MONITOR_SERIAL Serial
    #define RADAR_SERIAL Serial1
    #define RADAR_RX_PIN 4
    #define RADAR_TX_PIN 3
	ld2410 radar;
	uint32_t lastReading = 0;
	bool radarConnected = false;

using namespace std;
using namespace placeholders;

constexpr auto CONFIG_FILE = "/customconf.json"; ///< @brief Custom configuration file name

// -----------------------------------------
// You may add some global variables you need here,
// like serial port instances, I2C, etc
// -----------------------------------------


bool CONTROLLER_CLASS_NAME::processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	// Process incoming messages here
	// They are normally encoded as MsgPack so you can convert them to JSON very easily
	return true;
}


bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

void CONTROLLER_CLASS_NAME::connectInform () {

    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHABinarySensorDiscoveryM, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHABinarySensorDiscoveryS, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHADiscoveryM, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHADiscoveryS, this));

    EnigmaIOTjsonController::connectInform ();
    // Add more actions here if needed
    // Keep this method duration short
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;
	// You do node setup here. Use it as it was the normal setup() Arduino function
    //pinMode (BUTTON_PIN, INPUT);//_PULLUP);

	RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
	delay(500);
	MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));	
	MONITOR_SERIAL.println(RADAR_RX_PIN);
	MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
	MONITOR_SERIAL.println(RADAR_TX_PIN);
	MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));
  	if(radar.begin(RADAR_SERIAL))
 	 {
    	MONITOR_SERIAL.println(F("OK"));
    	MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    	MONITOR_SERIAL.print(radar.firmware_major_version);
    	MONITOR_SERIAL.print('.');
    	MONITOR_SERIAL.print(radar.firmware_minor_version);
    	MONITOR_SERIAL.print('.');
    	MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  	}
  	else
  	{
    	MONITOR_SERIAL.println(F("not connected"));
  	}

	// Send a 'hello' message when initalizing is finished
	sendStartAnouncement ();

	DEBUG_DBG ("Finish begin");

	// If your node should sleep after sending data do all remaining tasks here
}

void CONTROLLER_CLASS_NAME::loop () {
	const size_t capacity = JSON_OBJECT_SIZE (6);
	DynamicJsonDocument json (capacity);

	  radar.read();
  if(radar.isConnected() && millis() - lastReading > 1000)  //Report every 1000ms
  {
    lastReading = millis();
    if(radar.presenceDetected())
    {
      if(radar.stationaryTargetDetected())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.stationaryTargetEnergy());
        Serial.print(' ');
      }
      if(radar.movingTargetDetected())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance());
        Serial.print(F("cm energy:"));
        Serial.print(radar.movingTargetEnergy());
      }
      Serial.println();
    }
    else
    {
      Serial.println(F("No target"));
    }
  }

	json["stationaryT"] = radar.stationaryTargetDistance();
	json["stationaryE"] = radar.stationaryTargetEnergy();
	json["movingT"] = radar.movingTargetEnergy();
	json["movingE"] = radar.movingTargetEnergy();
	if (sendJson (json)) {
		DEBUG_WARN ("No motion sent");
	} else {
		DEBUG_ERROR ("No-motion send error");
	}

	
}

CONTROLLER_CLASS_NAME::~CONTROLLER_CLASS_NAME () {
	// If your class uses dynamic data free it up here
	// This is normally not needed but it is a good practice
}

void CONTROLLER_CLASS_NAME::configManagerStart () {

	DEBUG_INFO ("==== CCost Controller Configuration start ====");
	// If you need to add custom configuration parameters do it here
}

void CONTROLLER_CLASS_NAME::configManagerExit (bool status) {
	DEBUG_INFO ("==== CCost Controller Configuration result ====");
	// You can read configuration paramenter values here
}

bool CONTROLLER_CLASS_NAME::loadConfig () {
	// If you need to read custom configuration data do it here
	return true;
}

bool CONTROLLER_CLASS_NAME::saveConfig () {
	// If you need to save custom configuration data do it here
	return true;
}

#if SUPPORT_HA_DISCOVERY   
// Repeat this method for every entity

void CONTROLLER_CLASS_NAME::buildHABinarySensorDiscoveryM () {
    // Select corresponding HAEntiny type
    HABinarySensor* haBEntity2 = new HABinarySensor ();

    uint8_t* msgPackBuffer;

    if (!haBEntity2) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haBEntity2->setNameSufix ("movimiento");
    haBEntity2->setDeviceClass (bs_motion);
    haBEntity2->addExpiration (3600);
    haBEntity2->setPayloadOff ("0");
    haBEntity2->setPayloadOn ("1");
    haBEntity2->setValueField ("movingT");  // nombre del json del valor a capurar 
    // *******************************
	

    size_t bufferLen = haBEntity2->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haBEntity2->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_INFO ("Resulting MSG pack length: %d", len);

    if (!sendHADiscovery (msgPackBuffer, len)) {
        DEBUG_WARN ("Error sending HA discovery message");
    }

    if (haBEntity2) {
        delete (haBEntity2);
    }

    if (msgPackBuffer) {
        free (msgPackBuffer);
    }
}

void CONTROLLER_CLASS_NAME::buildHABinarySensorDiscoveryS () {
    // Select corresponding HAEntiny type
    HABinarySensor* haBEntity2 = new HABinarySensor ();

    uint8_t* msgPackBuffer;

    if (!haBEntity2) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haBEntity2->setNameSufix ("estatico");
    haBEntity2->setDeviceClass (bs_motion);
    haBEntity2->addExpiration (3600);
    haBEntity2->setPayloadOff ("0");
    haBEntity2->setPayloadOn ("1");
    haBEntity2->setValueField ("stationaryT");  // nombre del json del valor a capurar 
    // *******************************
	

    size_t bufferLen = haBEntity2->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haBEntity2->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_INFO ("Resulting MSG pack length: %d", len);

    if (!sendHADiscovery (msgPackBuffer, len)) {
        DEBUG_WARN ("Error sending HA discovery message");
    }

    if (haBEntity2) {
        delete (haBEntity2);
    }

    if (msgPackBuffer) {
        free (msgPackBuffer);
    }
}

void CONTROLLER_CLASS_NAME::buildHADiscoveryM () {
    // Select corresponding HAEntiny type
    HASensor* haEntity = new HASensor ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haEntity->setNameSufix ("Move Gate");
    haEntity->setDeviceClass (sensor_none);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("G");
    haEntity->setValueField ("movingE");
    //haEntity->setValueTemplate ("{%if value_json.dp==2-%}{{value_json.temp}}{%-else-%}{{states('sensor.***_temp')}}{%-endif%}");

    // *******************************

    size_t bufferLen = haEntity->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haEntity->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_INFO ("Resulting MSG pack length: %d", len);

    if (!sendHADiscovery (msgPackBuffer, len)) {
        DEBUG_WARN ("Error sending HA discovery message");
    }

    if (haEntity) {
        delete (haEntity);
    }

    if (msgPackBuffer) {
        free (msgPackBuffer);
    }
}

void CONTROLLER_CLASS_NAME::buildHADiscoveryS () {
    // Select corresponding HAEntiny type
    HASensor* haEntity = new HASensor ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haEntity->setNameSufix ("Static Gate");
    haEntity->setDeviceClass (sensor_none);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("G");
    haEntity->setValueField ("stationaryE");
    //haEntity->setValueTemplate ("{%if value_json.dp==2-%}{{value_json.temp}}{%-else-%}{{states('sensor.***_temp')}}{%-endif%}");

    // *******************************

    size_t bufferLen = haEntity->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haEntity->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_INFO ("Resulting MSG pack length: %d", len);

    if (!sendHADiscovery (msgPackBuffer, len)) {
        DEBUG_WARN ("Error sending HA discovery message");
    }

    if (haEntity) {
        delete (haEntity);
    }

    if (msgPackBuffer) {
        free (msgPackBuffer);
    }
}
#endif // SUPPORT_HA_DISCOVERY