// 
// 
// 

#include <functional>
#include "SmartSwitchController.h"

using namespace std;
using namespace placeholders;

constexpr auto CONFIG_FILE = "/customconf.json"; ///< @brief Custom configuration file name

// -----------------------------------------
// You may add some global variables you need here,
// like serial port instances, I2C, etc
// -----------------------------------------
const char* relayKey = "rly";
const char* commandKey = "cmd";
const char* buttonKey = "button";
const char* linkKey = "link";
const char* bootStateKey = "bstate";

#define ONE_WIRE_BUS D4

bool CONTROLLER_CLASS_NAME::processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	// Process incoming messages here
	// They are normally encoded as MsgPack so you can confert them to JSON very easily
	if (command != nodeMessageType_t::DOWNSTREAM_DATA_GET && command != nodeMessageType_t::DOWNSTREAM_DATA_SET) {
		DEBUG_WARN ("Wrong message type");
		return false;
	}
	// Check payload encoding
	if (payloadEncoding != MSG_PACK) {
		DEBUG_WARN ("Wrong payload encoding");
		return false;
	}

	// Decode payload
	DynamicJsonDocument doc (256);
	uint8_t tempBuffer[MAX_MESSAGE_LENGTH];

	memcpy (tempBuffer, buffer, length);
	DeserializationError error = deserializeMsgPack (doc, tempBuffer, length);
	// Check decoding
	if (error != DeserializationError::Ok) {
		DEBUG_WARN ("Error decoding command: %s", error.c_str ());
		return false;
	}

	DEBUG_WARN ("Command: %d = %s", command, command == nodeMessageType_t::DOWNSTREAM_DATA_GET ? "GET" : "SET");

	// Dump debug data
	size_t strLen = measureJson (doc) + 1;
	char* strBuffer = (char*)malloc (strLen);
	serializeJson (doc, strBuffer, strLen);
	DEBUG_WARN ("Data: %s", strBuffer);
	free (strBuffer);

	// Check cmd field on JSON data
	if (!doc.containsKey (commandKey)) {
		DEBUG_WARN ("Wrong format");
		return false;
	}

	if (command == nodeMessageType_t::DOWNSTREAM_DATA_GET) {
		if (!strcmp (doc[commandKey], relayKey)) {
			DEBUG_WARN ("Request relay status. Relay = %s", config.relayStatus == ON ? "ON" : "OFF");
			if (!sendRelayStatus ()) {
				DEBUG_WARN ("Error sending relay status");
				return false;
			}
		} else if (!strcmp (doc[commandKey], linkKey)) {
			DEBUG_WARN ("Request link status. Link = %s", config.linked ? "enabled" : "disabled");
			if (!sendLinkStatus ()) {
				DEBUG_WARN ("Error sending link status");
				return false;
			}

		} else if (!strcmp (doc[commandKey], bootStateKey)) {
			DEBUG_WARN ("Request boot status configuration. Boot = %d",
						config.bootStatus);
			if (!sendBootStatus ()) {
				DEBUG_WARN ("Error sending boot status configuration");
				return false;
			}

		}
	}

	if (command == nodeMessageType_t::DOWNSTREAM_DATA_SET) {
		if (!strcmp (doc[commandKey], relayKey)) {
			if (!doc.containsKey (relayKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			//DEBUG_WARN ("Set relay status. Relay = %s", doc[relayKey].as<bool> () ? "ON" : "OFF");

			setRelay (doc[relayKey].as<bool> ());

			if (!sendRelayStatus ()) {
				DEBUG_WARN ("Error sending relay status");
				return false;
			}


		} else if (!strcmp (doc[commandKey], linkKey)) {
			if (!doc.containsKey (linkKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set link status. Link = %s", doc[linkKey].as<bool> () ? "enabled" : "disabled");

			setLinked (doc[linkKey].as<bool> ());

			if (!sendLinkStatus ()) {
				DEBUG_WARN ("Error sending link status");
				return false;
			}

		} else if (!strcmp (doc[commandKey], bootStateKey)) {
			if (!doc.containsKey (bootStateKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set boot status. Link = %d", doc[bootStateKey].as<int> ());

			setBoot (doc[bootStateKey].as<int> ());

			if (!sendBootStatus ()) {
				DEBUG_WARN ("Error sending boot status configuration");
				return false;
			}

		}
	}

	return true;
}

bool CONTROLLER_CLASS_NAME::sendRelayStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (6);
	DynamicJsonDocument json (capacity);

	json[commandKey] = relayKey;
    json[relayKey] = config.relayStatus ? 1 : 0;
    json[linkKey] = config.linked ? 1 : 0;
    json[bootStateKey] = config.bootStatus;

	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendLinkStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);

	json[commandKey] = linkKey;
    json[linkKey] = config.linked ? 1 : 0;

	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendBootStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);

	json[commandKey] = bootStateKey;
	int bootStatus = config.bootStatus;
	json[bootStateKey] = bootStatus;

	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

bool CONTROLLER_CLASS_NAME::sendTemperature (float temp, float temp1) {
	const size_t capacity = JSON_OBJECT_SIZE (3);
	DynamicJsonDocument json (capacity);
	json["temp0"] = temp;
	json["temp1"] = temp1;

	return sendJson (json);
}

void CONTROLLER_CLASS_NAME::connectInform () {

#if SUPPORT_HA_DISCOVERY    
    // Register every HAEntity discovery function here. As many as you need
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHASwitchDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHATriggerDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHALinkDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHATempin, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHATempout, this));
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;

	// You do node setup here. Use it as it was the normal setup() Arduino function
	pinMode (config.buttonPin, INPUT_PULLUP);
	pinMode (config.relayPin, OUTPUT);

	#if !TEST
	oneWire = new OneWire (ONE_WIRE_BUS);
	sensors = new DallasTemperature (oneWire);
	sensors->begin ();
	sensors->setWaitForConversion (false);
	//sensors->requestTemperatures ();     Esto en el loop
#endif
    
    if (config.bootStatus != SAVE_RELAY_STATUS) {
		config.relayStatus = (bool)config.bootStatus;
		DEBUG_WARN ("Relay status set to Boot Status %d -> %d", config.bootStatus, config.relayStatus);
	}
	DEBUG_WARN ("Relay status set to %s", config.relayStatus ? "ON" : "OFF");
	digitalWrite (config.relayPin, config.relayStatus);
	// if (!sendRelayStatus ()) {
	// 	DEBUG_WARN ("Error sending relay status");
	// }

    // Send a 'hello' message when initalizing is finished
	sendStartAnouncement ();

	DEBUG_WARN ("Finish begin");

	// If your node should sleep after sending data do all remaining tasks here
}

void CONTROLLER_CLASS_NAME::toggleRelay () {
	DEBUG_INFO ("Toggle relay");
	config.relayStatus = !config.relayStatus;
	digitalWrite (config.relayPin, config.relayStatus ? ON : OFF);
	if (config.bootStatus == SAVE_RELAY_STATUS) {
		if (saveConfig ()) {
			DEBUG_INFO ("Config updated. Relay is %s", config.relayStatus ? "ON" : "OFF");
		} else {
			DEBUG_ERROR ("Error saving config");
		}
	}
	sendRelayStatus ();
}

void CONTROLLER_CLASS_NAME::setRelay (bool state) {
	DEBUG_WARN ("Set relay %s", state ? "ON" : "OFF");
	config.relayStatus = state;
	digitalWrite (config.relayPin, config.relayStatus ? ON : OFF);
	if (config.bootStatus == SAVE_RELAY_STATUS) {
		if (saveConfig ()) {
			DEBUG_WARN ("Config updated. Relay is %s", config.relayStatus ? "ON" : "OFF");
		} else {
			DEBUG_ERROR ("Error saving config");
		}
	}
}

void CONTROLLER_CLASS_NAME::setLinked (bool state) {
	DEBUG_WARN ("Set link %s", state ? "ON" : "OFF");
	config.linked = state;
	if (saveConfig ()) {
		DEBUG_WARN ("Config updated. Relay is %slinked", !config.relayStatus ? "not " : "");
	} else {
		DEBUG_ERROR ("Error saving config");
	}
}

void CONTROLLER_CLASS_NAME::setBoot (int state) {
	DEBUG_WARN ("Set boot state to %d", state);
	if (state >= RELAY_OFF && state <= SAVE_RELAY_STATUS) {
		config.bootStatus = (bootRelayStatus_t)state;
	} else {
		config.bootStatus = RELAY_OFF;
	}

	if (saveConfig ()) {
		DEBUG_WARN ("Config updated");
	} else {
		DEBUG_ERROR ("Error saving config");
	}
}

void CONTROLLER_CLASS_NAME::loop () {

	tempSent = false;
	// If your node stays allways awake do your periodic task here
	if (pushReleased) { // Enter this only if button were not pushed in the last loop
		if (!digitalRead (config.buttonPin)) {
			delay (50); // debounce button push
			if (!digitalRead (config.buttonPin)) {
				DEBUG_INFO ("Button triggered!");
				pushTriggered = true; // Button is pushed
				pushReleased = false; // Mark button as not released
			}
		}
	}

	if (pushTriggered) { // If button was pushed
		pushTriggered = false; // Disable push trigger
		const size_t capacity = JSON_OBJECT_SIZE (2);
		DynamicJsonDocument json (capacity);
		json[buttonKey] = config.buttonPin;
		json["push"] = 1;
		if (sendJson (json)) {
			DEBUG_INFO ("Push triggered sent");
		} else {
			DEBUG_ERROR ("Push send error");
		}
		if (config.linked) {
			toggleRelay ();
		}
	}

	if (!pushReleased) {
		if (digitalRead (config.buttonPin)) { // If button is released
			DEBUG_INFO ("Button released");
			pushReleased = true;
		}
    }

    static clock_t lastSentStatus;
    static clock_t sendStatusPeriod = 2000;
    if (enigmaIotNode->isRegistered () && millis () - lastSentStatus > sendStatusPeriod) {
        lastSentStatus = millis ();
        sendStatusPeriod = 60000;
        sendRelayStatus ();
		sensors.requestTemperatures();

		time_t start = millis ();

    	while (!sensors->isConversionComplete ()) {
			delay (0);
		}
	
    	tempC = sensors->getTempCByIndex (0);
		tempC1 = sensors->getTempCByIndex (1);
		
		if (!tempSent && enigmaIotNode->isRegistered()) {
        	if (sendTemperature (tempC, tempC1) ) {
            	tempSent = true;
        	}
			
    	}

	static clock_t lastOnStatus;
    static clock_t runWarmPeriod = 300000; //5minutos
	static bool warm = false;

	if (tempC > 35 && !warm){
		lastOnStatus = millis();
		warm = true;
		setRelay (true);

	}
	if (warm && millis () - lastOnStatus > runWarmPeriod ) {
		setRelay (false);
	}



    }
}

CONTROLLER_CLASS_NAME::~CONTROLLER_CLASS_NAME () {
	// It your class uses dynamic data free it up here
	// This is normally not needed but it is a good practice
	if (buttonPinParam) {
		delete (buttonPinParam);
		delete (relayPinParam);
		delete (bootStatusListParam);
		delete (bootStatusParam);
	}
}

void CONTROLLER_CLASS_NAME::configManagerStart () {
	DEBUG_WARN ("==== CCost Controller Configuration start ====");
	// If you need to add custom configuration parameters do it here

	static char buttonPinParamStr[4];
	itoa (DEFAULT_BUTTON_PIN, buttonPinParamStr, 10);
	static char relayPinParamStr[4];
	itoa (DEFAULT_RELAY_PIN, relayPinParamStr, 10);
	buttonPinParam = new AsyncWiFiManagerParameter ("buttonPin", "Button Pin", buttonPinParamStr, 3, "required type=\"text\" pattern=\"^1[2-5]$|^[0-5]$\"");
	relayPinParam = new AsyncWiFiManagerParameter ("relayPin", "Relay Pin", relayPinParamStr, 3, "required type=\"text\" pattern=\"^1[2-5]$|^[0-5]$\"");
	bootStatusParam = new AsyncWiFiManagerParameter ("bootStatus", "Boot Relay Status", "", 6, "required type=\"text\" list=\"bootStatusList\" pattern=\"^ON$|^OFF$|^SAVE$\"");
	bootStatusListParam = new AsyncWiFiManagerParameter ("<datalist id=\"bootStatusList\">" \
														 "<option value = \"OFF\" >" \
														 "<option valsenue = \"OFF\" >" \
														 "<option value = \"ON\">" \
														 "<option value = \"SAVE\">" \
														 "</datalist>");
	EnigmaIOTNode.addWiFiManagerParameter (buttonPinParam);
    EnigmaIOTNode.addWiFiManagerParameter (relayPinParam);
    EnigmaIOTNode.addWiFiManagerParameter (bootStatusListParam);
    EnigmaIOTNode.addWiFiManagerParameter (bootStatusParam);
}

void CONTROLLER_CLASS_NAME::configManagerExit (bool status) {
	DEBUG_WARN ("==== CCost Controller Configuration result ====");
	// You can read configuration paramenter values here
	DEBUG_WARN ("Button Pin: %s", buttonPinParam->getValue ());
	DEBUG_WARN ("Boot Relay Status: %s", bootStatusParam->getValue ());

	// TODO: Finish bootStatusParam analysis

	if (status) {
		config.buttonPin = atoi (buttonPinParam->getValue ());
		if (config.buttonPin > 15 || config.buttonPin < 0) {
			config.buttonPin = DEFAULT_BUTTON_PIN;
		}
		config.relayPin = atoi (relayPinParam->getValue ());
		if (config.relayPin > 15 || config.relayPin < 0) {
			config.relayPin = DEFAULT_BUTTON_PIN;
		}
		if (!strncmp (bootStatusParam->getValue (), "ON", 6)) {
			config.bootStatus = RELAY_ON;
		} else if (!strncmp (bootStatusParam->getValue (), "SAVE", 6)) {
			config.bootStatus = SAVE_RELAY_STATUS;
		} else {
			config.bootStatus = RELAY_OFF;
		}
		config.ON_STATE = ON;
		config.linked = true;

		if (!saveConfig ()) {
			DEBUG_ERROR ("Error writting blind controller config to filesystem.");
		} else {
			DEBUG_WARN ("Configuration stored");
		}
	} else {
		DEBUG_WARN ("Configuration does not need to be saved");
	}
}

void CONTROLLER_CLASS_NAME::defaultConfig () {
	config.buttonPin = DEFAULT_BUTTON_PIN;
	config.relayPin = DEFAULT_RELAY_PIN;
	config.linked = true;
	config.ON_STATE = ON;

}

bool CONTROLLER_CLASS_NAME::loadConfig () {
	// If you need to read custom configuration data do it here
	bool json_correct = false;

	if (!FILESYSTEM.begin ()) {
		DEBUG_WARN ("Error starting filesystem. Formatting");
        FILESYSTEM.format ();
	}

    // FILESYSTEM.remove (CONFIG_FILE); // Only for testing

    if (FILESYSTEM.exists (CONFIG_FILE)) {
		DEBUG_WARN ("Opening %s file", CONFIG_FILE);
        File configFile = FILESYSTEM.open (CONFIG_FILE, "r");
		if (configFile) {
			size_t size = configFile.size ();
			DEBUG_WARN ("%s opened. %u bytes", CONFIG_FILE, size);
			DynamicJsonDocument doc (512);
			DeserializationError error = deserializeJson (doc, configFile);
			if (error) {
				DEBUG_ERROR ("Failed to parse file");
			} else {
				DEBUG_WARN ("JSON file parsed");
			}

			if (doc.containsKey ("buttonPin") &&
				doc.containsKey ("relayPin") &&
				doc.containsKey ("linked") &&
				doc.containsKey ("ON_STATE") &&
				doc.containsKey ("bootStatus")) {

				json_correct = true;
				config.buttonPin = doc["buttonPin"].as<int> ();
				config.relayPin = doc["relayPin"].as<int> ();
				config.linked = doc["linked"].as<bool> ();
				config.ON_STATE = doc["ON_STATE"].as<int> ();
				int bootStatus = doc["bootStatus"].as<int> ();
				if (bootStatus >= RELAY_OFF && bootStatus <= SAVE_RELAY_STATUS) {
					config.bootStatus = (bootRelayStatus_t)bootStatus;
					DEBUG_WARN ("Boot status set to %d", config.bootStatus);
				} else {
					config.bootStatus = RELAY_OFF;
				}
			}

			if (doc.containsKey ("relayStatus")) {
				config.relayStatus = doc["relayStatus"].as<bool> ();
			}

			configFile.close ();
			if (json_correct) {
				DEBUG_WARN ("Smart switch controller configuration successfuly read");
			} else {
				DEBUG_WARN ("Smart switch controller configuration error");
			}
			DEBUG_WARN ("==== Smart switch Controller Configuration ====");
			DEBUG_WARN ("Button pin: %d", config.buttonPin);
			DEBUG_WARN ("Relay pin: %d", config.relayPin);
			DEBUG_WARN ("Linked: %s", config.linked ? "true" : "false");
			DEBUG_WARN ("ON level: %s ", config.ON_STATE ? "HIGH" : "LOW");
			DEBUG_WARN ("Boot relay status: %d ", config.bootStatus);


			size_t jsonLen = measureJsonPretty (doc) + 1;
			char* output = (char*)malloc (jsonLen);
			serializeJsonPretty (doc, output, jsonLen);

			DEBUG_WARN ("File content:\n%s", output);

			free (output);

		} else {
			DEBUG_WARN ("Error opening %s", CONFIG_FILE);
			defaultConfig ();
		}
	} else {
		DEBUG_WARN ("%s do not exist", CONFIG_FILE);
		defaultConfig ();
	}

	return json_correct;
}

bool CONTROLLER_CLASS_NAME::saveConfig () {
    // If you need to save custom configuration data do it here
    if (!FILESYSTEM.begin ()) {
		DEBUG_WARN ("Error opening filesystem");
		return false;
	}
	DEBUG_INFO ("Filesystem opened");

    File configFile = FILESYSTEM.open (CONFIG_FILE, "w");
	if (!configFile) {
		DEBUG_WARN ("Failed to open config file %s for writing", CONFIG_FILE);
		return false;
	} else {
		DEBUG_INFO ("%s opened for writting", CONFIG_FILE);
	}

	DynamicJsonDocument doc (512);

	doc["buttonPin"] = config.buttonPin;
	doc["relayPin"] = config.relayPin;
	doc["linked"] = config.linked;
	doc["ON_STATE"] = config.ON_STATE;
	doc["relayStatus"] = config.relayStatus;
	int bootStatus = config.bootStatus;
	doc["bootStatus"] = bootStatus;

	if (serializeJson (doc, configFile) == 0) {
		DEBUG_ERROR ("Failed to write to file");
		configFile.close ();
        //FILESYSTEM.remove (CONFIG_FILE);
		return false;
	}

	size_t jsonLen = measureJsonPretty (doc) + 1;
	char* output = (char*)malloc (jsonLen);
	serializeJsonPretty (doc, output, jsonLen);

	DEBUG_DBG ("File content:\n%s", output);

	free (output);

	configFile.flush ();
	size_t size = configFile.size ();

	//configFile.write ((uint8_t*)(&mqttgw_config), sizeof (mqttgw_config));
	configFile.close ();
	DEBUG_DBG ("Smart Switch controller configuration saved to flash. %u bytes", size);

	return true;
}

#if SUPPORT_HA_DISCOVERY   
// Repeat this method for every entity
void CONTROLLER_CLASS_NAME::buildHASwitchDiscovery () {
    // Select corresponding HAEntiny type
    HASwitch* haEntity = new HASwitch ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haEntity->setNameSufix ("switch");
    haEntity->setStateOn (1);
    haEntity->setStateOff (0);
    haEntity->setValueField ("rly");
    haEntity->setPayloadOff ("{\"cmd\":\"rly\",\"rly\":0}");
    haEntity->setPayloadOn ("{\"cmd\":\"rly\",\"rly\":1}");
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

void CONTROLLER_CLASS_NAME::buildHALinkDiscovery () {
    // Select corresponding HAEntiny type
    HASwitch* haEntity = new HASwitch ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haEntity->setNameSufix ("link");
    haEntity->setStateOn (1);
    haEntity->setStateOff (0);
    haEntity->setValueField ("link");
    haEntity->setPayloadOff ("{\"cmd\":\"link\",\"link\":0}");
    haEntity->setPayloadOn ("{\"cmd\":\"link\",\"link\":1}");
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

void CONTROLLER_CLASS_NAME::buildHATriggerDiscovery () {
    // Select corresponding HAEntiny type
    HATrigger* haEntity = new HATrigger ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haEntity->setNameSufix ("button");
    haEntity->setType (button_short_press);
    haEntity->setSubtype (turn_on);
    haEntity->setPayload ("{\"button\":4,\"push\":1}");
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

void CONTROLLER_CLASS_NAME::buildHATempin () {
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

    haEntity->setNameSufix ("piscina_in");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("temp0");
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
void CONTROLLER_CLASS_NAME::buildHATempout () {
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

    haEntity->setNameSufix ("piscina_out");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("temp1");
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
