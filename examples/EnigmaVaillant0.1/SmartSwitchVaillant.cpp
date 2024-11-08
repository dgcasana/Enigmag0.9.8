// 
// 
// 

#include <functional>
#include "SmartSwitchVaillant.h"

using namespace std;
using namespace placeholders;

constexpr auto CONFIG_FILE = "/customconf.json"; ///< @brief Custom configuration file name

// -----------------------------------------
// You may add some global variables you need here,
// like serial port instances, I2C, etc
// -----------------------------------------
const char* relayKey = "rly";
const char* commandKey = "cmd";
const char* statusKey = "status";
const char* infoKey = "info";



OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

int termosta;
unsigned long timeout;
DeviceAddress termIdaBaja   =  { 0x28, 0x6A, 0x33, 0x75, 0xD0, 0x01, 0x3C, 0x31 }; // 286A3375D0013C31
DeviceAddress termRetAlta   =   { 0x28, 0xFF, 0x34, 0xB6, 0x51, 0x17, 0x04, 0x78 };// 28204775D0013CF9
DeviceAddress termRetBaja    =  { 0x28, 0xFF, 0xC1, 0xEF, 0x52, 0x17, 0x04, 0xA3 }; // 28FFC1EF521704A3
DeviceAddress termIdaAlta   =   { 0x28, 0x20, 0x47, 0x75, 0xD0, 0x01, 0x3C, 0xF9 };// 28FF34B651170478
bool unaVez = true;
const size_t capacity = JSON_OBJECT_SIZE (5);

int lastState = 0;//, tParo, tArran;
bool perTempo = true, perRelay = true;
static time_t lastStart;



float tempRetBaja, tempIdaBaja, tempRetAlta, tempIdaAlta;



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
		
		} else if (!strcmp (doc[commandKey], statusKey)) {
			DEBUG_WARN ("Request node status.");
			if (!sendNodeStatus ()) {
				DEBUG_WARN ("Error sending node status");
				return false;
			}

		} else if (!strcmp (doc[commandKey], infoKey)) {
			DEBUG_WARN ("Request info commands.");
			if (!sendInfoCommnads ()) {
				DEBUG_WARN ("Error sending node status");
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
		}
	}
	return true;
}

bool CONTROLLER_CLASS_NAME::sendRelayStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (6);
	DynamicJsonDocument json (capacity);

	json[commandKey] = relayKey;
    json[relayKey] = config.relayStatus ? 1 : 0;
    

	return sendJson (json);
}


bool CONTROLLER_CLASS_NAME::sendNodeStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (9);
	DynamicJsonDocument json (capacity);

	json["lastState"] = lastState;
    json[relayKey] = config.relayStatus ? 1 : 0;
	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendInfoCommnads () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);
	
	json["get"] = "rly,bypass,status";
    json["set"] = "rly,bypass,tParo,tArranque,tMinima,enPellet";
	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

void CONTROLLER_CLASS_NAME::connectInform () {

#if SUPPORT_HA_DISCOVERY    
// Register every HAEntity discovery function here. As many as you need
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAIdaBajaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAIdaAltaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHARetBajaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHARetAltaDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAEvUDiscovery, this));
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;

    // You do node setup here. Use it as it was the normal setup() Arduino function

	pinMode (RELAY_PIN, OUTPUT);
	sensors.begin();
	sensors.setResolution(termRetBaja, 10);
	sensors.setResolution(termIdaBaja, 10);
	sensors.setResolution(termRetAlta, 10);
	sensors.setResolution(termIdaAlta, 10);

    if (config.bootStatus != SAVE_RELAY_STATUS) {
		config.relayStatus = (bool)config.bootStatus;
		DEBUG_WARN ("Relay status set to Boot Status %d -> %d", config.bootStatus, config.relayStatus);
	}
	DEBUG_WARN ("Relay status set to %s", config.relayStatus ? "ON" : "OFF");
	
	setRelay(config.relayStatus);

	//tArran = config.tArranq;
	//tParo = config.tParo;

	// Send a 'hello' message when initalizing is finished
    if (!(enigmaIotNode->getNode ()->getSleepy ())) {
        sendStartAnouncement ();  // Disable this if node is sleepy
    }

	
	DEBUG_DBG ("Finish begin");

	// If your node should sleep after sending data do all remaining tasks here
}

void CONTROLLER_CLASS_NAME::setRelay (bool state) {
	DEBUG_WARN ("Set relay %s", state ? "ON" : "OFF");
	config.relayStatus = state;
	digitalWrite (RELAY_PIN, config.relayStatus ? ON : OFF);

	if(state){
		lastState = true;
	}else{
		lastStart = millis();
		
		unaVez = true;
		lastState = false;
	}

	if (config.bootStatus == SAVE_RELAY_STATUS) {
		if (saveConfig ()) {
			DEBUG_WARN ("Config updated. Relay is %s", config.relayStatus ? "ON" : "OFF");
		} else {
			DEBUG_ERROR ("Error saving config");
		}
	}
	

}

void CONTROLLER_CLASS_NAME::userCode(){
	// Read sensor data
		// Put here your code to read sensor and compose buffer
			
		//---------------------------------------------------------------------------
		const size_t capacity = JSON_OBJECT_SIZE (6);
		DynamicJsonDocument json (capacity);
		
		// locate devices on the bus
		Serial.print("Locating devices...");
		Serial.print("Found ");
		Serial.print(sensors.getDeviceCount(), DEC);
		Serial.println(" devices.");
			
		sensors.requestTemperatures();
		// print the device information
		tempRetBaja = sensors.getTempC(termRetBaja);
		tempIdaBaja = sensors.getTempC(termIdaBaja);
		tempRetAlta = sensors.getTempC(termRetAlta);
		tempIdaAlta = sensors.getTempC(termIdaAlta);
		//int lecturas=0,suma=0;
		int distancia;

		Serial.printf ("tempRetBaja: %f\n", tempRetBaja);
		Serial.printf ("tempRetAlta: %f\n", tempRetAlta);
		Serial.printf ("tempIdaAlta: %f\n", tempIdaAlta);
		Serial.printf ("tempIdaBaja: %f\n", tempIdaBaja);
			
		// Nivel
		//msg.addAnalogInput (0, (float)(ESP.getVcc ()) / 1000);
		//msg.addTemperature (1, 20.34);

		//Serial.printf ("Vcc: %f\n", (float)(ESP.getVcc ()) / 1000);
		
		/*msg.addTemperature (8, tempRetorno);
		msg.addTemperature (9, tempPBaja);
		msg.addTemperature (10, tempPAlta);*/
		
		if (0 < tempRetBaja) json["RetBaja"] = tempRetBaja;  // evitamos temp -127
		if (0 < tempIdaBaja) json["IdaBaja"] = tempIdaBaja;
		if (0 < tempRetAlta) json["RetAlta"] = tempRetAlta;
		if (0 < tempIdaAlta) json["IdaAlta"] = tempIdaAlta;

		//sendJson (json);
				
		//sendMsgPack(json);
		//--
			
		sendJson(json);
		// End of user code
}



void CONTROLLER_CLASS_NAME::loop () {

	// If your node stays allways awake do your periodic task here

	// You can send your data as JSON. This is a basic example

		//const size_t capacity = JSON_OBJECT_SIZE (4);
		//DynamicJsonDocument json (capacity);
		//json["sensor"] = data_description;
		//json["meas"] = measurement;

		//sendJson (json);
	static clock_t lastSentStatus;
    static clock_t sendStatusPeriod = 6000;
    if (enigmaIotNode->isRegistered () && millis () - lastSentStatus > sendStatusPeriod) {
        lastSentStatus = millis ();
        sendStatusPeriod = 60000;
        sendRelayStatus ();
		//showTime ();
		userCode();
		
    }
}

CONTROLLER_CLASS_NAME::~CONTROLLER_CLASS_NAME () {
	// It your class uses dynamic data free it up here
	// This is normally not needed but it is a good practice
}

void CONTROLLER_CLASS_NAME::configManagerStart () {
	DEBUG_INFO ("==== CCost Controller Configuration start ====");
	// If you need to add custom configuration parameters do it here
}

void CONTROLLER_CLASS_NAME::configManagerExit (bool status) {
	DEBUG_INFO ("==== CCost Controller Configuration result ====");
	// You can read configuration paramenter values here
	if (status) {

		config.ON_STATE = ON;
		config.bootStatus = SAVE_RELAY_STATUS;
				
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
	
	//config.relayPin = RELAY_PIN;
	config.ON_STATE = ON;
	config.bootStatus = SAVE_RELAY_STATUS;
	
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

			if (doc.containsKey ("ON_STATE") &&
				doc.containsKey ("bootStatus")) {

				json_correct = true;
				//config.bypass = doc["linked"].as<bool> ();
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

bool CONTROLLER_CLASS_NAME::saveConfig() {
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

	/*config.bootStatus = SAVE_RELAY_STATUS;
	config.bypassStatus = SAVE_RELAY_STATUS;*/

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
	//size_t size = configFile.size ();

	//configFile.write ((uint8_t*)(&mqttgw_config), sizeof (mqttgw_config));
	configFile.close ();
	DEBUG_DBG ("Smart Switch controller configuration saved to flash. %u bytes", size);

	return true;	
}

#if SUPPORT_HA_DISCOVERY   
// Repeat this method for every entity



void CONTROLLER_CLASS_NAME::buildHAIdaBajaDiscovery () {
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

    haEntity->setNameSufix ("ida_Baja");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("IdaBaja");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHAIdaAltaDiscovery () {
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

    haEntity->setNameSufix ("ida_Alta");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("IdaAlta");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHARetBajaDiscovery () {
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

    haEntity->setNameSufix ("ret_Baja");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("RetBaja");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHARetAltaDiscovery () {
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

    haEntity->setNameSufix ("ret_Alta");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("RetAlta");  // nombre del json del valor a capurar 
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

void CONTROLLER_CLASS_NAME::buildHAEvUDiscovery () {
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

    haEntity->setNameSufix ("EVU");
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

#endif // SUPPORT_HA_DISCOVERY
