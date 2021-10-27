// 
// 
// 

#include <functional>
#include "SmartSwitchCaldera.h"

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
const char* linkKey = "bypass";
const char* bootStateKey = "bstate";



NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

int termosta;
unsigned long timeout;
DeviceAddress termCaldera   =  { 0x28, 0x20, 0x47, 0x75, 0xD0, 0x01, 0x3C, 0xF9 }; //{0x28, 0xE2, 0xE4, 0x95, 0xF0, 0x01, 0x3C, 0xA8}; 
DeviceAddress termPBaja     =  { 0x28, 0xFF, 0x34, 0xB6, 0x51, 0x17, 0x04, 0x78 }; //{0x28, 0xBE, 0xF7, 0x95, 0xF0, 0x01, 0x3C, 0xC9}; 
DeviceAddress termPAlta     =  { 0x28, 0xFF, 0x95, 0x6C, 0x53, 0x17, 0x04, 0xD3 }; // {0x28, 0xCD, 0x22, 0x95, 0xF0, 0x01, 0x3C, 0x84}; 
DeviceAddress termAcumula   =  { 0x28, 0xFF, 0xA8, 0xDE, 0x9E, 0x20, 0xD6, 0xC6 }; //{0x28, 0xDB, 0x69, 0x95, 0xF0, 0x01, 0x3C, 0xF4}; 

bool /*bypass = true,*/ unaVez = true;
const size_t capacity = JSON_OBJECT_SIZE (5);


float tempPrincipal, tempPBaja, tempPAlta, tempAcumula, nivelPellets;



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

		} /*else if (!strcmp (doc[commandKey], bootStateKey)) {
			DEBUG_WARN ("Request boot status configuration. Boot = %d",
						config.bootStatus);
			if (!sendBootStatus ()) {
				DEBUG_WARN ("Error sending boot status configuration");
				return false;
			}

		}*/
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

		} /*else if (!strcmp (doc[commandKey], bootStateKey)) {
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

		}*/
	}

	return true;
}

bool CONTROLLER_CLASS_NAME::sendRelayStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (6);
	DynamicJsonDocument json (capacity);

	json[commandKey] = relayKey;
    json[relayKey] = config.relayStatus ? 1 : 0;
    //json[linkKey] = config.linked ? 1 : 0;
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

/*bool CONTROLLER_CLASS_NAME::sendBootStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);

	json[commandKey] = bootStateKey;
	int bootStatus = config.bootStatus;
	json[bootStateKey] = bootStatus;

	return sendJson (json);
}*/

bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

void CONTROLLER_CLASS_NAME::connectInform () {

#if SUPPORT_HA_DISCOVERY    
    // Register every HAEntity discovery function here. As many as you need
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPBajaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPAltaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPrincipalDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAAcumulaDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHABypassDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPelletDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHACalderaDiscovery, this));
	//addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHABypassDiscovery, this));
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;
	

	// You do node setup here. Use it as it was the normal setup() Arduino function
	//pinMode (config.buttonPin, INPUT_PULLUP);
	pinMode (config.relayPin, OUTPUT);
	//pinMode(RELE_PIN,OUTPUT);

	sensors.begin();
	sensors.setResolution(termCaldera, 10);
	sensors.setResolution(termPBaja, 10);
	sensors.setResolution(termPAlta, 10);
	sensors.setResolution(termAcumula, 10);

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

/*void CONTROLLER_CLASS_NAME::sendMsgPack (DynamicJsonDocument json) {
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
}*/

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
		tempPrincipal = sensors.getTempC(termCaldera);
		tempPBaja = sensors.getTempC(termPBaja);
		tempPAlta = sensors.getTempC(termPAlta);
		tempAcumula = sensors.getTempC(termAcumula);
		//int lecturas=0,suma=0;
		int distancia;

		Serial.printf ("tempPrincipal: %f\n", tempPrincipal);
		Serial.printf ("tempPAlta: %f\n", tempPAlta);
		Serial.printf ("tempPBaja: %f\n", tempPBaja);
			
		// Nivel
		//msg.addAnalogInput (0, (float)(ESP.getVcc ()) / 1000);
		//msg.addTemperature (1, 20.34);

		//Serial.printf ("Vcc: %f\n", (float)(ESP.getVcc ()) / 1000);
		
		/*msg.addTemperature (8, tempPrincipal);
		msg.addTemperature (9, tempPBaja);
		msg.addTemperature (10, tempPAlta);*/
		
		json["Principal"] = tempPrincipal;
		json["PBaja"] = tempPBaja;
		json["PAlta"] = tempPAlta;
		json["Acumula"] = tempAcumula;

		//sendJson (json);
				
		//sendMsgPack(json);
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
			nivelPellets = (120-distancia) * 0.83; //map(distancia,3,120,100,0);
			
			/*msg.addAnalogInput(11, nivelPellets);
			msg.addDistance(35,distancia);*/
			
			json["Pellets"] = nivelPellets;
			//sendMsgPack(json);
			//--
		/*json["idx"] = 35;
		json["nvalue"] = 0;
		json["svalue"] = String (distancia);
		sendMsgPack(json);*/
		//--
		Serial.printf ("Distancia: %i\n", distancia);
		Serial.printf ("Nivel pellets: %f\n", nivelPellets);
		}
		
		json["termostato"] = termosta ? "ON" : "OFF";

		
		sendJson(json);
		// End of user code
}

void CONTROLLER_CLASS_NAME::arranque(){
	static int lastState = 1, tParo = 57, tArran = 50;
	static bool permisoTemp = true;

	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);
	static time_t lastStart;
	static const time_t START_PERIOD = 7200000;  // 2 horas

	if (tempPBaja - tempPrincipal < 6) tParo = 60; // Si el suelo esta frio aumento la histeresis
	else tParo = 57;
	
	if ((tempPrincipal < tArran) && (nivelPellets > 35))	termosta = HIGH;
	else if( tempPrincipal > tParo) termosta = LOW;

	if(termosta == HIGH && unaVez){
		unaVez = false;
		
		Serial.println("Demanda de caldera");
	}
	if ((termosta == HIGH) && config.linked && permisoTemp && !lastState){
		lastState = true;
		
		//digitalWrite (RELE_PIN,1);
		setRelay(HIGH);
		Serial.println("Orden arranque caldera");

	}
	else if (termosta == LOW && lastState == HIGH){
		lastStart = millis();
		permisoTemp = LOW;
		unaVez = true;
		//digitalWrite (RELE_PIN,0);
		
		setRelay(LOW);
		lastState = false;
		
		/*json["caldera"] = 0;
		sendMsgPack(json);*/
		Serial.println("Desactivado 2 horas");
	}
	if (millis () - lastStart > START_PERIOD && !permisoTemp) {
		permisoTemp = HIGH;
		Serial.println("Arranque permitido por tiempo");
	}


}

void CONTROLLER_CLASS_NAME::loop () {

	// If your node stays allways awake do your periodic task here
	/*if (pushReleased) { // Enter this only if button were not pushed in the last loop
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
    }*/

    static clock_t lastSentStatus;
    static clock_t sendStatusPeriod = 2000;
    if (enigmaIotNode->isRegistered () && millis () - lastSentStatus > sendStatusPeriod) {
        lastSentStatus = millis ();
        sendStatusPeriod = 300000;
        sendRelayStatus ();
		showTime ();
		userCode();
		arranque();
    }
	
}

void CONTROLLER_CLASS_NAME::showTime () {
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

	

	/*static char buttonPinParamStr[4];
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
    EnigmaIOTNode.addWiFiManagerParameter (bootStatusParam);*/
}

void CONTROLLER_CLASS_NAME::configManagerExit (bool status) {
	/*DEBUG_WARN ("==== CCost Controller Configuration result ====");
	// You can read configuration paramenter values here
	DEBUG_WARN ("Button Pin: %s", buttonPinParam->getValue ());
	DEBUG_WARN ("Boot Relay Status: %s", bootStatusParam->getValue ());

	// TODO: Finish bootStatusParam analysis*/

	

	
}

void CONTROLLER_CLASS_NAME::defaultConfig () {
	config.buttonPin = DEFAULT_BUTTON_PIN;
	config.relayPin = DEFAULT_RELAY_PIN;
	config.linked = true;
	config.ON_STATE = ON;
	config.bootStatus = SAVE_RELAY_STATUS;

}

bool CONTROLLER_CLASS_NAME::loadConfig () {
	// If you need to read custom configuration data do it here
	return true;
	/*bool json_correct = false;

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

	return json_correct;*/
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

void CONTROLLER_CLASS_NAME::buildHABypassDiscovery () {
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

    haEntity->setNameSufix ("bypass");
    haEntity->setStateOn (1);
    haEntity->setStateOff (0);
    haEntity->setValueField ("bypass");
    haEntity->setPayloadOff ("{\"cmd\":\"bypass\",\"bypass\":0}");
    haEntity->setPayloadOn ("{\"cmd\":\"bypass\",\"bypass\":1}");
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

void CONTROLLER_CLASS_NAME::buildHAPBajaDiscovery () {
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

    haEntity->setNameSufix ("P.Baja");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("PBaja");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHAPAltaDiscovery () {
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

    haEntity->setNameSufix ("P.Alta");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("PAlta");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHAPrincipalDiscovery () {
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

    haEntity->setNameSufix ("Principal");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("Pricipal");  // nombre del json del valor a capurar 
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
void CONTROLLER_CLASS_NAME::buildHAAcumulaDiscovery () {
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

    haEntity->setNameSufix ("Acumulador");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("Acumula");  // nombre del json del valor a capurar 
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

void CONTROLLER_CLASS_NAME::buildHACalderaDiscovery () {
    // Select corresponding HAEntiny type
    HABinarySensor* haBEntity = new HABinarySensor ();
	
    uint8_t* msgPackBuffer;

    if (!haBEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    // *******************************
    // Add your characteristics here
    // There is no need to futher modify this function

    haBEntity->setNameSufix ("Caldera");
	haBEntity->setDeviceClass (bs_heat);
    haBEntity->addExpiration (3600);
	haBEntity->setPayloadOff ("OFF");
    haBEntity->setPayloadOn ("ON");
    haBEntity->setValueField ("rly");  // nombre del json del valor a capurar 
    //haEntity->setValueTemplate ("{%if value_json.dp==2-%}{{value_json.temp}}{%-else-%}{{states('sensor.***_temp')}}{%-endif%}");

    // *******************************

    size_t bufferLen = haBEntity->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haBEntity->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_INFO ("Resulting MSG pack length: %d", len);

    if (!sendHADiscovery (msgPackBuffer, len)) {
        DEBUG_WARN ("Error sending HA discovery message");
    }

    if (haBEntity) {
        delete (haBEntity);
    }

    if (msgPackBuffer) {
        free (msgPackBuffer);
    }
		
}
void CONTROLLER_CLASS_NAME::buildHAPelletDiscovery () {
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

    haEntity->setNameSufix ("Pellt");
    haEntity->setDeviceClass (sensor_humidity);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("%");
    haEntity->setValueField ("Pellets");  // nombre del json del valor a capurar 
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
/*void CONTROLLER_CLASS_NAME::buildHATriggerDiscovery () {
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
}*/
#endif // SUPPORT_HA_DISCOVERY
