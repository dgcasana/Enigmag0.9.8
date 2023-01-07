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
const char* statusKey = "status";
const char* bypassKey = "bypass";
const char* pelletKey = "enPellet";
const char* infoKey = "info";
const char* tParoKey = "tParo";
const char* tArranqueKey = "tArranque";
const char* tMinimaKey = "tMinima";
const char* pbajaKey = "pbaja";
const char* paltaKey = "palta";


const time_t START_PERIOD = 7200000;  // 2 horas

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

int termosta;
unsigned long timeout;
DeviceAddress termRetorno   =  { 0x28, 0x20, 0x47, 0x75, 0xD0, 0x01, 0x3C, 0xF9 }; // {0x28, 0xE2, 0xE4, 0x95, 0xF0, 0x01, 0x3C, 0xA8};//
DeviceAddress termPBaja     =  { 0x28, 0xFF, 0x34, 0xB6, 0x51, 0x17, 0x04, 0x78 }; // {0x28, 0xBE, 0xF7, 0x95, 0xF0, 0x01, 0x3C, 0xC9}; //
DeviceAddress termPAlta     =  { 0x28, 0xFF, 0x95, 0x6C, 0x53, 0x17, 0x04, 0xD3 }; // {0x28, 0xCD, 0x22, 0x95, 0xF0, 0x01, 0x3C, 0x84}; //
DeviceAddress termAcumula   =  { 0x28, 0xFF, 0xA8, 0xDE, 0x9E, 0x20, 0xD6, 0xC6 }; // {0x28, 0xDB, 0x69, 0x95, 0xF0, 0x01, 0x3C, 0xF4}; //

bool unaVez = true;
const size_t capacity = JSON_OBJECT_SIZE (5);

int lastState = 0;//, tParo, tArran;
bool perTempo = true, perRelay = true;
static time_t lastStart;



float tempRetorno, tempPBaja, tempPAlta, tempAcumula, tempMin, nivelPellets;



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
		} else if (!strcmp (doc[commandKey], bypassKey)) {
			DEBUG_WARN ("Request link status. Link = %s", config.bypass ? "enabled" : "disabled");
			if (!sendBypassStatus ()) {
				DEBUG_WARN ("Error sending link status");
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


		} else if (!strcmp (doc[commandKey], bypassKey)) {
			if (!doc.containsKey (bypassKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set bypass status. Bypass = %s", doc[bypassKey].as<bool> () ? "enabled" : "disabled");

			setBypass (doc[bypassKey].as<bool> ());

			if (!sendBypassStatus ()) {
				DEBUG_WARN ("Error sending link status");
				return false;
			}

		} else if (!strcmp (doc[commandKey], pelletKey)) {
			if (!doc.containsKey (pelletKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set bypass status. Bypass = %s", doc[pelletKey].as<bool> () ? "enabled" : "disabled");

			config.pelletCtl = doc[pelletKey].as<bool> ();

			if (!sendPelletStatus ()) {
				DEBUG_WARN ("Error sending link status");
				return false;
			}

		}else if (!strcmp (doc[commandKey], tParoKey)) {
			if (!doc.containsKey (tParoKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set temp Paro. tParo = %i", doc[tParoKey].as<int> ());
			setParo(doc[tParoKey].as<int> ());
		
			//tParo = (doc[tParoKey].as<int> ());

		}else if (!strcmp (doc[commandKey], tArranqueKey)) {
			if (!doc.containsKey (tArranqueKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set temp Arranque. tArran = %i", doc[tArranqueKey].as<int> ());
			setArran(doc[tArranqueKey].as<int> ());

			//tArran = (doc[tArranqueKey].as<int> ());
		
		}else if (!strcmp (doc[commandKey], tMinimaKey)) {
			if (!doc.containsKey (tMinimaKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set temp Minima. tMin = %i", doc[tMinimaKey].as<int> ());
			setMin(doc[tMinimaKey].as<int> ());

			//tArran = (doc[tArranqueKey].as<int> ());
		} else if (!strcmp (doc[commandKey], pbajaKey)) {
			if (!doc.containsKey (pbajaKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set pabaja status. Bypass = %s", doc[pbajaKey].as<bool> () ? "enabled" : "disabled");

			setPbaja (doc[pbajaKey].as<bool> ());

			if (!sendPbajaStatus ()) {
				DEBUG_WARN ("Error sending link status");
				return false;
			}

		} else if (!strcmp (doc[commandKey], paltaKey)) {
			if (!doc.containsKey (paltaKey)) {
				DEBUG_WARN ("Wrong format");
				return false;
			}
			DEBUG_WARN ("Set palta status. Bypass = %s", doc[paltaKey].as<bool> () ? "enabled" : "disabled");

			setPalta (doc[paltaKey].as<bool> ());

			if (!sendPaltaStatus ()) {
				DEBUG_WARN ("Error sending link status");
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

bool CONTROLLER_CLASS_NAME::sendBypassStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);

	json[commandKey] = bypassKey;
    json[bypassKey] = config.bypass ? 1 : 0;

	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendPelletStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);

	json[commandKey] = pelletKey;
    json[pelletKey] = config.pelletCtl ? 1 : 0;

	return sendJson (json);
}

bool CONTROLLER_CLASS_NAME::sendNodeStatus () {
	const size_t capacity = JSON_OBJECT_SIZE (9);
	DynamicJsonDocument json (capacity);

	json["lastState"] = lastState;
    json[bypassKey] = config.bypass ? 1 : 0;
	json[pelletKey] = config.pelletCtl ? 1 : 0;
	json[relayKey] = config.relayStatus ? 1 : 0;
	json["tParo"] = config.tParo;
	json["tArranque"] = config.tArranq;
	json["tMinima"] = config.tMin;
	json["PerTempo"] = perTempo ? "Si" : "No";
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
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPBajaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPAltaDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHARetornoDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAAcumulaDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHABypassDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAPelletDiscovery, this));
	addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHACalderaDiscovery, this));
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;

    // You do node setup here. Use it as it was the normal setup() Arduino function

	pinMode (RELAY_PIN, OUTPUT);
	pinMode (PBAJA_PIN, OUTPUT);
	pinMode (PALTA_PIN, OUTPUT);
	sensors.begin();
	sensors.setResolution(termRetorno, 10);
	sensors.setResolution(termPBaja, 10);
	sensors.setResolution(termPAlta, 10);
	sensors.setResolution(termAcumula, 10);

    if (config.bootStatus != SAVE_RELAY_STATUS) {
		config.relayStatus = (bool)config.bootStatus;
		DEBUG_WARN ("Relay status set to Boot Status %d -> %d", config.bootStatus, config.relayStatus);
	}
	DEBUG_WARN ("Relay status set to %s", config.relayStatus ? "ON" : "OFF");
	
	setRelay(config.relayStatus);
	setPalta(config.palta);
	setPbaja(config.pbaja);

	//tArran = config.tArranq;
	//tParo = config.tParo;

	if (config.bypassStatus != SAVE_RELAY_STATUS) {
		config.bypass = (bool)config.bypassStatus;
		DEBUG_WARN ("Bypass status set to Bypass Status %d -> %d", config.bypassStatus, config.bypass);
	}
	DEBUG_WARN ("Bypass status set to %s", config.bypass ? "ON" : "OFF");

	if (config.pbajaStatus != SAVE_RELAY_STATUS) {
		config.pbaja = (bool)config.pbajaStatus;
		DEBUG_WARN ("pbaja status set to pbaja Status %d -> %d", config.pbajaStatus, config.bypass);
	}
	DEBUG_WARN ("pbaja status set to %s", config.pbaja ? "ON" : "OFF");

	if (config.paltaStatus != SAVE_RELAY_STATUS) {
		config.palta = (bool)config.paltaStatus;
		DEBUG_WARN ("palta status set to palta Status %d -> %d", config.paltaStatus, config.bypass);
	}
	DEBUG_WARN ("palta status set to %s", config.palta ? "ON" : "OFF");

    /// Send a 'hello' message when initalizing is finished
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

void CONTROLLER_CLASS_NAME::setBypass (bool state) {
	DEBUG_WARN ("Set bypass %s", state ? "ON" : "OFF");
	config.bypass = state;
	if (saveConfig ()) {
		DEBUG_WARN ("Config updated. bypass");
	} else {
		DEBUG_ERROR ("Error saving config");
	}
}

void CONTROLLER_CLASS_NAME::setPbaja (bool state) {
	DEBUG_WARN ("Set pbaja %s", state ? "ON" : "OFF");
	config.pbaja = state;
	digitalWrite (PBAJA_PIN, config.pbaja ? ON : OFF);

	if (config.pbajaStatus == SAVE_RELAY_STATUS) {
		if (saveConfig ()) {
			DEBUG_WARN ("Config updated. Relay is %s", config.pbaja ? "ON" : "OFF");
		} else {
			DEBUG_ERROR ("Error saving config");
		}
	}

}

void CONTROLLER_CLASS_NAME::setPalta (bool state) {
	DEBUG_WARN ("Set palta %s", state ? "ON" : "OFF");
	config.palta = state;
	digitalWrite (PALTA_PIN, config.palta ? ON : OFF);

	if (config.paltaStatus == SAVE_RELAY_STATUS) {
		if (saveConfig ()) {
			DEBUG_WARN ("Config updated. Relay is %s", config.palta ? "ON" : "OFF");
		} else {
			DEBUG_ERROR ("Error saving config");
		}
	}
	

}

void CONTROLLER_CLASS_NAME::setParo (int temp) {
	DEBUG_WARN ("Set tParo %d", temp);
	config.tParo = temp;
	if (saveConfig ()) {
		DEBUG_WARN ("Config updated. tParo");
	} else {
		DEBUG_ERROR ("Error saving config");
	}
}

void CONTROLLER_CLASS_NAME::setArran (int temp) {
	DEBUG_WARN ("Set tArranque %d", temp);
	config.tArranq = temp;
	if (saveConfig ()) {
		DEBUG_WARN ("Config updated. tArranque");
	} else {
		DEBUG_ERROR ("Error saving config");
	}
}

void CONTROLLER_CLASS_NAME::setMin (int temp) {
	DEBUG_WARN ("Set tMinima %d", temp);
	config.tMin = temp;
	if (saveConfig ()) {
		DEBUG_WARN ("Config updated. tMinima");
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
		tempRetorno = sensors.getTempC(termRetorno);
		tempPBaja = sensors.getTempC(termPBaja);
		tempPAlta = sensors.getTempC(termPAlta);
		tempAcumula = sensors.getTempC(termAcumula);
		//int lecturas=0,suma=0;
		int distancia;

		Serial.printf ("tempRetorno: %f\n", tempRetorno);
		Serial.printf ("tempPAlta: %f\n", tempPAlta);
		Serial.printf ("tempPBaja: %f\n", tempPBaja);
			
		// Nivel
		//msg.addAnalogInput (0, (float)(ESP.getVcc ()) / 1000);
		//msg.addTemperature (1, 20.34);

		//Serial.printf ("Vcc: %f\n", (float)(ESP.getVcc ()) / 1000);
		
		/*msg.addTemperature (8, tempRetorno);
		msg.addTemperature (9, tempPBaja);
		msg.addTemperature (10, tempPAlta);*/
		
		json["Retorno"] = tempRetorno;
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
			
		}

		distancia= sonar.ping_cm();

		if((distancia>3)&&(distancia<120)){
			Serial.println("Medida correcta,envia");
			nivelPellets =  (120-distancia) * 0.83; //map(distancia,3,120,100,0);
			
			/*msg.addAnalogInput(11, nivelPellets);
			msg.addDistance(35,distancia);*/
			
			
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
		json["Pellets"] = nivelPellets;
		json["termostato"] = termosta ? 1 : 0;  //influxdb no reconoce on/off

		
		sendJson(json);
		// End of user code
}

void CONTROLLER_CLASS_NAME::arranque(){
	int tParoVar;
	static int lastParoVar = config.tParo;

	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);
	
	

	if (tempPBaja - tempRetorno > 6) {
		tParoVar = config.tParo + 4; // Si el suelo esta frio aumento la histeresis
		if(lastParoVar!=tParoVar){
			json["tParoVar"]= tParoVar;
			sendJson(json);
			lastParoVar = tParoVar;
		}
	}
	else {
		tParoVar = config.tParo;
		if(lastParoVar!=tParoVar){
			json["tParoVar"]= tParoVar;
			sendJson(json);
			lastParoVar = tParoVar;
		}
	}
	
	
	if (((0 < tempAcumula)&&(tempAcumula < config.tArranq)) && ((nivelPellets > 30) || !config.pelletCtl)){  // config.pelletCtl deshabilita el control nivelPellets
		termosta = HIGH;
		if (perTempo || tempAcumula<config.tMin)  perRelay = true;
		if (config.bypass && perRelay && !lastState){
			setRelay(HIGH);
			Serial.println("Orden arranque caldera");

		}
	}	
	else if( tempAcumula > tParoVar) {
		termosta = LOW;
		if (lastState == HIGH){
			setRelay(LOW);
			lastStart = millis();
			perTempo = LOW;
			unaVez = true;
			perRelay = false;
			
			Serial.println("Desactivado 2 horas");
		}
	}

	if(termosta == HIGH && unaVez){
		unaVez = false;
		
		Serial.println("Demanda de caldera");
	}
	
	
	if (millis () - lastStart > START_PERIOD && !perTempo) {
		perTempo = HIGH;
		Serial.println("Arranque permitido por tiempo");
	}

	if (tempAcumula < 38){
		perTempo = HIGH;
		setBypass(1);
		Serial.println("Arranque activado por baja temp. acumulador");
	}

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
    static clock_t sendStatusPeriod = 30000;
    if (enigmaIotNode->isRegistered () && millis () - lastSentStatus > sendStatusPeriod) {
        lastSentStatus = millis ();
        sendStatusPeriod = 300000;
        sendRelayStatus ();
		//showTime ();
		userCode();
		arranque();
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

		config.bypass = true;
		config.ON_STATE = ON;
		config.bootStatus = SAVE_RELAY_STATUS;
		config.bypassStatus = SAVE_RELAY_STATUS;
		config.paltaStatus = SAVE_RELAY_STATUS;
		config.pbajaStatus = SAVE_RELAY_STATUS;
		config.tArranq = 48;
		config.tParo = 57;
		config.tMin = 39;
		config.palta = OFF;
		config.pbaja = OFF;
		
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
	config.bypass = true;
	config.pelletCtl = true;
	config.ON_STATE = ON;
	config.bootStatus = SAVE_RELAY_STATUS;
	config.bypassStatus = SAVE_RELAY_STATUS;
	config.pbajaStatus = SAVE_RELAY_STATUS;
	config.paltaStatus = SAVE_RELAY_STATUS;
	config.tArranq = 48;
	config.tParo = 57;
	config.tMin = 39;
	config.pbaja = OFF;
	config.palta = OFF;

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

			if (doc.containsKey ("bypass") &&
				doc.containsKey ("ON_STATE") &&
				doc.containsKey ("bypassStatus") &&
				doc.containsKey ("bootStatus")) {

				json_correct = true;
				//config.bypass = doc["linked"].as<bool> ();
				config.ON_STATE = doc["ON_STATE"].as<int> ();
				int bootStatus = doc["bootStatus"].as<int> ();
				int bypassStatus = doc["bypassStatus"].as<int> ();
				if (bootStatus >= RELAY_OFF && bootStatus <= SAVE_RELAY_STATUS) {
					config.bootStatus = (bootRelayStatus_t)bootStatus;
					DEBUG_WARN ("Boot status set to %d", config.bootStatus);
				} else {
					config.bootStatus = RELAY_OFF;
				}
				if (bypassStatus >= RELAY_OFF && bypassStatus <= SAVE_RELAY_STATUS) {
					config.bypassStatus = (bootRelayStatus_t)bypassStatus;
					DEBUG_WARN ("Bypass status set to %d", config.bypassStatus);
				} else {
					config.bypassStatus = RELAY_OFF;
				}
				
			}
			if (doc.containsKey ("pbaja") &&
				doc.containsKey ("palta") &&
				doc.containsKey ("pbajaStatus") &&
				doc.containsKey ("paltaStatus")) {

				json_correct = true;
				//config.bypass = doc["linked"].as<bool> ();
				int pbajaStatus = doc["pbajaStatus"].as<int> ();
				int paltaStatus = doc["paltaStatus"].as<int> ();
				if (pbajaStatus >= RELAY_OFF && pbajaStatus <= SAVE_RELAY_STATUS) {
					config.pbajaStatus = (bootRelayStatus_t)pbajaStatus;
					DEBUG_WARN ("planta baja status set to %d", config.pbajaStatus);
				} else {
					config.pbajaStatus = RELAY_OFF;
				}
				if (paltaStatus >= RELAY_OFF && paltaStatus <= SAVE_RELAY_STATUS) {
					config.paltaStatus = (bootRelayStatus_t)paltaStatus;
					DEBUG_WARN ("planta alta status set to %d", config.paltaStatus);
				} else {
					config.paltaStatus = RELAY_OFF;
				}
				
				
			}
			if (doc.containsKey ("tArranq")) {
				config.tArranq = doc["tArranq"].as<int> ();
			}

			if (doc.containsKey ("tParo")) {
				config.tParo = doc["tParo"].as<int> ();
			}

			if (doc.containsKey ("tMin")) {
				config.tMin = doc["tMin"].as<int> ();
			}

			if (doc.containsKey ("relayStatus")) {
				config.relayStatus = doc["relayStatus"].as<bool> ();
			}
			if (doc.containsKey ("bypass")) {
				config.bypass = doc["bypass"].as<bool> ();
			}
			if (doc.containsKey ("pelletCtl")) {
				config.pelletCtl = doc["pelletCtl"].as<bool> ();
			}

			configFile.close ();
			if (json_correct) {
				DEBUG_WARN ("Smart switch controller configuration successfuly read");
			} else {
				DEBUG_WARN ("Smart switch controller configuration error");
			}
			DEBUG_WARN ("==== Smart switch Controller Configuration ====");
			DEBUG_WARN ("Bypass: %s", config.bypass ? "true" : "false");
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

	doc["bypass"] = config.bypass;
	doc["pelletCtl"] = config.pelletCtl;
	doc["ON_STATE"] = config.ON_STATE;
	doc["relayStatus"] = config.relayStatus;
	int bootStatus = config.bootStatus;
	doc["bootStatus"] = bootStatus;
	int bypassStatus = config.bypassStatus;
	doc["bootStatus"] = bootStatus;
	int pbajaStatus = config.pbajaStatus;
	doc["pbajaStatus"] = pbajaStatus;
	int paltaStatus = config.paltaStatus;
	doc["paltaStatus"] = paltaStatus;
	doc["tArranq"] = config.tArranq;
	doc["tParo"] = config.tParo;
	doc["tMin"] = config.tMin;
	doc["palta"] = config.palta;
	doc["pbaja"] = config.pbaja;


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
void CONTROLLER_CLASS_NAME::buildHARetornoDiscovery () {
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

    haEntity->setNameSufix ("Retorno");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("Retorno");  // nombre del json del valor a capurar 
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
	haBEntity->setPayloadOff ("0");
    haBEntity->setPayloadOn ("1");
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
#endif // SUPPORT_HA_DISCOVERY
