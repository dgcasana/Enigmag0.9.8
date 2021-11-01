// 
// 
// 

#include <functional>
#include "Puerta-Cochera.h"

using namespace std;
using namespace placeholders;

constexpr auto CONFIG_FILE = "/customconf.json"; ///< @brief Custom configuration file name

// -----------------------------------------
// You may add some global variables you need here,
// like serial port instances, I2C, etc
// -----------------------------------------

bool CONTROLLER_CLASS_NAME::processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	// Process incoming messages here
	// They are normally encoded as MsgPack so you can confert them to JSON very easily
	return true;
}


bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

void CONTROLLER_CLASS_NAME::connectInform () {

#if SUPPORT_HA_DISCOVERY    
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHADiscovery, this));
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;
    pinMode (CARRO_PIN,INPUT_PULLUP);
	// You do node setup here. Use it as it was the normal setup() Arduino function

	// Send a 'hello' message when initalizing is finished
	// Not needed as this will reboot after deep sleep
	// sendStartAnouncement (); 

	DEBUG_DBG ("Finish begin");

	// If your node should sleep after sending data do all remaining tasks here
}

void CONTROLLER_CLASS_NAME::loop () {

	// If your node stays allways awake do your periodic task here

	// You can send your data as JSON. This is a basic example

    bool puertaAbriendo;

    if (!carroPressSent && enigmaIotNode->isRegistered ()) {
        const size_t capacity = JSON_OBJECT_SIZE (2);
        DynamicJsonDocument json (capacity);
        
        puertaAbriendo = digitalRead(CARRO_PIN);
        json["Puerta"] = puertaAbriendo ? "Abriendo":"Cerrando";  // ¿Por Qué?

        if (sendJson (json)) {
            DEBUG_WARN ("Button press sent");
            carroPressSent = true;
        } else {
            DEBUG_WARN ("Error sending message");
        }
    }

        //const size_t capacity = JSON_OBJECT_SIZE (4);
		//DynamicJsonDocument json (capacity);
		//json["sensor"] = data_description;
		//json["meas"] = measurement;

		//sendJson (json);

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
void CONTROLLER_CLASS_NAME::buildHADiscovery () {
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

    haBEntity->setNameSufix ("Puerta");
	haBEntity->setDeviceClass (bs_garage_door);
    //haBEntity->addExpiration (3600);
	haBEntity->setPayloadOff ("Cerrando");
    haBEntity->setPayloadOn ("Abriendo");
    haBEntity->setValueField ("Puerta");  // nombre del json del valor a capurar 
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
#endif // SUPPORT_HA_DISCOVERY
