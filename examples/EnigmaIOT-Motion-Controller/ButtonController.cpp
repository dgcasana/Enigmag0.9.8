// 
// 
// 

#include <functional>
#include "ButtonController.h"

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

    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHADiscovery, this));

    EnigmaIOTjsonController::connectInform ();
    // Add more actions here if needed
    // Keep this method duration short
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;
	// You do node setup here. Use it as it was the normal setup() Arduino function
    pinMode (BUTTON_PIN, INPUT);//_PULLUP);

	// Send a 'hello' message when initalizing is finished
	sendStartAnouncement ();

	DEBUG_DBG ("Finish begin");

	// If your node should sleep after sending data do all remaining tasks here
}

void CONTROLLER_CLASS_NAME::loop () {

	// If your node stays allways awake do your periodic task here

	if (pushReleased) { // Enter this only if button were not pushed in the last loop
		if (!digitalRead (BUTTON_PIN)) {
			delay (50); // debounce button push
			if (!digitalRead (BUTTON_PIN)) {
				DEBUG_WARN ("Motion detected!");
				pushTriggered = true; // Button is pushed
				pushReleased = false; // Mark button as not released
			}
		}
	}

	if (pushTriggered) { // If button was pushed
		pushTriggered = false; // Disable push trigger
		const size_t capacity = JSON_OBJECT_SIZE (2);
		DynamicJsonDocument json (capacity);
		json["sensor"] = BUTTON_PIN;
		json["motion"] = 0;
		if (sendJson (json)) {
			DEBUG_WARN ("Motion sent");
		} else {
			DEBUG_ERROR ("Motion send error");
		}
	}

	if (!pushReleased) {
		if (digitalRead (BUTTON_PIN)) { // If button is released
			DEBUG_WARN ("No motion");
			pushReleased = true;
			const size_t capacity = JSON_OBJECT_SIZE (2);
			DynamicJsonDocument json (capacity);
			json["sensor"] = BUTTON_PIN;
			json["motion"] = 1;
			if (sendJson (json)) {
				DEBUG_WARN ("No motion sent");
			} else {
				DEBUG_ERROR ("No-motion send error");
			}
		}
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

void CONTROLLER_CLASS_NAME::buildHADiscovery () {
    HATrigger* haEntity = new HATrigger ();

    uint8_t* msgPackBuffer;

    if (!haEntity) {
        DEBUG_WARN ("JSON object instance does not exist");
        return;
    }

    haEntity->setType (button_short_press);
    haEntity->setSubtype (button_1);

    size_t bufferLen = haEntity->measureMessage ();

    msgPackBuffer = (uint8_t*)malloc (bufferLen);

    size_t len = haEntity->getAnounceMessage (bufferLen, msgPackBuffer);

    DEBUG_WARN ("Resulting MSG pack length: %d", len);

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
