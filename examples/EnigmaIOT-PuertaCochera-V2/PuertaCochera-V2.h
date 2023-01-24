// BasicController.h

#ifndef _BASICCONTROLLER_h
#define _BASICCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#define DEBUG_SERIAL

#ifdef ESP32
#include <SPIFFS.h>
#endif

#include <EnigmaIOTjsonController.h>
#define CONTROLLER_CLASS_NAME SmartSwitchController
static const char* CONTROLLER_NAME = "Puerta Cochera";

#if SUPPORT_HA_DISCOVERY    
#include <haTrigger.h>
#include <haSwitch.h>
#include <haBinarySensor.h>
#endif

// --------------------------------------------------
// You may define data structures and constants here
// --------------------------------------------------
#define DEFAULT_BUTTON_PIN 0
#define DEFAULT_RELAY_PIN 5  //D1 pin tipico del relay shield
#define DEFAULT_OPEN_PIN 12  //D6
#define DEFAULT_CLOSE_PIN 14  //D5
#define ON HIGH
#define OFF !ON

typedef enum {
	RELAY_OFF = 0,
	RELAY_ON = 1,
	SAVE_RELAY_STATUS = 2
} bootRelayStatus_t;

struct smartSwitchControllerHw_t {
	int relayPin;
	bool relayStatus;
	uint8_t closedPin;
	uint8_t openedPin;
	//bool linked;
	//bootRelayStatus_t bootStatus;
	int ON_STATE;
};

class CONTROLLER_CLASS_NAME : EnigmaIOTjsonController {
protected:
	// --------------------------------------------------
	// add all parameters that your project needs here
	// --------------------------------------------------
	bool pushTriggered = false;
	bool pushReleased = true;
	smartSwitchControllerHw_t config;
	AsyncWiFiManagerParameter* closedPinParam;
	AsyncWiFiManagerParameter* openedPinParam;
	AsyncWiFiManagerParameter* relayPinParam;
	

public:
	void setup (EnigmaIOTNodeClass* node, void* data = NULL);

	bool processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding);

	void loop ();

	~CONTROLLER_CLASS_NAME ();

	/**
	 * @brief Called when wifi manager starts config portal
	 * @param node< Pointer to EnigmaIOT gateway instance
	 */
	void configManagerStart ();

	/**
	 * @brief Called when wifi manager exits config portal
	 * @param status `true` if configuration was successful
	 */
	void configManagerExit (bool status);

	/**
	 * @brief Loads output module configuration
	 * @return Returns `true` if load was successful. `false` otherwise
	 */
	bool loadConfig ();

    void connectInform ();

protected:
	/**
	  * @brief Saves output module configuration
	  * @return Returns `true` if save was successful. `false` otherwise
	  */
	bool saveConfig ();

	bool sendCommandResp (const char* command, bool result);

    bool sendStartAnouncement () {
        // You can send a 'hello' message when your node starts. Useful to detect unexpected reboot
        const size_t capacity = JSON_OBJECT_SIZE (10);
        DynamicJsonDocument json (capacity);
        json["status"] = "start";
        json["device"] = CONTROLLER_NAME;
        char version_buf[10];
        snprintf (version_buf, 10, "%d.%d.%d",
                  ENIGMAIOT_PROT_VERS[0], ENIGMAIOT_PROT_VERS[1], ENIGMAIOT_PROT_VERS[2]);
        json["version"] = String (version_buf);

        return sendJson (json);
    }

    /**
     * @brief Sends a HA discovery message for a single entity. Add as many functions like this
     * as number of entities you need to create
     */
    void buildHASwitchDiscovery ();
    void buildHABinarySensorDiscovery ();
    //void buildHALinkDiscovery ();

	// ------------------------------------------------------------
	// You may add additional method definitions that you need here
	// ------------------------------------------------------------
	void defaultConfig ();

	//void toggleRelay ();

	void setRelay (bool state);

	bool sendRelayStatus ();

	//void setLinked (bool state);

	//bool sendLinkStatus ();

	//void setBoot (int state);

	bool sendBootStatus ();
};

#endif

