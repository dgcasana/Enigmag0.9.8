// BasicController.h

#ifndef _BASICCONTROLLER_h
#define _BASICCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#define DEBUG_SERIAL

#include <EnigmaIOTjsonController.h>
#define CONTROLLER_CLASS_NAME SmartSwitchController
static const char* CONTROLLER_NAME = "Caldera controller";

#if SUPPORT_HA_DISCOVERY    
#include <haSensor.h>
#include <haBinarySensor.h>
#include <haSwitch.h>
#endif

//-------------------------------//
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewPing.h>

// --------------------------------------------------
// You may define data structures and constants here
// --------------------------------------------------

#define RELAY_PIN D1
#define ECHO_PIN     D2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN  D3  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define RESET_PIN D5 // You can set a different configuration reset pin here. Check for conflicts with used pins.
//#define PBAJA_PIN D5
#define ONE_WIRE_BUS D6
//#define PALTA_PIN D7

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define ON HIGH
#define OFF !ON

static const uint8_t CALDERA_MY_VERS[3] = { 2,3,2 }; ///< @brief Caldera Version

typedef enum {
	RELAY_OFF = 0,
	RELAY_ON = 1,
	SAVE_RELAY_STATUS = 2
} bootRelayStatus_t;


struct smartSwitchControllerHw_t {
	//int relayPin;
	bool relayStatus;
	bool bypass;
    bool pelletCtl;
	bootRelayStatus_t bootStatus;
    bootRelayStatus_t bypassStatus;
	int tParo;
    int tArranq;
	int ON_STATE;
	//int BY_STATE;
};


class CONTROLLER_CLASS_NAME : EnigmaIOTjsonController {
protected:
	// --------------------------------------------------
	// add all parameters that your project needs here
	// --------------------------------------------------

    smartSwitchControllerHw_t config;

public:
    /**
     * @brief Initializes controller structures
     * @param node Pointer to EnigmaIOT gateway instance
     * @param data Parameter data for controller
     */
	void setup (EnigmaIOTNodeClass* node, void* data = NULL);

    /**
     * @brief Processes received GET or SET commands
     * @param address Origin MAC address
     * @param buffer Command payload
     * @param length Payload length in bytes
     * @param command Command type. nodeMessageType_t::DOWNSTREAM_DATA_GET or nodeMessageType_t::DOWNSTREAM_DATA_SET
     * @param payloadEncoding Payload encoding. MSG_PACK is recommended
     */
	bool processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding);

    /**
     * @brief Executes repetitive tasks on controller
     */
	void loop ();

    /**
     * @brief Default destructor
     */
	~CONTROLLER_CLASS_NAME ();

	/**
	 * @brief Called when wifi manager starts config portal
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

    /**
     * @brief Executed as soon as node is registered on EnigmaIOT network
     */
    void connectInform ();

protected:
	/**
	  * @brief Saves output module configuration
	  * @return Returns `true` if save was successful. `false` otherwise
	  */
	bool saveConfig ();

    /**
     * @brief Send response to commands to gateway
     * @param command Refered command
     * @param result `true` if command was correctly executed, `false` otherwise
     */
	bool sendCommandResp (const char* command, bool result);

    /**
     * @brief Sends a notification message including configurable controller name and protocol version
     */
    bool sendStartAnouncement () {
        // You can send a 'hello' message when your node starts. Useful to detect unexpected reboot
        const size_t capacity = JSON_OBJECT_SIZE (10);
        DynamicJsonDocument json (capacity);
        json["status"] = "start";
        json["device"] = CONTROLLER_NAME;
        char version_buf[14];
        snprintf (version_buf, 14, "%d.%d.%d-%d.%d.%d",
                  CALDERA_MY_VERS[0], CALDERA_MY_VERS[1], CALDERA_MY_VERS[2], ENIGMAIOT_PROT_VERS[0], ENIGMAIOT_PROT_VERS[1], ENIGMAIOT_PROT_VERS[2]);
        json["version"] = String (version_buf);

        return sendJson (json);
    }

    /**
     * @brief Sends a HA discovery message for a single entity. Add as many functions like this
     * as number of entities you need to create
     */

    void buildHAPBajaDiscovery ();
	void buildHAPAltaDiscovery ();
	void buildHARetornoDiscovery ();
	void buildHAAcumulaDiscovery ();
    void buildHABypassDiscovery ();
	void buildHAPelletDiscovery ();
	void buildHACalderaDiscovery ();

	// ------------------------------------------------------------
	// You may add additional method definitions that you need here
	// ------------------------------------------------------------
    void defaultConfig ();
    
    void setRelay (bool state);

	bool sendRelayStatus ();

    void setBypass (bool state);

    void setParo (int temp);

    void setArran (int temp);

    void setMin (int temp);

	bool sendBypassStatus ();

    bool sendNodeStatus (); 

    bool sendInfoCommnads ();   
   
    void userCode();

	void arranque();

};

#endif


