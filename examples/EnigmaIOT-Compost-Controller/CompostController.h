// BasicController.h

#ifndef _COMPOSTCONTROLLER_h
#define _COMPOSTCONTROLLER_h

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
#define CONTROLLER_CLASS_NAME CompostController
static const char* CONTROLLER_NAME = "Compost controller";
#if SUPPORT_HA_DISCOVERY    
#include <haSensor.h>
#endif

#include <DallasTemperature.h>
#include <HTU21D.h>
#include <Wire.h>
// --------------------------------------------------
// You may define data structures and constants here
// --------------------------------------------------


static const uint8_t PUERTA_MY_VERS[3] = { 1,0,1 }; ///< @brief Compost Version

class CONTROLLER_CLASS_NAME : EnigmaIOTjsonController {
protected:
	// --------------------------------------------------
	// add all parameters that your project needs here
	// --------------------------------------------------
	
	DallasTemperature* sensors;
    DeviceAddress insideThermometer;
    bool tempSent = false;
	//bool tempSent2 = false;
    float tempC;
	float   temperature = 0;
	float   humidity    = 0;


public:
	void setup (EnigmaIOTNodeClass* node, void* data = NULL);

	bool processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding);

	void loop ();

	~CONTROLLER_CLASS_NAME ();

	/**
	 * @brief Called when wifi manager starts config portal
	 * @param node Pointer to EnigmaIOT gateway instance
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

#if SUPPORT_HA_DISCOVERY    
    /**
     * @brief Sends a HA discovery message for a single entity. Add as many functions like this
     * as number of entities you need to create
     */
    void buildHADs18b20Discovery ();
	void buildHAAht10TempDiscovery ();
	void buildHAAht10HumDiscovery ();
	void buildHABattDiscovery ();
#endif
    
    // ------------------------------------------------------------
	// You may add additional method definitions that you need here
	// ------------------------------------------------------------

    //bool sendTemperature (float temp);
	bool sendTempHum (float tempC, float temp, float hum);

};

#endif

