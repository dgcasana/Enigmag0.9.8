// 
// 
// 

#include <functional>
#include "CompostController.h"

using namespace std;
using namespace placeholders;

#define TEST 0

constexpr auto CONFIG_FILE = "/customconf.json"; ///< @brief Custom configuration file name

// -----------------------------------------
// You may add some global variables you need here,
// like serial port instances, I2C, etc
// -----------------------------------------

#define ONE_WIRE_BUS 14 //(D5)  evita el gpio0 (D2)
 

float valueTH = 0;
AHT10 myAHT10 (AHT10_ADDRESS_0X38);


//ADC_MODE (ADC_VCC);


bool CONTROLLER_CLASS_NAME::processRxCommand (const uint8_t* address, const uint8_t* buffer, uint8_t length, nodeMessageType_t command, nodePayloadEncoding_t payloadEncoding) {
	// Process incoming messages here
	// They are normally encoded as MsgPack so you can confert them to JSON very easily
	return true;
}


bool CONTROLLER_CLASS_NAME::sendCommandResp (const char* command, bool result) {
	// Respond to command with a result: true if successful, false if failed 
	return true;
}

/*bool CONTROLLER_CLASS_NAME::sendTemperature (float temp) {
	const size_t capacity = JSON_OBJECT_SIZE (2);
	DynamicJsonDocument json (capacity);
	json["18b20"] = temp;

	return sendJson (json);
}*/

bool CONTROLLER_CLASS_NAME::sendTempHum (float temp, float tempe, float hum) {
	const size_t capacity = JSON_OBJECT_SIZE (4);
	DynamicJsonDocument json (capacity);
	json["TInt"] = temp;
    json["TExt"] = tempe;
	json["hum"] = hum;
    json["solar"] = (float)(analogRead(A0)) / 157; // R300+220k; 1023/6.5v
	

	return sendJson (json);
}

void CONTROLLER_CLASS_NAME::connectInform () {

#if SUPPORT_HA_DISCOVERY    
    // Register every HAEntity discovery function here. As many as you need
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHADs18b20Discovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAAht10TempDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHAAht10HumDiscovery, this));
    addHACall (std::bind (&CONTROLLER_CLASS_NAME::buildHASolarVDiscovery, this));
    
#endif

    EnigmaIOTjsonController::connectInform ();
}

void CONTROLLER_CLASS_NAME::setup (EnigmaIOTNodeClass* node, void* data) {
	enigmaIotNode = node;

	// You do node setup here. Use it as it was the normal setup() Arduino function

#if !TEST
	oneWire = new OneWire (ONE_WIRE_BUS);
	sensors = new DallasTemperature (oneWire);
	sensors->begin ();
	sensors->setWaitForConversion (false);
	sensors->requestTemperatures ();
#endif
    myAHT10.begin(D1, D2);
	//myHTU21D.begin(); // default SDA & SCL pin (D2,D1)
    time_t start = millis ();

    // Send a 'hello' message when initalizing is finished
    if (!enigmaIotNode->getNode ()->getSleepy ()) {
        if (!(enigmaIotNode->getNode ()->getSleepy ())) {
            sendStartAnouncement ();  // Disable this if node is sleepy
        }
    }

#if !TEST
	while (!sensors->isConversionComplete ()) {
		delay (0);
	}
	DEBUG_WARN ("Conversion completed in %d ms", millis () - start);
    tempC = sensors->getTempCByIndex (0);
#else
    tempC = 25.8;
#endif
	temperature = myAHT10.readTemperature(AHT10_FORCE_READ_DATA); //read 6-bytes over I2C
    humidity    = myAHT10.readHumidity(AHT10_USE_READ_DATA);      //use same 6-bytes
    
    // Send a 'hello' message when initalizing is finished
    //sendStartAnouncement ();

	DEBUG_DBG ("Finish begin");

      

	// If your node should sleep after sending data do all remaining tasks here
    
}

void CONTROLLER_CLASS_NAME::loop () {

	// If your node stays allways awake do your periodic task here

	// You can send your data as JSON. This is a basic example

       if (!tempSent && enigmaIotNode->isRegistered()) {
        /*if (sendTemperature (tempC)) {
            tempSent = true;
        }*/
        if (sendTempHum (tempC, temperature, humidity)) {
            tempSent = true;
        }
        // else {
        //}
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

void CONTROLLER_CLASS_NAME::buildHADs18b20Discovery () {
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

    haEntity->setNameSufix ("Ds18b20");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("TInt");  // nombre del json del valor a capurar
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
void CONTROLLER_CLASS_NAME::buildHAAht10TempDiscovery () {
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

    haEntity->setNameSufix ("Aht10");
    haEntity->setDeviceClass (sensor_temperature);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("ºC");
    haEntity->setValueField ("TExt");   
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
void CONTROLLER_CLASS_NAME::buildHAAht10HumDiscovery () {
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

    haEntity->setNameSufix ("Hum");
    haEntity->setDeviceClass (sensor_humidity);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("%");
    haEntity->setValueField ("hum");
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
void CONTROLLER_CLASS_NAME::buildHASolarVDiscovery () {
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

    haEntity->setNameSufix ("Solar_v");
    haEntity->setDeviceClass (sensor_voltage);
    haEntity->setExpireTime (3600);
    haEntity->setUnitOfMeasurement ("v");
    haEntity->setValueField ("solar");
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
