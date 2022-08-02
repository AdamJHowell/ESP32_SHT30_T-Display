/*
 * This sketch is a branch of my PubSubWeather sketch, modified for the TTGO T-Display ESP32.
 * This sketch will use a GT-HT30 sensor (SHT-30 compatible) on address 0x44 to show temperature and humidity.
 * https://usa.banggood.com/GT-HT30-Module-SHT30-High-Precision-Digital-Temperature-and-Humidity-Measurement-Sensor-Module-IIC-I2C-Interface-p-1879767.html
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include <TFT_eSPI.h>				// This header is included in https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#include "WiFi.h"						// This header is added to the IDE libraries after the ESP32 is added in board manager.
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include "esp_adc_cal.h"			// This header is added to the IDE libraries after the ESP32 is added in board manager.
#include "daughters.h"				// Created at http://www.rinkydinkelectronics.com/t_imageconverter565.php
#include "ClosedCube_SHT31D.h"	// This header is used to read from the HT30 sensor.  https://github.com/closedcube/ClosedCube_SHT31D_Arduino
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <ArduinoJson.h>			// https://arduinojson.org/
#include <ESPmDNS.h>					// OTA
#include <WiFiUdp.h>					// OTA
#include <ArduinoOTA.h>				// OTA

#define ADC_EN					14  	// ADC_EN is the Analog to Digital Converter detection enable port.
#define ADC_PIN				34

/**
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
//const char * wifiSsid = "yourSSID";					// Typically kept in "privateInfo.h".
//const char * wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char * mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;								// Typically kept in "privateInfo.h".

/*
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Typically declared in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Typically declared in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Typically declared in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Typically declared in "privateInfo.h".

const char * hostName = "T-Display_ESP32_SHT40_OTA";										// The hostname used for OTA access.
const char * notes = "Lillygo TFT with HT30 and OTA";										// Notes sent in the bulk publish.
const char * espControlTopic = "espControl";													// This is a topic we subscribe to, to get updates.
const char * commandTopic = "masterBedroom/tDisplay/command";							// The topic used to subscribe to update the configuration.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char * sketchTopic = "masterBedroom/tDisplay/sketch";								// The topic used to publish the sketch name.
const char * macTopic = "masterBedroom/tDisplay/mac";										// The topic used to publish the MAC address.
const char * ipTopic = "masterBedroom/tDisplay/ip";										// The topic used to publish the IP address.
const char * rssiTopic = "masterBedroom/tDisplay/rssi";									// The topic used to publish the WiFi Received Signal Strength Indicator.
const char * publishCountTopic = "masterBedroom/tDisplay/publishCount";				// The topic used to publish the loop count.
const char * notesTopic = "masterBedroom/tDisplay/notes";								// The topic used to publish notes relevant to this project.
const char * tempCTopic = "masterBedroom/tDisplay/sht40/tempC";						// The topic used to publish the temperature in Celsius.
const char * tempFTopic = "masterBedroom/tDisplay/sht40/tempF";						// The topic used to publish the temperature in Fahrenheit.
const char * humidityTopic = "masterBedroom/tDisplay/sht40/humidity";				// The topic used to publish the humidity.
const char * mqttStatsTopic = "espStats";														// The topic this device will publish to upon connection to the broker.
const char * mqttTopic = "espWeather";															// The topic used to publish a single JSON message containing all data.
const unsigned long JSON_DOC_SIZE = 1024;														// The ArduinoJson document size, and size of some buffers.
unsigned long publishInterval = 60000;															// The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long sensorPollInterval = 10000;														// The delay between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long mqttReconnectInterval = 5000;													// The time between MQTT connection attempts.
unsigned long wifiConnectionTimeout = 10000;													// The maximum amount of time in milliseconds to wait for a WiFi connection before trying a different SSID.
unsigned long lastPublishTime = 0;																// Stores the time of the last MQTT publish.
unsigned long lastPublish = 0;																	// This is used to determine the time since last MQTT publish.
unsigned long bootTime = 0;																		// Stores the time of the most recent boot.
unsigned long lastPollTime = 0;																	// Stores the time of the last sensor poll.
unsigned long publishCount = 0;																	// A count of how many publishes have taken place.
unsigned int networkIndex = 2112;																// An unsigned integer to hold the correct index for the network arrays: wifiSsidArray[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
char ipAddress[16];																					// The IPv4 address of the WiFi interface.
char macAddress[18];																					// The MAC address of the WiFi interface.
long rssi;																								// A global to hold the Received Signal Strength Indicator.
float tempC;																							// The sensor temperature in Celsius.
float tempF;																							// The sensor temperature in Fahrenheit.
float humidity;																						// The sensor relative humidity as a percetage.
float voltage;																							// This holds the calculated voltage.
int vref = 1100;																						// The number is used to tune for variances in the ADC.
String ht30SerialNumber = "";																		// Typically something like 927334746.

// Create class objects.
WiFiClient espClient;							// Network client.
PubSubClient mqttClient( espClient );		// MQTT client.
TFT_eSPI tft = TFT_eSPI( 135, 240 );		// Graphics library.
ClosedCube_SHT31D sht3xd;						// SH30 library.
SHT31D result;										// The struct which will hold sensor data.


void onReceiveCallback( char* topic, byte* payload, unsigned int length )
{
	char str[length + 1];
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	int i=0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char ) payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	str[i] = 0; // Null termination
	StaticJsonDocument <JSON_DOC_SIZE> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, changeSeaLevelPressure, or publishStatus.
	const char* command = doc["command"];
	if( strcmp( command, "publishTelemetry") == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor and immediately publish the readings.
		result = readTelemetry();
		if( result.error == SHT3XD_NO_ERROR )
		{
			publishTelemetry( result );
		}
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval") == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishInterval = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishInterval );
		lastPublish = 0;
	}
	else if( strcmp( command, "changeSeaLevelPressure") == 0 )
	{
		Serial.println( "Sea-level pressure is not implemented on the SHT series of sensors." );
	}
	else if( strcmp( command, "publishStatus") == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else if( strcmp( command, "pollSensor") == 0 )
	{
		Serial.println( "Polling the sensor and updating the display." );
		result = sht3xd.periodicFetchData();
		// Print the results to the onboard TFT screen.
		printResult( result.t, result.rh, getVoltage(), WiFi.RSSI() );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


// This function puts the ESP32 into light sleep, which saves power compared to the traditional delay().
void espDelay( int ms )
{
	esp_sleep_enable_timer_wakeup( ms * 1000 );
	esp_sleep_pd_config( ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON );
	esp_light_sleep_start();
} // End of espDelay() function.


// Get the voltage of the battery or the 5 volt pin of the USB connection.
float getVoltage()
{
	uint16_t adcValue = analogRead( ADC_PIN );
	return ( ( float ) adcValue / 4095.0 ) * 2.0 * 3.3 * ( vref / 1000.0 );
} // End of getVoltage() function.


// printResults will print very specific values to very specific locations.
// The first thing it does is black-out the screen, so previous information on screen is lost.
void printResult( float temperature, float humidity, float voltage, long rssi )
{
	// Black-out the screen to ensure no stale data interferes.
	tft.fillScreen( TFT_BLACK );
	// Set the middle center (MC) as the reference point.
	tft.setTextDatum( MC_DATUM );

	String voltageString = "Voltage: " + String( voltage ) + "V";

	// DrawString cannot print a float, so it needs to be inserted into a String.
	String tempBuffer;
	tempBuffer += F( "Temp : " );
	tempBuffer += String( temperature );
	// This font does not handle the degree symbol.
	tempBuffer += F( "*C");

	String humidityBuffer;
	humidityBuffer += F( "Humidity : " );
	humidityBuffer += String( humidity );
	humidityBuffer += F( "%");

	// Draw this line 48 pixels above middle.
	tft.drawString( macAddress,  tft.width() / 2, tft.height() / 2 - 48 );

	// Draw this line 32 pixels above middle.
	tft.drawString( ipAddress,  tft.width() / 2, tft.height() / 2 - 32 );

	// Draw this line 16 pixels above middle.
	tft.drawString( "S/N : " + String ( ht30SerialNumber ),	tft.width() / 2, tft.height() / 2 - 16 );

	// Draw this line centered vertically and horizontally.
	tft.drawString( tempBuffer,  tft.width() / 2, tft.height() / 2 );

	// Draw this line 16 pixels below middle.
	tft.drawString( humidityBuffer,  tft.width() / 2, tft.height() / 2 + 16 );

	// Draw this line 32 pixels below middle.
	tft.drawString( voltageString,	tft.width() / 2, tft.height() / 2 + 32 );

	String min = " minutes";
	if( publishCount == 1 )
		min = " minute";
	// Draw this line 48 pixels below middle.
	tft.drawString( String ( publishCount ) + min, tft.width() / 2, tft.height() / 2 + 48 );

	String dBm = "dBm";
	// Draw this line 64 pixels below middle.
	tft.drawString( String ( rssi ) + dBm, tft.width() / 2, tft.height() / 2 + 64 );
} // End of printResult() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	espDelay( 500 );
	// Start the Serial communication to send messages to the connected serial port.
	Serial.begin( 115200 );
	if( !Serial )
		espDelay( 1000 );
	Serial.println( '\n' );
	Serial.print( sketchName );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );
	Wire.begin();

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	Serial.println( "Initializing the HT30 sensor..." );
	// Initialize the HT30 seonsor.
	startSensor();

	/*
	ADC_EN is the Analog to Digital Converter detection enable port.
	If the USB port is used for power supply, it is turned on by default.
	If it is powered by battery, it needs to be set to high level.
	*/
	pinMode( ADC_EN, OUTPUT );
	digitalWrite( ADC_EN, HIGH );

	// Initialize the TFT screen.
	initTFT();

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );
	mqttClient.setCallback( onReceiveCallback );				 // Assign the onReceiveCallback() function to handle MQTT callbacks.

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	// Black-out the screen to ensure no stale data interferes.
	tft.fillScreen( TFT_BLACK );
	// Set the middle center (MC) as the reference point.
	tft.setTextDatum( MC_DATUM );

	result = sht3xd.periodicFetchData();
	// Print the results to the onboard TFT screen.
	printResult( result.t, result.rh, getVoltage(), WiFi.RSSI() );

	String logString = "Connecting to WiFi...";
	// Draw this line centered horizontally, and near the bottom of the screen.
	tft.drawString( logString,  tft.width() / 2, tft.height() / 2 + 96 );

	// Try to connect to the configured WiFi network, up to 10 times.
	wifiConnect( 10 );

	// Port defaults to 3232
	// ArduinoOTA.setPort( 3232 );

	// Hostname defaults to esp32-[MAC]
	// ArduinoOTA.setHostname( "myesp32" );

	// No authentication by default
	// ArduinoOTA.setPassword( "admin" );

	// Password can be set with it's md5 value as well
	// MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );

	ArduinoOTA
		.onStart( []()
		{
			String type;
			if( ArduinoOTA.getCommand() == U_FLASH )
				type = "sketch";
			else // U_SPIFFS
				type = "filesystem";

			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println( "Start updating " + type );
		} )
		.onEnd( []()
		{
			Serial.println( "\nEnd" );
		} )
		.onProgress( []( unsigned int progress, unsigned int total )
		{
			Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) );
		} )
		.onError( []( ota_error_t error )
		{
			Serial.printf( "Error[%u]: ", error );
			if( error == OTA_AUTH_ERROR )
				Serial.println( "Auth Failed" );
			else if( error == OTA_BEGIN_ERROR )
				Serial.println( "Begin Failed" );
			else if( error == OTA_CONNECT_ERROR )
				Serial.println( "Connect Failed" );
			else if( error == OTA_RECEIVE_ERROR )
				Serial.println( "Receive Failed" );
			else if( error == OTA_END_ERROR )
				Serial.println( "End Failed" );
		} );

	ArduinoOTA.begin();

	// Print the current values.
	result = sht3xd.periodicFetchData();
	// Print the results to the onboard TFT screen.
	printResult( result.t, result.rh, getVoltage(), WiFi.RSSI() );

	lastPublish = 0;
} // End of setup() function.


void startSensor()
{
	sht3xd.begin( 0x44 ); // I2C address: 0x44 or 0x45
	ht30SerialNumber = sht3xd.readSerialNumber();
	Serial.print( "Serial # " );
	Serial.println( ht30SerialNumber );
	// Start the HT30 sensor and check the return value.
	if( sht3xd.periodicStart( SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ ) != SHT3XD_NO_ERROR )
		Serial.println( "[ERROR] Cannot start periodic mode" );
} // End of startSensor() function.


void initTFT()
{
	// Initialize the TFT driver.
	tft.init();
	// Set the orientation where the antenna is on the left and the USB port is on the right.
	tft.setRotation( 1 );
	// Set the orientation where the antenna is on the right and the USB port is on the left.
	tft.setRotation( 3 );
	// Set the background to black.
	tft.fillScreen( TFT_BLACK );
	tft.setTextSize( 2 );
	// This color of green has good contrast on black.
	tft.setTextColor( TFT_GREEN );
	tft.setCursor( 0, 0 );
	// Set the refernce point to middle center (MC).
	tft.setTextDatum( MC_DATUM );
	tft.setTextSize( 1 );

	tft.setSwapBytes( true );
	tft.pushImage( 0, 0,	240, 135, daughters );
	espDelay( 5000 );
	// Set the orientation where the antenna is up and the USB port is down.
	tft.setRotation( 0 );
	tft.fillScreen( TFT_BLACK );
	tft.setTextDatum( MC_DATUM );
} // End of initTFT() function.


void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		espDelay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	WiFi.setAutoReconnect( true );
	WiFi.persistent( true );

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( ipAddress );
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
void mqttConnect( int maxAttempts )
{
	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( "connected!" );
			mqttClient.subscribe( espControlTopic );		// Subscribe to the designated MQTT topic.
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 5 seconds" );
			// Wait 5 seconds before retrying.
			espDelay( 5000 );
		}
		i++;
	}
	mqttClient.setBufferSize( JSON_DOC_SIZE );
} // End of mqttConnect() function.


SHT31D readTelemetry()
{
	return sht3xd.periodicFetchData();
}


void publishTelemetry( SHT31D result )
{
	// Get the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "WiFi RSSI: " );
	Serial.println( rssi );

	// Print the results to the onboard TFT screen.
	printResult( result.t, result.rh, getVoltage(), rssi );

	// Prepare a String to hold the JSON.
	char mqttString[JSON_DOC_SIZE];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, JSON_DOC_SIZE, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"serial\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"humidity\": %.1f,\n\t\"voltage\": %.2f,\n\t\"rssi\": %ld,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, ht30SerialNumber, result.t, result.rh, voltage, rssi, publishCount, notes );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
		Serial.println( "Successfully published this to the broker:" );
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublish = millis();
}


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	// Reconnect to WiFi if necessary.
	if( WiFi.status() != WL_CONNECTED )
	{
		String logString = "Connecting to WiFi...";
		// Draw this line centered horizontally, and near the bottom of the screen.
		tft.drawString( logString,  tft.width() / 2, tft.height() / 2 + 96 );
		wifiConnect( 10 );
		// Clear the line.
		tft.drawString( "                     ",  tft.width() / 2, tft.height() / 2 + 96 );
	}
	// Reconnect to the MQTT broker if necessary.
	if( !mqttClient.connected() )
	{
		String logString = "Connecting MQTT...";
		// Draw this line centered horizontally, and near the bottom of the screen.
		tft.drawString( logString,  tft.width() / 2, tft.height() / 2 + 96 );
		mqttConnect( 10 );
		// Clear the line.
		tft.drawString( "                                ",  tft.width() / 2, tft.height() / 2 + 96 );
	}
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	// ToDo: Move all this into a function, and call it from setup() and from loop().
	unsigned long time = millis();
	// When time is less than publishInterval, subtracting publishInterval from time causes an overlow which results in a very large number.
	if( ( time > publishInterval ) && ( time - publishInterval ) > lastPublish )
	{
		publishCount++;
		voltage = getVoltage();
		Serial.println( sketchName );
		Serial.print( "Connected to broker at \"" );
		Serial.print( mqttBroker );
		Serial.print( ":" );
		Serial.print( mqttPort );
		Serial.println( "\"" );
		Serial.print( "Listening for control messages on topic \"" );
		Serial.print( espControlTopic );
		Serial.println( "\"." );

		// Get temperature and relative humidity from the SHT30 library.
		result = readTelemetry();
		if( result.error == SHT3XD_NO_ERROR )
		{
			String logString = "Publishing telemetry...";
			// Draw this line centered horizontally, and near the bottom of the screen.
			tft.drawString( logString,  tft.width() / 2, tft.height() / 2 + 96 );
			publishTelemetry( result );
		}
		else
		{
			Serial.println( "\nUnable to read from the sensor!\n" );
		}
		// Clear the line.
		tft.drawString( "                       ",  tft.width() / 2, tft.height() / 2 + 96 );
		Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / 1000 );
	}
} // End of loop() function.
