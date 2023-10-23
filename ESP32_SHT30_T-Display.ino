/*
 * This sketch is a fork of my old PubSubWeather sketch, modified and modernized for the LilyGo T-Display (v 1.1) ESP32 devkit.
 * The devkit has a 1.14" ST7789V IPS display using GPIOs 19, 18, 5, 16, 23, and 4.
 * The battery management uses GPIO 14 as a reference voltage and GPIO 34 as an ADC to measure input voltage.
 * This sketch will use a GT-HT30 sensor (SHT-30 compatible) on address 0x44 to show temperature and humidity.
 * https://usa.banggood.com/GT-HT30-Module-SHT30-High-Precision-Digital-Temperature-and-Humidity-Measurement-Sensor-Module-IIC-I2C-Interface-p-1879767.html
 * The ESP-32 SDA pin is GPIO 21, and SCL is GPIO 22.
 * Arduino IDE settings: Board: ESP32 Dev Module, PSRAM: Disabled, Flash Size: 4MB (32Mb), all other settings at the default value.
 * @copyright   Copyright © 2023 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include <TFT_eSPI.h>			 // This header is included in https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#include "WiFi.h"					 // This header is added to the IDE libraries after the ESP32 is added in board manager.
#include <Wire.h>					 // This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include "esp_adc_cal.h"		 // Espressif's header for measuring input voltage.  This is added to the IDE libraries when the ESP32 is added in board manager.
#include "daughters.h"			 // Created at http://www.rinkydinkelectronics.com/t_imageconverter565.php
#include "ClosedCube_SHT31D.h" // This header is used to read from the HT30 sensor.  https://github.com/closedcube/ClosedCube_SHT31D_Arduino
#include <PubSubClient.h>		 // PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"		 // I use this file to hide my network information from random people browsing my GitHub repo.
#include <ArduinoJson.h>		 // https://arduinojson.org/
#include <ESPmDNS.h>				 // Multicast DNS.  This is added to the IDE libraries when the ESP32 is added in board manager.
#include <WiFiUdp.h>				 // OTA via Wi-Fi.  This is added to the IDE libraries when the ESP32 is added in board manager.
#include <ArduinoOTA.h>			 // OTA.  This is added to the IDE libraries when the ESP32 is added in board manager.

#define ADC_EN 14	 // ADC_EN is the GPIO used for a reference voltage.
#define ADC_PIN 34 // ADC_PIN is the GPIO used to take a voltage reading from.

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

// Device topic format: <location>/<device>/<metric>
// Sensor topic format: <location>/<device>/<sensor>/<metric>
const char *hostName = "T-Display_ESP32_HT30_OTA";						  // The hostname used for OTA access.
const char *notes = "LilyGo TFT with HT30 and OTA";						  // Notes sent in the bulk publish.
const char *espControlTopic = "espControl";									  // This is a topic we subscribe to, to get updates.
const char *commandTopic = "MasterBedroom/tDisplay/command";			  // The topic used to subscribe to update the configuration.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *sketchTopic = "MasterBedroom/tDisplay/sketch";				  // The topic used to publish the sketch name.
const char *macTopic = "MasterBedroom/tDisplay/mac";						  // The topic used to publish the MAC address.
const char *ipTopic = "MasterBedroom/tDisplay/ip";							  // The topic used to publish the IP address.
const char *rssiTopic = "MasterBedroom/tDisplay/rssi";					  // The topic used to publish the Wi-Fi Received Signal Strength Indicator.
const char *publishCountTopic = "MasterBedroom/tDisplay/publishCount"; // The topic used to publish the loop count.
const char *notesTopic = "MasterBedroom/tDisplay/notes";					  // The topic used to publish notes relevant to this project.
const char *tempCTopic = "MasterBedroom/tDisplay/sht30/tempC";			  // The topic used to publish the temperature in Celsius.
const char *tempFTopic = "MasterBedroom/tDisplay/sht30/tempF";			  // The topic used to publish the temperature in Fahrenheit.
const char *humidityTopic = "MasterBedroom/tDisplay/sht30/humidity";	  // The topic used to publish the humidity.
const unsigned long JSON_DOC_SIZE = 1024;										  // The ArduinoJson document size, and size of some buffers.
unsigned long publishInterval = 20000;											  // The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long sensorPollInterval = 5000;										  // The delay between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long mqttReconnectInterval = 5000;									  // The time between MQTT connection attempts.
unsigned long wifiConnectionTimeout = 10000;									  // The maximum amount of time in milliseconds to wait for a Wi-Fi connection before trying a different SSID.
unsigned long lastPublishTime = 0;												  // Stores the time of the last MQTT publish.
unsigned long bootTime = 0;														  // Stores the time of the most recent boot.
unsigned long lastPollTime = 0;													  // Stores the time of the last sensor poll.
unsigned long publishCount = 0;													  // A count of how many publishes have taken place.
unsigned int networkIndex = 2112;												  // An unsigned integer to hold the correct index for the network arrays: wifiSsidArray[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
char ipAddress[16];																	  // The IPv4 address of the Wi-Fi interface.
char macAddress[18];																	  // The MAC address of the Wi-Fi interface.
long rssi;																				  // A global to hold the Received Signal Strength Indicator.
float tempC;																			  // The sensor temperature in Celsius.
float tempF;																			  // The sensor temperature in Fahrenheit.
float humidity;																		  // The sensor relative humidity as a percentage.
float voltage;																			  // This holds the calculated voltage.
int referenceVoltage = 1100;														  // The number is used to tune for variances in the ADC.
String ht30SerialNumber = "";														  // Typically something like 927334746.

// Create class objects.
WiFiClient wiFiClient;						// Network client.
PubSubClient mqttClient( wiFiClient ); // MQTT client.
TFT_eSPI tft = TFT_eSPI( 135, 240 );	// Graphics library.
ClosedCube_SHT31D sht30;					// SH30 library.
SHT31D result;									// The struct which will hold sensor data.


void onReceiveCallback( char *topic, byte *payload, unsigned int length )
{
	char str[length + 1];
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	int i = 0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char )payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	str[i] = 0; // Null termination
	StaticJsonDocument<JSON_DOC_SIZE> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, changeSeaLevelPressure, or publishStatus.
	const char *command = doc["command"];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor and immediately publish the readings.
		readTelemetry();
		if( result.error == SHT3XD_NO_ERROR )
		{
			publishTelemetry();
		}
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mix-up.
		if( tempValue > 4000 )
			publishInterval = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishInterval );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "changeSeaLevelPressure" ) == 0 )
	{
		Serial.println( "Sea-level pressure is not implemented on the SHT series of sensors." );
	}
	else if( strcmp( command, "publishStatus" ) == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else if( strcmp( command, "pollSensor" ) == 0 )
	{
		Serial.println( "Polling the sensor and updating the display." );
		readTelemetry();
		// Print the results to the onboard TFT screen.
		printResult();
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
void getVoltage()
{
	uint16_t adcValue = analogRead( ADC_PIN );
	voltage = ( ( float )adcValue / 4095.0 ) * 2.0 * 3.3 * ( referenceVoltage / 1000.0 );
} // End of getVoltage() function.


// printResults will print very specific values to very specific locations.
// The first thing it does is black-out the screen, so previous information on screen is lost.
void printResult()
{
	// Black-out the screen to ensure no stale data interferes.
	tft.fillScreen( TFT_BLACK );
	// Set the middle center (MC) as the reference point.
	tft.setTextDatum( MC_DATUM );

	String voltageString = "Voltage: " + String( voltage ) + "V";

	// DrawString cannot print a float, so it needs to be inserted into a String.
	String tempBuffer;
	tempBuffer += F( "Temp : " );
	tempBuffer += String( tempC );
	// This font does not handle the degree symbol.
	tempBuffer += F( "*C" );

	String humidityBuffer;
	humidityBuffer += F( "Humidity : " );
	humidityBuffer += String( humidity );
	humidityBuffer += F( "%" );

	// Draw this line 48 pixels above middle.
	tft.drawString( macAddress, tft.width() / 2, tft.height() / 2 - 48 );

	// Draw this line 32 pixels above middle.
	tft.drawString( ipAddress, tft.width() / 2, tft.height() / 2 - 32 );

	// Draw this line 16 pixels above middle.
	tft.drawString( "S/N : " + String( ht30SerialNumber ), tft.width() / 2, tft.height() / 2 - 16 );

	// Draw this line centered vertically and horizontally.
	tft.drawString( tempBuffer, tft.width() / 2, tft.height() / 2 );

	// Draw this line 16 pixels below middle.
	tft.drawString( humidityBuffer, tft.width() / 2, tft.height() / 2 + 16 );

	// Draw this line 32 pixels below middle.
	tft.drawString( voltageString, tft.width() / 2, tft.height() / 2 + 32 );

	String min = " minutes";
	if( publishCount == 1 )
		min = " minute";
	// Draw this line 48 pixels below middle.
	tft.drawString( String( publishCount ) + min, tft.width() / 2, tft.height() / 2 + 48 );

	String dBm = "dBm";
	// Draw this line 64 pixels below middle.
	tft.drawString( String( rssi ) + dBm, tft.width() / 2, tft.height() / 2 + 64 );
} // End of printResult() function.


void startSensor()
{
	sht30.begin( 0x44 ); // I2C address: 0x44 or 0x45
	sht30.heaterDisable();
	ht30SerialNumber = sht30.readSerialNumber();
	Serial.print( "Serial # " );
	Serial.println( ht30SerialNumber );
	// Start the HT30 sensor and check the return value.
	if( sht30.periodicStart( SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ ) != SHT3XD_NO_ERROR )
		Serial.println( "[ERROR] Cannot start periodic mode" );
} // End of startSensor() function.


void initTFT()
{
	Serial.println( "Initializing the TFT..." );
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
	// Set the reference point to middle center (MC).
	tft.setTextDatum( MC_DATUM );
	tft.setTextSize( 1 );

	tft.setSwapBytes( true );
	Serial.println( "Displaying the image and pausing 5 seconds." );
	tft.pushImage( 0, 0, 240, 135, daughters );
	espDelay( 5000 );
	// Set the orientation where the antenna is up and the USB port is down.
	tft.setRotation( 0 );
	tft.fillScreen( TFT_BLACK );
	tft.setTextDatum( MC_DATUM );
	Serial.println( "TFT initialization complete." );
} // End of initTFT() function.


void wifiConnect( int maxAttempts )
{
	// Announce Wi-Fi parameters.
	String logString = "Wi-Fi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the Wi-Fi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	if( WiFi.setHostname( hostName ) )
		Serial.printf( "Network hostname set to '%s'\n", hostName );
	else
		Serial.printf( "Failed to set the network hostname to '%s'\n", hostName );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until Wi-Fi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	WiFi.setAutoReconnect( true );
	WiFi.persistent( true );

	// Print that Wi-Fi has connected.
	Serial.println( '\n' );
	Serial.println( "Wi-Fi connection established!" );
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
			mqttClient.subscribe( espControlTopic ); // Subscribe to the designated MQTT topic.
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


void readTelemetry()
{
	// Get the signal strength:
	rssi = WiFi.RSSI();
	result = sht30.periodicFetchData();
	tempC = result.t;
	tempF = ( tempC * 9 / 5 ) + 32;
	humidity = result.rh;
	getVoltage();
} // End of the readTelemetry() function.


void publishTelemetry()
{
	char buffer[20];
	if( mqttClient.publish( sketchTopic, __FILE__, false ) )
		Serial.printf( "  %s\n", sketchTopic );
	if( mqttClient.publish( macTopic, macAddress, false ) )
		Serial.printf( "  %s\n", macTopic );
	if( mqttClient.publish( ipTopic, ipAddress, false ) )
		Serial.printf( "  %s\n", ipTopic );
	if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
		Serial.printf( "  %s\n", rssiTopic );
	if( mqttClient.publish( publishCountTopic, ltoa( publishCount, buffer, 10 ), false ) )
		Serial.printf( "  %s\n", publishCountTopic );
	if( mqttClient.publish( notesTopic, notes, false ) )
		Serial.printf( "  %s\n", notesTopic );

	dtostrf( tempC, 1, 3, buffer );
	if( mqttClient.publish( tempCTopic, buffer, false ) )
		Serial.printf( "  %s\n", tempCTopic );
	dtostrf( tempF, 1, 3, buffer );
	if( mqttClient.publish( tempFTopic, buffer, false ) )
		Serial.printf( "  %s\n", tempFTopic );
	dtostrf( humidity, 1, 3, buffer );
	if( mqttClient.publish( humidityTopic, buffer, false ) )
		Serial.printf( "  %s\n", humidityTopic );
} // End of the publishTelemetry() function.


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
	Serial.println( __FILE__ );
	Serial.println( " is beginning its setup()." );
	Wire.begin();

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	Serial.println( "Initializing the HT30 sensor..." );
	// Initialize the HT30 sensor.
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
	mqttClient.setCallback( onReceiveCallback ); // Assign the onReceiveCallback() function to handle MQTT callbacks.

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	// Black-out the screen to ensure no stale data interferes.
	tft.fillScreen( TFT_BLACK );
	// Set the middle center (MC) as the reference point.
	tft.setTextDatum( MC_DATUM );

	readTelemetry();
	// Print the results to the onboard TFT screen.
	printResult();

	String logString = "Connecting to WiFi...";
	// Draw this line centered horizontally, and near the bottom of the screen.
	tft.drawString( logString, tft.width() / 2, tft.height() / 2 + 96 );

	// Try to connect to the configured Wi-Fi network, up to 10 times.
	wifiConnect( 10 );

	// Port defaults to 3232
	// ArduinoOTA.setPort( 3232 );

	// Hostname defaults to esp32-[MAC]
	ArduinoOTA.setHostname( hostName );
	Serial.printf( "Using OTA hostname '%s'\n", hostName );

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
						  Serial.println( "Start updating " + type ); } )
		 .onEnd( []()
					{ Serial.println( "\nEnd" ); } )
		 .onProgress( []( unsigned int progress, unsigned int total )
						  { Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) ); } )
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
							  Serial.println( "End Failed" ); } );

	ArduinoOTA.begin();

	// Print the current values.
	readTelemetry();
	// Print the results to the onboard TFT screen.
	printResult();
} // End of setup() function.


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	// Reconnect to Wi-Fi if necessary.
	if( WiFi.status() != WL_CONNECTED )
	{
		String logString = "Connecting to WiFi...";
		// Draw this line centered horizontally, and near the bottom of the screen.
		tft.drawString( logString, tft.width() / 2, tft.height() / 2 + 96 );
		wifiConnect( 3 );
		// Clear the line after wifiConnect finishes.
		tft.drawString( "                     ", tft.width() / 2, tft.height() / 2 + 96 );
	}
	// Reconnect to the MQTT broker if necessary.
	if( !mqttClient.connected() )
	{
		String logString = "Connecting MQTT...";
		// Draw this line centered horizontally, and near the bottom of the screen.
		tft.drawString( logString, tft.width() / 2, tft.height() / 2 + 96 );
		mqttConnect( 2 );
		// Clear the line after mqttConnect finishes.
		tft.drawString( "                                ", tft.width() / 2, tft.height() / 2 + 96 );
	}
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( time - sensorPollInterval ) > lastPollTime )
	{
		readTelemetry();
		printResult();
		Serial.printf( "Next telemetry poll in %lu seconds.\n\n", sensorPollInterval / 1000 );
		lastPollTime = millis();
	}

	time = millis();
	if( ( time > publishInterval ) && ( time - publishInterval ) > lastPublishTime )
	{
		// If the reading from the SHT30 library was valid.
		if( result.error == SHT3XD_NO_ERROR )
		{
			String logString = "Publishing telemetry...";
			// Draw this line centered horizontally, and near the bottom of the screen.
			tft.drawString( logString, tft.width() / 2, tft.height() / 2 + 96 );
			publishTelemetry();
			publishCount++;
		}
		else
		{
			Serial.println( "\nUnable to read from the sensor!\n" );
		}
		// Clear the line.
		tft.drawString( "                       ", tft.width() / 2, tft.height() / 2 + 96 );
		Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / 1000 );
		lastPublishTime = millis();
	}
} // End of loop() function.
