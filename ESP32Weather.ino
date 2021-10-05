/*
 * This sketch is a branc of my PubSubWeather sketch.
 * This sketch will use a HT30 sensor (0x44) to show temperature and humidity.
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 */
#include <TFT_eSPI.h>				// This header is included in https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#include <SPI.h>						// This header is added to the IDE libraries after the ESP32 is added in board manager.
#include "WiFi.h"						// This header is added to the IDE libraries after the ESP32 is added in board manager.
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include "esp_adc_cal.h"			// This header is added to the IDE libraries after the ESP32 is added in board manager.
#include "bmp.h"						// Created at http://www.rinkydinkelectronics.com/t_imageconverter565.php
#include "ClosedCube_SHT31D.h"	// This header is used to read from the HT30 sensor.  https://github.com/closedcube/ClosedCube_SHT31D_Arduino
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.

#define ADC_EN					14  // ADC_EN is the ADC detection enable port.
#define ADC_PIN				34

/**
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
//const char* wifiSsid = "yourSSID";				// Typically kept in "privateInfo.h".
//const char* wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char* mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;							// Typically kept in "privateInfo.h".
const char* mqttTopic = "ajhWeather";
char clientAddress[16];
char macAddress[18];
// Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
// ThingSpeak variables
unsigned long myChannelNumber = 1;
//const char* myWriteAPIKey = "yourWriteKey";	// Typically kept in "privateInfo.h".
String ht30SerialNumber = "";

// Create class objects.
WiFiClient espClient;
PubSubClient mqttClient( espClient );		// MQTT client.
TFT_eSPI tft = TFT_eSPI( 135, 240 );		// Graphics library.
ClosedCube_SHT31D sht3xd;						// SH30 library.
int loopCount = 0;
int vref = 1100;


// This function puts the ESP into shallow sleep, which saves power compared to the traditional delay().
void espDelay( int ms )
{
	esp_sleep_enable_timer_wakeup( ms * 1000 );
	esp_sleep_pd_config( ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON );
	esp_light_sleep_start();
}


float getVoltage()
{
	uint16_t adcValue = analogRead( ADC_PIN );
	return ( ( float ) adcValue / 4095.0 ) * 2.0 * 3.3 * ( vref / 1000.0 );
} // End of getVoltage()


void printResult( String text, float temperature, float humidity, String voltage )
{
	tft.fillScreen( TFT_BLACK );
	tft.setTextDatum( MC_DATUM );

	// DrawString cannot print a float, so it needs to be inserted into a String.
	String tempBuffer;
	tempBuffer += F( "Temp : " );
	tempBuffer += String( temperature );
	// This display does not handle the degree symbol well.
	tempBuffer += F( "°C");

	String humidityBuffer;
	humidityBuffer += F( "Humidity : " );
	humidityBuffer += String( humidity );
	humidityBuffer += F( "%");

	// Draw this line 16 pixels above middle.
	tft.drawString( tempBuffer,  tft.width() / 2, tft.height() / 2 - 16 );

	// Draw this line centered vertically and horizontally.
	tft.drawString( humidityBuffer,  tft.width() / 2, tft.height() / 2 );

	// Draw this line 16 pixels below middle.
	tft.drawString( voltage,	tft.width() / 2, tft.height() / 2 + 16 );
} // End of printResult()


/**
 * mqttConnect() will attempt to (re)connect the MQTT client.
 */
void mqttConnect()
{
	// Loop until MQTT has connected.
	while( !mqttClient.connected() )
	{
		Serial.print( "Attempting MQTT connection..." );
		if( mqttClient.connect( "ESP32 Client" ) ) // Attempt to mqttConnect using the designated clientID.
		{
			Serial.println( "connected!" );
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 2 seconds" );
			// Wait 2 seconds before retrying.
			delay( 2000 );
		}
	}
	Serial.println( "MQTT is connected!\n" );
} // End of mqttConnect() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	delay( 10 );
	Serial.println( '\n' );
	Wire.begin();
	sht3xd.begin( 0x44 ); // I2C address: 0x44 or 0x45

	ht30SerialNumber = sht3xd.readSerialNumber();

	Serial.print( "Serial #" );
	Serial.println( ht30SerialNumber );
	if( sht3xd.periodicStart( SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ ) != SHT3XD_NO_ERROR )
		Serial.println( "[ERROR] Cannot start periodic mode" );

	/*
	ADC_EN is the ADC detection enable port.
	If the USB port is used for power supply, it is turned on by default.
	If it is powered by battery, it needs to be set to high level.
	*/
	pinMode( ADC_EN, OUTPUT );
	digitalWrite( ADC_EN, HIGH );

	tft.init();
	tft.setRotation( 1 );
	tft.fillScreen( TFT_BLACK );
	tft.setTextSize( 2 );
	tft.setTextColor( TFT_GREEN );
	tft.setCursor( 0, 0 );
	tft.setTextDatum( MC_DATUM );
	tft.setTextSize( 1 );

	tft.setSwapBytes( true );
	tft.pushImage( 0, 0,	240, 135, daughters );
	delay( 5000 );
	tft.setRotation( 0 );
	tft.fillScreen( TFT_BLACK );
	tft.setTextDatum( MC_DATUM );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 0;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED )
	{
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( ++i );
		Serial.println( " seconds" );
	}

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( clientAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( clientAddress );
} // End of setup() function.


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	loopCount++;

	float voltage = getVoltage();
	String voltageString = "Voltage: " + String( voltage ) + "V";

	Serial.println();
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect();
	}
	mqttClient.loop();

	// Get temperature and relative humidity from the SHT30 library.
	SHT31D result = sht3xd.periodicFetchData();
	if( result.error == SHT3XD_NO_ERROR )
	{
		// Temperature is always a floating point in Centigrade units. Relative humidity is a 32 bit integer in Pascal units.
		float temperature = result.t;	 				// Get temperature.
		float humidity = result.rh;			 		// Get relative humidity.

		// Print the results to the onboard TFT screen.
		printResult( "Periodic Mode", temperature, humidity, voltageString );

		// Prepare a String to hold the JSON.
		char mqttString[256];
		// Write the readings to the String in JSON format.
		snprintf( mqttString, 256, "{\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.1f,\n\t\"humidity\": %.1f,\n\t\"voltage\": %.2f\n}", macAddress, clientAddress, temperature, humidity, voltage );
		// Publish the JSON to the MQTT broker.
		mqttClient.publish( mqttTopic, mqttString );
		// Print the JSON to the Serial port.
		Serial.println( mqttString );
	}
	else
	{
		Serial.println( "\nUnable to read from the sensor!\n" );
	}


	Serial.println( "Pausing for 60 seconds..." );
	delay( 60000 );	// Wait for 60 seconds.
} // End of loop() function.
