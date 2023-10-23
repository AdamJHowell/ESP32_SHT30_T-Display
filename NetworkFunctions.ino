/*
 * The purpose of this file is to hold network-related functions which are device-agnostic.
 * Ideally, this file could be used by an ESP32, an ESP8266, or an Arduino Uno with WiFi.
 * Because memory capacity varies wildly from device to device, buffer sizes are declared as variables in the entry-point file.
 */


/*
 * onReceiveCallback() is a callback function to process MQTT subscriptions.
 * When a message comes in on a topic the MQTT client has subscribed to, this message is called.
 */
void onReceiveCallback2( char *topic, byte *payload, unsigned int length )
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
	// Add the null terminator.
	str[i] = 0;
	StaticJsonDocument<JSON_DOC_SIZE> callbackJsonDoc;
	deserializeJson( callbackJsonDoc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, or publishStatus.
	const char *command = callbackJsonDoc["command"];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = callbackJsonDoc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishInterval = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishInterval );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStatus" ) == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 *
 */
void configureOTA()
{
	Serial.println( "Configuring OTA." );

#ifdef ESP8266
	// The ESP8266 hostname defaults to esp8266-[ChipID]
	// The ESP8266 port defaults to 8266
	// ArduinoOTA.setPort( 8266 );
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( ( const char * )"admin" );
#elif ESP32
	// The ESP32 hostname defaults to esp32-[MAC]
	// The ESP32 port defaults to 3232
	// ArduinoOTA.setPort( 3232 );
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( "admin" );
	// Password can be set with it's md5 value as well
	// MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );
#else
	// ToDo: Verify how stock Arduino code is meant to handle the port, username, and password.
#endif
	ArduinoOTA.setHostname( hostName );
	Serial.printf( "Using OTA hostname '%s'\n", hostName );

	String type = "filesystem"; // SPIFFS
	if( ArduinoOTA.getCommand() == U_FLASH )
		type = "sketch";

	// Configure the OTA callbacks.
	ArduinoOTA.onStart( []()
							  {
		String type = "flash";	// U_FLASH
		if( ArduinoOTA.getCommand() == U_SPIFFS )
			type = "filesystem";
		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.printf( "OTA is updating the %s\n", type ); } );
	ArduinoOTA.onEnd( []()
							{ Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total )
								  { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error )
							  {
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );

	// Start listening for OTA commands.
	ArduinoOTA.begin();

	Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.


/*
 * wifiMultiConnect() will iterate through 'wifiSsidArray[]', attempting to connect with the password stored at the same index in 'wifiPassArray[]'.
 *
 */
void wifiMultiConnect()
{
	Serial.println( "\nEntering wifiMultiConnect()" );
	for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
	{
		// Get the details for this connection attempt.
		const char *wifiSsid = wifiSsidArray[networkArrayIndex];
		const char *wifiPassword = wifiPassArray[networkArrayIndex];

		// Announce the WiFi parameters for this connection attempt.
		Serial.print( "Attempting to connect to SSID \"" );
		Serial.print( wifiSsid );
		Serial.println( "\"" );

		// Don't even try to connect if the SSID cannot be found.
		if( checkForSSID( wifiSsid ) )
		{
			// Attempt to connect to this WiFi network.
			Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
			if( WiFi.setHostname( hostName ) )
				Serial.printf( "Network hostname set to '%s'\n", hostName );
			else
				Serial.printf( "Failed to set the network hostname to '%s'\n", hostName );
			WiFi.begin( wifiSsid, wifiPassword );

			unsigned long wifiConnectionStartTime = millis();
			Serial.printf( "Waiting up to %lu seconds for a connection.\n", wifiConnectionTimeout / 1000 );
			/*
			WiFi.status() return values:
			WL_IDLE_STATUS      = 0,
			WL_NO_SSID_AVAIL    = 1,
			WL_SCAN_COMPLETED   = 2,
			WL_CONNECTED        = 3,
			WL_CONNECT_FAILED   = 4,
			WL_CONNECTION_LOST  = 5,
			WL_WRONG_PASSWORD   = 6,
			WL_DISCONNECTED     = 7
			*/
			while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
			{
				Serial.print( "." );
				delay( 1000 );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				// Set the global 'networkIndex' to the index which successfully connected.
				networkIndex = networkArrayIndex;
				// Print that WiFi has connected.
				Serial.println( "\nWiFi connection established!" );
				snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
				Serial.printf( "IP address: %s", ipAddress );
				return;
			}
			else
				Serial.println( "Unable to connect to WiFi!" );
		}
		else
			Serial.println( "That network was not found!" );
	}
	Serial.println( "Exiting wifiMultiConnect()\n" );
} // End of wifiMultiConnect() function.


/*
 * checkForSSID() is used by wifiMultiConnect() to avoid attempting to connect to SSIDs which are not in range.
 * Returns 1 if 'ssidName' can be found.
 * Returns 0 if 'ssidName' cannot be found.
 */
int checkForSSID( const char *ssidName )
{
	byte networkCount = WiFi.scanNetworks();
	if( networkCount == 0 )
		Serial.println( "No WiFi SSIDs are in range!" );
	else
	{
		Serial.printf( "%d networks found.\n", networkCount );
		for( int i = 0; i < networkCount; ++i )
		{
			// Check to see if this SSID matches the parameter.
			if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
				return 1;
			// Alternately, the String compareTo() function can be used like this: if( WiFi.SSID( i ).compareTo( ssidName ) == 0 )
		}
	}
	Serial.printf( "SSID '%s' was not found!\n", ssidName );
	return 0;
} // End of checkForSSID() function.


/*
 * mqttMultiConnect() will:
 * 1. Check the WiFi connection, and reconnect WiFi as needed.
 * 2. Attempt to connect the MQTT client designated in 'mqttBrokerArray[networkIndex]' up to 'maxAttempts' number of times.
 * 3. Subscribe to the topic defined in 'commandTopic'.
 * If the broker connection cannot be made, an error will be printed to the serial port.
 */
bool mqttMultiConnect( int maxAttempts )
{
	Serial.println( "\nFunction mqttMultiConnect() has initiated." );
	if( WiFi.status() != WL_CONNECTED )
		wifiMultiConnect();

	/*
	 * The networkIndex variable is initialized to 2112.
	 * If it is still 2112 at this point, then WiFi failed to connect.
	 * This is only needed to display the name and port of the broker being used.
	 */
	if( networkIndex != 2112 )
		Serial.printf( "Attempting to connect to the MQTT broker at '%s:%d' up to %d times.\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex], maxAttempts );
	else
		Serial.printf( "Attempting to connect to the MQTT broker up to %d times.\n", maxAttempts );


	int attemptNumber = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && attemptNumber < maxAttempts )
	{
		// Put the macAddress and random number into clientId.
		char clientId[22];
		snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ) );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		Serial.printf( "Connecting with client ID '%s'.\n", clientId );
		Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ) );
		if( mqttClient.connect( clientId ) )
		{
			Serial.println( " connected." );
			if( !mqttClient.setBufferSize( JSON_DOC_SIZE ) )
			{
				Serial.printf( "Unable to create a buffer %d bytes long!\n", JSON_DOC_SIZE );
				Serial.println( "Restarting the device!" );
				ESP.restart();
			}
			//publishStats();
			// Subscribe to the command topic.
			if( mqttClient.subscribe( commandTopic ) )
				Serial.printf( "Successfully subscribed to topic '%s'.\n", commandTopic );
			else
				Serial.printf( "Failed to subscribe to topic '%s'!\n", commandTopic );
		}
		else
		{
			int mqttState = mqttClient.state();
			/*
				Possible values for client.state():
				MQTT_CONNECTION_TIMEOUT     -4		// Note: This also comes up when the clientID is already in use.
				MQTT_CONNECTION_LOST        -3
				MQTT_CONNECT_FAILED         -2
				MQTT_DISCONNECTED           -1
				MQTT_CONNECTED               0
				MQTT_CONNECT_BAD_PROTOCOL    1
				MQTT_CONNECT_BAD_CLIENT_ID   2
				MQTT_CONNECT_UNAVAILABLE     3
				MQTT_CONNECT_BAD_CREDENTIALS 4
				MQTT_CONNECT_UNAUTHORIZED    5
			*/
			Serial.printf( " failed!  Return code: %d", mqttState );
			if( mqttState == -4 )
				Serial.println( " - MQTT_CONNECTION_TIMEOUT" );
			else if( mqttState == 2 )
				Serial.println( " - MQTT_CONNECT_BAD_CLIENT_ID" );
			else
				Serial.println( "" );

			Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectInterval / 1000 );
			delay( mqttReconnectInterval );
		}
		attemptNumber++;
	}

	if( !mqttClient.connected() )
	{
		Serial.println( "Unable to connect to the MQTT broker!" );
		return false;
	}

	Serial.println( "Function mqttMultiConnect() has completed.\n" );
	return true;
} // End of mqttMultiConnect() function.
