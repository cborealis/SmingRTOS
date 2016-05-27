/**
 * Basic Servo test application
 * A simplied servo demo based on the classical arduino Sweep
 *
 * v1.0 Robotiko
 *
*/

#include <user_config.h>
#include <SmingCore.h>
#include <libraries/Servo/ServoChannel.h>
#include <libraries/SparkFun_APDS9960/SparkFun_APDS9960.h>
#include <libraries/Adafruit_ST7735/Adafruit_ST7735.h>
#include "BPMDraw.h"

#define APDS9960_INT  12 // Needs to be an interrupt pin
#define STACK_SIZE 512

#define SERVO_PIN 15 // GPIO2

#define TFT_SCLK 	14
#define TFT_MOSI 	13
#define TFT_RST  	16
#define	TFT_DC   	0
#define TFT_CS   	2

#define WIFI_SSID "ti8m-IoT" // Put you SSID and Password here
#define WIFI_PWD "4abc625070c50c85fc36152c8d75e43c"

// ... and/or MQTT username and password
#define MQTT_USERNAME ""
#define MQTT_PWD ""

// ... and/or MQTT host and port
#define MQTT_HOST "10.10.0.156"
#define MQTT_PORT 1883

ServoChannel *servoChannel; //Servo instance

SparkFun_APDS9960 apds = SparkFun_APDS9960();

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

xTaskHandle xGestureTaskHandle = NULL;
xTaskHandle xDrawBitmapHandle = NULL;

boolean entry = true;

// Forward declarations
void startMqttClient();
void onMessageReceived(String topic, String message);

Timer procTimer;

void printWelcome()
{
	tft.setCursor(0, 0);
	tft.fillScreen(ST7735_WHITE);
	tft.setTextColor(ST7735_BLACK);
	tft.setTextSize(2);
	tft.println("Willkommenim");
	bmpDraw(tft, "ti8m_logo.bmp", 0,32);
//	tft.setTextSize(4);
//	tft.setTextColor(ST7735_BLUE);
//	tft.println("ti&m");
//	tft.setTextColor(ST7735_BLACK);
	tft.setCursor(0,82);
//	tft.setTextSize(2);
	tft.println("Parking");
	tft.setTextSize(0);
}

portTASK_FUNCTION( vDrawBitmapTask, pvParameters )
{
    for( ;; )
    {
    	vTaskSuspend( NULL );
    	if (entry) {
	    	bmpDraw(tft, "smiley.bmp", 0, 0);
//			tft.setCursor(0, 0);
//			tft.fillScreen(ST7735_WHITE);
//			tft.setTextColor(ST7735_BLACK);
//			tft.setTextSize(2);
//			tft.println("Ihr");
//			tft.setTextColor(ST7735_BLUE);
//			tft.println("Parkplatz");
//			tft.setTextColor(ST7735_BLACK);
//			tft.println("steht");
//			tft.println("bereit!");
//			tft.setTextSize(0);
    	} else {
	    	bmpDraw(tft, "smiley2.bmp", 0, 0);
//			tft.setCursor(0, 0);
//			tft.fillScreen(ST7735_WHITE);
//			tft.setTextColor(ST7735_BLACK);
//			tft.setTextSize(2);
//			tft.println("Auf");
//			tft.setTextColor(ST7735_BLUE);
//			tft.println("Wieder-");
//			tft.println("sehen!");
//			tft.setTextColor(ST7735_BLACK);
//			tft.setTextSize(0);
    	}
    }

    /* Tasks must not attempt to return from their implementing
    function or otherwise exit.  In newer FreeRTOS port
    attempting to do so will result in an configASSERT() being
    called if it is defined.  If it is necessary for a task to
    exit then have the task call vTaskDelete( NULL ) to ensure
    its exit is clean. */
    vTaskDelete( NULL );
}

void openBoom()
{
	servoChannel->setDegree(0); //Move the servo
	Serial.println("Opened Boom"); //Output current servo position
}

void closeBoom()
{
	servoChannel->setDegree(90); //Move the servo
	Serial.println("Closed Boom"); //Output current servo position
}

// MQTT client
// For quick check you can use: http://www.hivemq.com/demos/websocket-client/ (Connection= test.mosquitto.org:8080)
MqttClient mqtt(MQTT_HOST, MQTT_PORT, onMessageReceived);

// Check for MQTT Disconnection
void checkMQTTDisconnect(TcpClient& client, bool flag){

	// Called whenever MQTT connection is failed.
	tft.setTextColor(ST7735_RED);
	if (flag == true) {
		Serial.println("MQTT Broker Disconnected!!");
		tft.println("MQTT disconnected!");
	}
	else
	{
		Serial.println("MQTT Broker Unreachable!!");
		tft.println("MQTT unreachable!");
	}
	tft.setTextColor(ST7735_WHITE);

	// Restart connection attempt after few seconds
	procTimer.initializeMs(2 * 1000, startMqttClient).start(); // every 2 seconds
}

// Callback for messages, arrived from MQTT server
void onMessageReceived(String topic, String message)
{
	Serial.print(topic);
	Serial.print(":\r\n\t"); // Pretify alignment for printing
	Serial.println(message);
}

// Run MQTT client
void startMqttClient()
{
	procTimer.stop();
	tft.println("Starting MQTT");
	if(!mqtt.setWill("last/will","The connection from this device is lost:(", 1, true)) {
		debugf("Unable to set the last will and testament. Most probably there is not enough memory on the device.");
	}
	mqtt.connect("esp8266", MQTT_USERNAME, MQTT_PWD);
	// Assign a disconnect callback function
	mqtt.setCompleteDelegate(checkMQTTDisconnect);
	mqtt.setKeepAlive(5);
	mqtt.subscribe("main/status/#");
	mqtt.subscribe("boomcommand");

	mqtt.commandProcessing(true,"boomcommand","boomcmdreply");
	printWelcome();
}

void onStationGotIP(IPAddress ip, IPAddress mask, IPAddress gateway) {
	Serial.println("Got an IP!");
	Serial.println(ip.toString());
}

void onStationDisconnect(String, uint8_t, uint8_t[6], uint8_t) {
	Serial.println("Lost WLAN connection...");
	procTimer.stop();
}

// Will be called when WiFi station was connected to AP
void connectOk()
{
	Serial.println("I'm CONNECTED");
	tft.println("WIFI connected");
	// Run MQTT client
	startMqttClient();
}

// Will be called when WiFi station timeout was reached
void connectFail()
{
	Serial.println("I'm NOT CONNECTED. Need help :(");
	tft.setTextColor(ST7735_RED);
	tft.println("WIFI connection failed!");
	tft.setTextColor(ST7735_WHITE);

	WifiStation.waitConnection(connectOk, 40, connectFail); // We recommend 20+ seconds for connection timeout at start
}

void processOpenCommand(Command reqCommand, CommandOutput* commandOutput)
{
	debugf("OpenCommand entered");
	commandOutput->printf("Opening Boom...\r\n");
	openBoom();
}

void processOpenEntryCommand(Command reqCommand, CommandOutput* commandOutput)
{
	entry = true;
	vTaskResume( xDrawBitmapHandle);
	debugf("OpenEntryCommand entered");
	commandOutput->printf("Opening Boom...\r\n");
	openBoom();
}

void processOpenExitCommand(Command reqCommand, CommandOutput* commandOutput)
{
	entry = false;
	vTaskResume( xDrawBitmapHandle);
	debugf("OpenExitCommand entered");
	commandOutput->printf("Opening Boom...\r\n");
	openBoom();
}

void processCloseCommand(Command reqCommand, CommandOutput* commandOutput)
{
	printWelcome();
	debugf("CloseCommand entered");
	commandOutput->printf("Closing Boom...\r\n");
	closeBoom();
}

void IRAM_ATTR handleGesture() {
	while (apds.isGestureAvailable()) {
		switch (apds.readGesture()) {
		case DIR_UP:
			Serial.println("UP");
			break;
		case DIR_DOWN:
			Serial.println("DOWN");
			break;
		case DIR_LEFT:
			Serial.println("LEFT");
			printWelcome();
			closeBoom();
			break;
		case DIR_RIGHT:
			Serial.println("RIGHT");
			break;
		case DIR_NEAR:
			Serial.println("NEAR");
			break;
		case DIR_FAR:
			Serial.println("FAR");
			break;
		default:
			Serial.println("NONE");
		}
	}
}

portTASK_FUNCTION( vHandleGestureTask, pvParameters )
{
    for( ;; )
    {
    	vTaskSuspend( NULL );
    	handleGesture();
    }

    /* Tasks must not attempt to return from their implementing
    function or otherwise exit.  In newer FreeRTOS port
    attempting to do so will result in an configASSERT() being
    called if it is defined.  If it is necessary for a task to
    exit then have the task call vTaskDelete( NULL ) to ensure
    its exit is clean. */
    vTaskDelete( NULL );
}

void IRAM_ATTR gestureSensorISR() {
	/* Unblock the handling task so the task can perform any processing necessitated
	by the interrupt.  xHandlingTask is the task's handle, which was obtained
	when the task was created. */
	xTaskResumeFromISR( xGestureTaskHandle);
}

void init()
{
	Serial.begin(115200); // 115200 by default
	Serial.systemDebugOutput(true); // Debug output to serial

	Serial.println("Init Schranken Demo");
	System.setCpuFrequency(eCF_80MHz);

	spiffs_mount(); // Mount file system, in order to work with files

	tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab
	tft.fillScreen(ST7735_BLACK);

	servoChannel = new(ServoChannel); //Create the servo channel instance
	servoChannel->attach(SERVO_PIN); //attach the servo channel to the servo gpio

	tft.setCursor(0, 0);
	tft.setTextColor(ST7735_WHITE);
	tft.setTextWrap(true);
	tft.println("Starting...");

	closeBoom();
	tft.println("Closing boom");

	commandHandler.registerSystemCommands();
	commandHandler.registerCommand(CommandDelegate("open", "Opens Boom", "boom", CommandProcessDelegate(processOpenCommand)));
	commandHandler.registerCommand(CommandDelegate("close", "Closes Boom", "boom", CommandProcessDelegate(processCloseCommand)));
	commandHandler.registerCommand(CommandDelegate("enter", "Opens Boom for entry", "boom", CommandProcessDelegate(processOpenEntryCommand)));
	commandHandler.registerCommand(CommandDelegate("exit", "Opens Boom for exit", "boom", CommandProcessDelegate(processOpenExitCommand)));

	tft.println("Starting WIFI");

	WifiStation.config(WIFI_SSID, WIFI_PWD);
	WifiStation.enable(true);
	WifiAccessPoint.enable(false);

	WifiEvents.onStationGotIP(onStationGotIP);
	WifiEvents.onStationDisconnect(onStationDisconnect);

	// Run our method when station was connected to AP (or not connected)
	WifiStation.waitConnection(connectOk, 40, connectFail); // We recommend 20+ seconds for connection timeout at start

	Wire.pins(5,4);

	// Initialize APDS-9960 (configure I2C and initial values)
	if (apds.init()) {
		tft.println("Sensor init success");
		Serial.println("APDS-9960 initialization complete");
	} else {
		tft.setTextColor(ST7735_RED);
		tft.println("Sensor error!");
		tft.setTextColor(ST7735_WHITE);
		Serial.println("Something went wrong during APDS-9960 init!");
	}

	portBASE_TYPE xReturned;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
					vHandleGestureTask,       /* Function that implements the task. */
					(const signed char*)"GESTURES",          /* Text name for the task. */
					STACK_SIZE,      /* Stack size in words, not bytes. */
					nullptr,    /* Parameter passed into the task. */
					9,/* Priority at which the task is created. */
					&xGestureTaskHandle );      /* Used to pass out the created task's handle. */

	if( xReturned == pdPASS )
	{
		tft.println("Interrupt handler task started");
		// Initialize interrupt service routine
		pinMode(APDS9960_INT, (GPIO_INT_TYPE)GPIO_PIN_INTR_ANYEDGE);
		attachInterrupt(APDS9960_INT, gestureSensorISR, FALLING);

		// Start running the APDS-9960 gesture sensor engine
		if (apds.enableGestureSensor(true)) {
			tft.println("Gesture init success");
			Serial.println("Gesture sensor is now running");
		} else {
			tft.setTextColor(ST7735_RED);
			tft.println("Gesture init error!");
			tft.setTextColor(ST7735_WHITE);
			Serial.println("Something went wrong during gesture sensor init!");
		}
	}
	else
	{
		tft.setTextColor(ST7735_RED);
		tft.println("Interrupt task error!");
		tft.setTextColor(ST7735_WHITE);
		Serial.println("Task could not be started");
	}
	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
					vDrawBitmapTask,       /* Function that implements the task. */
					(const signed char*)"BITMAP",          /* Text name for the task. */
					STACK_SIZE,      /* Stack size in words, not bytes. */
					nullptr,    /* Parameter passed into the task. */
					tskIDLE_PRIORITY,/* Priority at which the task is created. */
					&xDrawBitmapHandle );      /* Used to pass out the created task's handle. */

	if( xReturned == pdPASS ) {
		tft.println("Bitmap painter task started");
	}
	else
	{
		tft.setTextColor(ST7735_RED);
		tft.println("Bitmap painter task error!");
		tft.setTextColor(ST7735_WHITE);
		Serial.println("Bitmap painter task could not be started");
	}
	// make sure the first interrupt gets cleared
	handleGesture();
}
