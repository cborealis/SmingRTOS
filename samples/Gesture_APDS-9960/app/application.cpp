#include <user_config.h>
#include <SmingCore.h>

#include <libraries/SparkFun_APDS9960/SparkFun_APDS9960.h>
SparkFun_APDS9960 apds = SparkFun_APDS9960();

// For I2C
// Default I2C pins 0 and 2. Pin 4 - interrupt pin
//
#define APDS9960_INT  2 // Needs to be an interrupt pin
#define STACK_SIZE 512

xTaskHandle xGestureTaskHandle = NULL;

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

void init() {
	Serial.begin(SERIAL_BAUD_RATE); // 115200 by default
	Serial.systemDebugOutput(true); // Enable debug output to serial

	// WIFI not needed for demo. So disabling WIFI.
	WifiStation.enable(false);
	WifiAccessPoint.enable(false);

	Serial.println();
	Serial.println("--------------------------------");
	Serial.println("SparkFun APDS-9960 - GestureTest");
	Serial.println("--------------------------------");

	Wire.pins(5,4);

	// Initialize APDS-9960 (configure I2C and initial values)
	if (apds.init()) {
		Serial.println("APDS-9960 initialization complete");
	} else {
		Serial.println("Something went wrong during APDS-9960 init!");
	}

	// Start running the APDS-9960 gesture sensor engine
	if (apds.enableGestureSensor(true)) {
		Serial.println("Gesture sensor is now running");
	} else {
		Serial.println("Something went wrong during gesture sensor init!");
	}

	portBASE_TYPE xReturned;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
					vHandleGestureTask,       /* Function that implements the task. */
					(const signed char*)"GESTURES",          /* Text name for the task. */
					STACK_SIZE,      /* Stack size in words, not bytes. */
					nullptr,    /* Parameter passed into the task. */
					tskIDLE_PRIORITY,/* Priority at which the task is created. */
					&xGestureTaskHandle );      /* Used to pass out the created task's handle. */

	if( xReturned == pdPASS )
	{
		// Initialize interrupt service routine
		pinMode(APDS9960_INT, (GPIO_INT_TYPE)GPIO_PIN_INTR_ANYEDGE);
		attachInterrupt(APDS9960_INT, gestureSensorISR, FALLING);
	}
	else
	{
		Serial.println("Task could not be started");
	}
}

