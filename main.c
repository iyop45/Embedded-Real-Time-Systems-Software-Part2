/**************************************************************************//**
 *
 * @file        Main.c
 * @brief       FreeRTOS Examples
 * @author      Geoffrey Daniels
 * @author		Jez Dalton and Sam Walder
 * @version     1.21 (GW)
 * @date        17/02/2015
 *
 * Copyright(C) 2012, Geoffrey Daniels, GPDaniels.com
 * Copyright(C) 2015, Jeremy Dalton, jd0185@my.bristol.ac.uk
 * All rights reserved.
 *
******************************************************************************/
/******************************************************************************
 * FreeRTOS includes.
 *****************************************************************************/
#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_Task.h"
#include "FreeRTOS_Queue.h"
#include "FreeRTOS_Timers.h"
#include "FreeRTOS_Semaphore.h"

/******************************************************************************
 * Library includes.
 *****************************************************************************/
#include "stdio.h"
#include "LPC17xx.h"
#include "LPC17xx_GPIO.h"

/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/
#define SOFTWARE_TIMER_PERIOD_MS (1000 / portTICK_RATE_MS)	// The timer period (1 second)
#define WAVPLAYER_INCLUDE_SAMPLESONGS						// Include the sample in WavPlayer_Sample.h
#define PutStringOLED PutStringOLED1						// Select which to use
//#define PutStringOLED PutStringOLED2						// Select which to use

/******************************************************************************
 * Library includes.
 *****************************************************************************/
#include "dfrobot.h"
#include "pca9532.h"
#include "joystick.h"
#include "OLED.h"
#include "WavPlayer.h"

extern const uint8_t cantinaBandSample[];
extern const uint32_t cantinaBandSampleLength;
#include "cantinaBandSample.h"

/******************************************************************************
 * Global variables
 *****************************************************************************/
// Variable defining the SPI port, used by the OLED and 7 segment display
Peripheral_Descriptor_t SPIPort;

// Fixed Seven segment values. Encoded to be upside down.
static const uint8_t SevenSegmentDecoder[] = {0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x30};

// Variables associated with the software timer
static xTimerHandle SoftwareTimer = NULL;
uint8_t Seconds, Minutes, Hours;

// Variables associated with the WEEE navigation
unsigned dx = 0, dy = 0, cx = 0, cy = 0;

/******************************************************************************
 * Semaphores
 *****************************************************************************/

xSemaphoreHandle xCountingSemaphore;

/******************************************************************************
 * Task Defintions
 *****************************************************************************/
/******************************************************************************
 * Description:	The callback function assigned to the SoftwareTimer.
 *
 *****************************************************************************/
static void SoftwareTimerCallback(xTimerHandle xTimer)
{
    (void)xTimer;

	// Increment timers, inside critical so that they can't be accessed while updating them
	taskENTER_CRITICAL();
		++Seconds;
		if (Seconds == 60) { Seconds = 0; ++Minutes; }
		if (Minutes == 60) { Minutes = 0; ++Hours; }
	taskEXIT_CRITICAL();
}


/******************************************************************************
 * Description:	OLED helper writing functions. Put out entire string
 *				in one critical section.
 *****************************************************************************/
void PutStringOLED1(uint8_t* String, uint8_t Line)
{
	uint8_t X = 2;
	uint8_t Ret = 1;
	while(1)
	{
		if ((*String)=='\0')
			break;
		taskENTER_CRITICAL();
			Ret = OLED_Char(X, ((Line)%7)*9 + 1, *String++, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		taskEXIT_CRITICAL();
		if (Ret == 0)
			break;

		X += 6;
	}
}


/******************************************************************************
 * Description:	Put out characters one by one, each in a critical section
 *
 *****************************************************************************/
void PutStringOLED2(uint8_t* String, uint8_t Line)
{
	taskENTER_CRITICAL();
		OLED_String(2,  ((Line)%7)*9 + 1, String, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	taskEXIT_CRITICAL();
}


/******************************************************************************
 * Description:	This task counts seconds and shows the number on the seven
 *				segment display
 *****************************************************************************/
static void SevenSegmentTask(void *pvParameters)
{
	const portTickType TaskPeriodms = 1200UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	uint8_t i = 0;
	(void)pvParameters;

	// Initialise LastExecutionTime prior to the first call to vTaskDelayUntil().
	// This only needs to be done once, as after this call, LastExectionTime is updated inside vTaskDelayUntil.
	LastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		for(i = 0; i < 10; ++i)
		{
			// Critical section here so that we don't use the SPI at the same time as the OLED
			taskENTER_CRITICAL();
				board7SEG_ASSERT_CS();
					FreeRTOS_write(SPIPort, &(SevenSegmentDecoder[i]), sizeof(uint8_t));
				board7SEG_DEASSERT_CS();
			taskEXIT_CRITICAL();

			// Delay until it is time to update the display with a new digit.
			vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
		}
	}
}


/******************************************************************************
 * Description:	This task makes the top four lines of the OLED black boxes
 *
 *****************************************************************************/
static void OLEDTask1(void *pvParameters)
{
	const portTickType TaskPeriodms = 1000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		PutStringOLED((uint8_t*)"", 0);
		PutStringOLED((uint8_t*)"", 1);
		PutStringOLED((uint8_t*)"", 2);
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task makes the top four lines of the OLED empty
 *
 *****************************************************************************/
static void OLEDTask2(void *pvParameters)
{
	const portTickType TaskPeriodms = 2000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		PutStringOLED((uint8_t*)"                ", 0);
		vTaskDelay((portTickType)100);
		PutStringOLED((uint8_t*)"                ", 1);
		vTaskDelay((portTickType)100);
		PutStringOLED((uint8_t*)"                ", 2);
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task makes the top four lines of the OLED a char
 *
 *****************************************************************************/
static void OLEDTask3(void *pvParameters)
{
	const portTickType TaskPeriodms = 4000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		PutStringOLED((uint8_t*)"<<<<<<<<<<<<<<< ", 0);
		PutStringOLED((uint8_t*)" >>>>>>>>>>>>>>>", 1);
		vTaskDelay((portTickType)400);
		PutStringOLED((uint8_t*)"<<<<<<<<<<<<<<< ", 2);
		//vTaskDelay(TaskPeriodms);
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task displays a moving + on a bar of -
 *
 *****************************************************************************/
static void OLEDTask4(void *pvParameters)
{
	const portTickType TaskPeriodms = 100UL / portTICK_RATE_MS;
	char Buffer[17] = "----------------";
	uint8_t Up = 1;
	uint8_t ID = 0;
	(void)pvParameters;

	for(;;)
	{
		if (Up)
			Buffer[ID] = '+';
		else
			Buffer[ID] = '-';

		if (ID == 15) { ID = 0; Up = !Up; }
		else { ++ID; }

		PutStringOLED((uint8_t*)Buffer, 3);

		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task displays the running time every five seconds
 * 				Not currently running
 *****************************************************************************/
static void OLEDTask5(void *pvParameters)
{
	const portTickType TaskPeriodms = 5000UL / portTICK_RATE_MS;
	char Buffer[17];
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();

	for(;;)
	{
		// Critical to prevent time variables being changed to while writing
		taskENTER_CRITICAL();
			if ((Hours < 10) && (Minutes < 10) && (Seconds < 10))	sprintf(Buffer, "Time:  0%d:0%d:0%d", (int)Hours, Minutes, Seconds);
			else if ((Hours < 10) && (Minutes < 10))				sprintf(Buffer, "Time:  0%d:0%d:%d", (int)Hours, Minutes, Seconds);
			else if ((Hours < 10) && (Seconds < 10))				sprintf(Buffer, "Time:  0%d:%d:0%d", (int)Hours, Minutes, Seconds);
			else if ((Minutes < 10) && (Seconds < 10))				sprintf(Buffer, "Time:  %d:0%d:0%d", (int)Hours, Minutes, Seconds);
			else if (Seconds < 10)									sprintf(Buffer, "Time:  %d:%d:0%d", (int)Hours, Minutes, Seconds);
			else if (Minutes < 10)									sprintf(Buffer, "Time:  %d:0%d:%d", (int)Hours, Minutes, Seconds);
			else if (Hours < 10)									sprintf(Buffer, "Time:  0%d:%d:%d", (int)Hours, Minutes, Seconds);
			else 													sprintf(Buffer, "Time:  %d:%d:%d", (int)Hours, Minutes, Seconds);
		taskEXIT_CRITICAL();

		PutStringOLED((uint8_t*)Buffer, 6);

		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task starts the tune playing, and displays its
 * 				current state on the OLED
 *****************************************************************************/
static void TuneTask(void *pvParameters)
{
	const portTickType TaskPeriodms =10UL / portTICK_RATE_MS;
	uint8_t SongStarted = 1;
	(void)pvParameters;

	for(;;)
	{
		// Pressing both buttons at same time (left and right)
		if ((((GPIO_ReadValue(0) >> 4) & 0x01) == 0) && (((GPIO_ReadValue(1) >> 31) & 0x01) == 0) && (WavPlayer_IsPlaying() == 0))
		{
			PutStringOLED((uint8_t*)" Tune: Playing  ", 4);
			SongStarted = 1;

			// Play tune
			//WavPlayer_Play(WavPlayer_Sample, WavPlayer_SampleLength);
			WavPlayer_Play(cantinaBandSample, cantinaBandSampleLength);

		} else if ((WavPlayer_IsPlaying() == 0) && (SongStarted == 1)) {
			PutStringOLED((uint8_t*)" Tune: Stopped  ", 4);
			SongStarted = 0;
		}

		vTaskDelay(TaskPeriodms);
	}
}

enum states {JOYSTICK, ROUTING, MOTOR, ENCODER};
enum states currentState;

int gridLocation[2] = {0, 0};
enum movements {NONE, LEFT, RIGHT, FORWARDS, BACKWARDS};
enum movements joystickCommands[20] = {NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE,NONE};
uint8_t joystickIndex = 0;

uint8_t isCenterJoystickPressed = 0;
uint8_t isReadyToMove = 0;

uint8_t X = 0;
uint8_t Y = 1;

uint8_t hasBeenPressed = 0;

/******************************************************************************
 * Description:	This task stores the joystick positions given by the user
 *****************************************************************************/
static void JoystickTask(void *pvParameters)
{
	const portTickType TaskPeriodms =10UL / portTICK_RATE_MS;
	(void)pvParameters;

	for(;;)
	{
		if(currentState == JOYSTICK){
			// Joystick Up
			if (((GPIO_ReadValue(2) >> 3) & 0x01) == 0){
				joystickCommands[joystickIndex] = FORWARDS;
				hasBeenPressed = 1;

			// Joystick Down
			}else if(((GPIO_ReadValue(0) >> 15) & 0x01) == 0){
				joystickCommands[joystickIndex] = BACKWARDS;
				hasBeenPressed = 1;

			// Joystick Left
			}else if(((GPIO_ReadValue(2) >> 4) & 0x01) == 0){
				joystickCommands[joystickIndex] = LEFT;
				hasBeenPressed = 1;

			// Joystick Right
			}else if(((GPIO_ReadValue(0) >> 16) & 0x01) == 0){
				joystickCommands[joystickIndex] = RIGHT;
				hasBeenPressed = 1;

			// Joystick Center
			}else if(((GPIO_ReadValue(0) >> 17) & 0x01) == 0){
				// Loop through all the queued movements
				currentState = ROUTING;
			}else{
				// comment on this
				if(joystickCommands[joystickIndex] != NONE){
					joystickIndex++;
				}

				hasBeenPressed = 0;
			}

		}

		// Delay the for loop
		vTaskDelay(TaskPeriodms*10); //100ms
	}
}

enum actions {NA, CLOCKWISE, ANTICLOCKWISE, FWD, BKWD};

struct motorInstruction {
	enum actions action_type; // Either a rotation or a movement
	int magnitude; // Distance or angle depending on action type
};

int finalGridPosition[2] = {0, 0};
struct motorInstruction queuedMotorInstructions[4];
uint8_t motorInstructionIndex = 0;
/******************************************************************************
 * Description:	Movement control. This task creates the movement objects required to reach the destination
 *****************************************************************************/
static void RoutingTask(void *pvParameters)
{
	const portTickType TaskPeriodms =10UL / portTICK_RATE_MS;
	(void)pvParameters;

	int expectedGridPosition[2];
	int xMovement;
	int yMovement;

	uint8_t i = 0;
	struct motorInstruction a1;
	struct motorInstruction a2;
	struct motorInstruction a3;
	struct motorInstruction a4;

	for(;;)
	{
		if(currentState == ROUTING){
			// Reset expected grid position
			finalGridPosition[X] = gridLocation[X];
			finalGridPosition[Y] = gridLocation[Y];

			uint8_t i;
			for(i = 0; i < sizeof(joystickCommands); i++){
				switch(joystickCommands[i]){
					case FORWARDS:
						finalGridPosition[Y]++;
						break;
					case BACKWARDS:
						finalGridPosition[Y]--;
						break;
					case LEFT:
						finalGridPosition[X]--;
						break;
					case RIGHT:
						finalGridPosition[X]++;
						break;
				}
			}

			xMovement = finalGridPosition[X] - gridLocation[X];
			yMovement = finalGridPosition[Y] - gridLocation[Y];

			if(xMovement > 0){
				// Rotate right 90 degrees
				a1.action_type = CLOCKWISE;
				a1.magnitude = 90;
				queuedMotorInstructions[motorInstructionIndex] = a1;
				motorInstructionIndex++;

				// Move forward
				a2.action_type = FORWARDS;
				a2.magnitude = abs(xMovement);
				queuedMotorInstructions[motorInstructionIndex] = a2;
				motorInstructionIndex++;

				// Rotate left 90 degrees
				a3.action_type = ANTICLOCKWISE;
				a3.magnitude = 90;
				queuedMotorInstructions[motorInstructionIndex] = a3;
				motorInstructionIndex++;

			}else if(xMovement < 0){
				// Rotate left 90 degrees
				a1.action_type = ANTICLOCKWISE;
				a1.magnitude = 90;
				queuedMotorInstructions[motorInstructionIndex] = a1;
				motorInstructionIndex++;

				// Move forward
				a2.action_type = FORWARDS;
				a2.magnitude = abs(xMovement);
				queuedMotorInstructions[motorInstructionIndex] = a2;
				motorInstructionIndex++;

				// Rotate right 90 degrees
				a3.action_type = CLOCKWISE;
				a3.magnitude = 90;
				queuedMotorInstructions[motorInstructionIndex] = a3;
				motorInstructionIndex++;
			}

			if(yMovement > 0){
				// Move forward
				a4.action_type = FORWARDS;
				a4.magnitude = yMovement;
				queuedMotorInstructions[motorInstructionIndex] = a4;
				motorInstructionIndex++;

			}else if(yMovement < 0){
				// Move backwards
				a4.action_type = BACKWARDS;
				a4.magnitude = yMovement;
				queuedMotorInstructions[motorInstructionIndex] = a4;
				motorInstructionIndex++;
			}

			// Clear the joystick movement queue
			memset(&joystickCommands[0], 0, sizeof(joystickCommands));

			currentState = MOTOR;
			motorInstructionIndex = 0; // Reset the index for the motor control task

		}

		// Delay the for loop
		vTaskDelay(TaskPeriodms); //10ms
	}
}

uint8_t movementSpeed = 10;
uint8_t isWaitingForEncoder = 0;
/******************************************************************************
 * Description:	Moves the motors according to the movement structs, with encoder feedback
 *****************************************************************************/
static void MotorControlTask(void *pvParameters)
{
	const portTickType TaskPeriodms =10UL / portTICK_RATE_MS;
	(void)pvParameters;

	uint8_t distance;

	for(;;)
	{
		if(currentState == MOTOR){
			DFR_Stop();
			// Move onto next action
			switch(queuedMotorInstructions[motorInstructionIndex].action_type){
				case FORWARDS:
					DFR_DriveForward(movementSpeed);
					// Set left and right wheel magnitude for a given action
					DFR_SetRightWheelDestination(queuedMotorInstructions[motorInstructionIndex].magnitude);
					DFR_SetLeftWheelDestination(queuedMotorInstructions[motorInstructionIndex].magnitude);
					break;
				case BACKWARDS:
					DFR_DriveBackward(movementSpeed);
					DFR_SetRightWheelDestination(queuedMotorInstructions[motorInstructionIndex].magnitude);
					DFR_SetLeftWheelDestination(queuedMotorInstructions[motorInstructionIndex].magnitude);
					break;
				case CLOCKWISE:
					DFR_DriveRight(movementSpeed);
					// If the magnitude is, for example, 90 degrees, this will correspond to a wheel destination of
					distance = queuedMotorInstructions[motorInstructionIndex].magnitude / 22.5;
					DFR_SetRightWheelDestination(distance);
					DFR_SetLeftWheelDestination(distance);
					break;
				case ANTICLOCKWISE:
					DFR_DriveLeft(movementSpeed);
					distance = queuedMotorInstructions[motorInstructionIndex].magnitude / 22.5;
					DFR_SetRightWheelDestination(distance);
					DFR_SetLeftWheelDestination(distance);
					break;
			}

			currentState = ENCODER;
		}

		vTaskDelay(TaskPeriodms);
	}
}

static void EncoderControlTask(void *pvParameters)
{
	const portTickType TaskPeriodms =10UL / portTICK_RATE_MS;
	(void)pvParameters;


	for(;;)
	{
		if(isWaitingForEncoder == 1){
			//if(DFR_GetRightWheelCount() > DFR_SetRightWheelDestination()){


			//}

			motorInstructionIndex++;
			isWaitingForEncoder = 0; // Buggy has reached the destination
		}

		vTaskDelay(TaskPeriodms);
	}
}

/******************************************************************************
 * Description: Read User Input
 *
 *****************************************************************************/
static void WEEEInputTask(void *pvParameters)
{
	const portTickType TaskPeriodms =50UL / portTICK_RATE_MS;
	(void)pvParameters;

	long test;

	for(;;)
	{

		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description: Write Input to Display
 *
 *****************************************************************************/
static void WEEEDisplayTask(void *pvParameters)
{
	const portTickType TaskPeriodms =200UL / portTICK_RATE_MS;
	(void)pvParameters;

	long test;

	for(;;)
	{
		// Show Destination X and Y | Current X and Y
		PutStringOLED((uint8_t*)"Des:0,0 Cur:0,0", 5);
		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description: Move the Robot Around
 *
 *****************************************************************************/
static void WEEEOutputTask(void *pvParameters)
{
	const portTickType TaskPeriodms =50UL / portTICK_RATE_MS;
	(void)pvParameters;

	long test;

	for(;;)
	{

		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description:
 *
 *****************************************************************************/
int main(void)
{
	// The examples assume that all priority bits are assigned as preemption priority bits.
    NVIC_SetPriorityGrouping(0UL);

    // Init SPI...
    SPIPort = FreeRTOS_open(board_SSP_PORT, (uint32_t)((void*)0));

    // Init 7seg
    GPIO_SetDir(board7SEG_CS_PORT, board7SEG_CS_PIN, boardGPIO_OUTPUT );
    board7SEG_DEASSERT_CS();

    // Init OLED
    OLED_Init(SPIPort);
    OLED_ClearScreen(OLED_COLOR_WHITE);

   	// Init wav player
   	WavPlayer_Init();

   	// Joystick Init
   	joystick_init();

   	// LED Banks Init
   	pca9532_init();

   	// Init Chassis Driver
   	DFR_RobotInit();

   	// Enable GPIO Interrupts
   	NVIC_EnableIRQ(EINT3_IRQn);

    // Create a software timer
   	SoftwareTimer = xTimerCreate((const int8_t*)"TIMER",   // Just a text name to associate with the timer, useful for debugging, but not used by the kernel.
                                 SOFTWARE_TIMER_PERIOD_MS, // The period of the timer.
                                 pdTRUE,                   // This timer will autoreload, so uxAutoReload is set to pdTRUE.
                                 NULL,                     // The timer ID is not used, so can be set to NULL.
                                 SoftwareTimerCallback);   // The callback function executed each time the timer expires.
    xTimerStart(SoftwareTimer, portMAX_DELAY);

    // Create the Seven Segment task
    xTaskCreate(SevenSegmentTask,               // The task that uses the SPI peripheral and seven segment display.
                (const int8_t* const)"7SEG",    // Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself.
                configMINIMAL_STACK_SIZE*2,     // The size of the stack allocated to the task.
                NULL,                           // The parameter is not used, so NULL is passed.
                0U,                             // The priority allocated to the task.
                NULL);                          // A handle to the task being created is not required, so just pass in NULL.

    // Create the tasks
    xTaskCreate(OLEDTask1, 			(const int8_t* const)"OLED1", 		configMINIMAL_STACK_SIZE*2, NULL, 1U, NULL);
    xTaskCreate(OLEDTask2, 			(const int8_t* const)"OLED2", 		configMINIMAL_STACK_SIZE*2, NULL, 2U, NULL);
    xTaskCreate(OLEDTask3, 			(const int8_t* const)"OLED3", 		configMINIMAL_STACK_SIZE*2, NULL, 3U, NULL);
    xTaskCreate(OLEDTask4, 			(const int8_t* const)"OLED4", 		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(OLEDTask5, 		(const int8_t* const)"OLED5", 		configMINIMAL_STACK_SIZE*2, NULL, 6U, NULL);
    xTaskCreate(TuneTask,  			(const int8_t* const)"TUNE",  		configMINIMAL_STACK_SIZE*2, NULL, 6U, NULL);
    xTaskCreate(WEEEInputTask,		(const int8_t* const)"Input",		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(WEEEDisplayTask,	(const int8_t* const)"Display",		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(WEEEOutputTask,		(const int8_t* const)"Output",		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);

    // Create the tasks we made
    xTaskCreate(JoystickTask,  			(const int8_t* const)"JoyStick",  		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(RoutingTask,		(const int8_t* const)"Routing",				configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(EncoderControlTask,	(const int8_t* const)"Encoder",				configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);

    currentState = JOYSTICK;

	// Start the FreeRTOS scheduler.
	vTaskStartScheduler();

	// The following line should never execute.
	// If it does, it means there was insufficient FreeRTOS heap memory available to create the Idle and/or timer tasks.
    for(;;);
}


/******************************************************************************
 * Interrupt Service Routines
 *****************************************************************************/
void EINT3_IRQHandler (void)
{
	// Encoder input 1 (Left)
	if ((((LPC_GPIOINT->IO2IntStatR) >> 11)& 0x1) == ENABLE)
	{

	}

	// Encoder input 2 (Right)
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 12)& 0x1) == ENABLE)
	{

	}

	// Joystick UP
	if ((((LPC_GPIOINT->IO2IntStatR) >> 3)& 0x1) == ENABLE)
	{

	}
	// Left Button
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 4)& 0x1) == ENABLE)
	{

	}

	// Clear GPIO Interrupt Flags
	// SW3
    GPIO_ClearInt(0,1 << 4);
    // Joystick | Encoder | Encoder
    GPIO_ClearInt(2,1 << 3 | 1 << 11 | 1 << 12);
}


/******************************************************************************
 * Error Checking Routines
 *****************************************************************************/
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	// Unused variables
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
    for(;;);
}


void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
    for(;;);
}
