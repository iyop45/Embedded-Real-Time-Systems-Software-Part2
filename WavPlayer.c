/**************************************************************************//**
 *
 * @file		WavPlayer.c
 * @brief		Source file for a wav file player using the DAC
 * @author		Vlad Cazan
 * @author		Stephan Rochon
 * @author		Tom Coxon
 * @author		Geoffrey Daniels
 * @version		1.0
 * @date		19 July. 2012
 *
 * Copyright(C) 2012, Vlad Cazan and Stephan Rochon
 * All rights reserved.
 *
******************************************************************************/

// Includes
#include "LPC17xx_GPIO.h"
#include "LPC17xx_PinSelect.h"
#include "LPC17xx_DAC.h"
#include "LPC17xx_Timer.h"

#include "FreeRTOS.h"
#include "FreeRTOS_Task.h"
#include "FreeRTOS_Queue.h"
#include "FreeRTOS_Semaphore.h"
#include "FreeRTOS_IO.h"

#include "WavPlayer.h"

//------------------------------------------------------------------------------

// Defines and typedefs
//...

//------------------------------------------------------------------------------

// External global variables
//...

//------------------------------------------------------------------------------

// Local variables
//...

//------------------------------------------------------------------------------

// Local Functions

//------------------------------------------------------------------------------

// Public Functions
void WavPlayer_Init()
{
	PINSEL_CFG_Type PinConfig;

	GPIO_SetDir(2, 1<<0, 1); // ?
	GPIO_SetDir(2, 1<<1, 1); // ?

	GPIO_SetDir(0, 1<<27, 1); // ?
	GPIO_SetDir(0, 1<<28, 1); // ?
	GPIO_SetDir(2, 1<<13, 1); // ?
	GPIO_SetDir(0, 1<<26, 1); // ?

	GPIO_ClearValue(0, 1<<27); // Audio amplifier - Clock
	GPIO_ClearValue(0, 1<<28); // Audio amplifier - Up/Down
	GPIO_ClearValue(2, 1<<13); // Audio amplifier - Shutdown

	PinConfig.Funcnum = 2;
	PinConfig.OpenDrain = 0;
	PinConfig.Pinmode = 0;
	PinConfig.Pinnum = 26;
	PinConfig.Portnum = 0;

	PINSEL_ConfigPin(&PinConfig);

	DAC_Init(LPC_DAC);
}

static void Init_Timer0(uint32_t Time, uint32_t Prescale) {
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct;
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue = Prescale;
	TIM_MatchConfigStruct.MatchChannel = 0;
	TIM_MatchConfigStruct.IntOnMatch = TRUE;
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	TIM_MatchConfigStruct.StopOnMatch = FALSE;
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	TIM_MatchConfigStruct.MatchValue = Time;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM0, &TIM_MatchConfigStruct);
	NVIC_SetPriority(TIMER0_IRQn, ((0x01<<4)|0x01));
	NVIC_EnableIRQ(TIMER0_IRQn);
	TIM_ResetCounter(LPC_TIM0);
	TIM_Cmd(LPC_TIM0, ENABLE);
}

uint32_t SongPosition = 0;
uint32_t SongLength = 0;
const uint8_t* SongPointer;

uint8_t isPaused = 0;
void WavPlayer_Play(const uint8_t *WavArray, const uint32_t Length)
{
	portTickType Delay;
	uint32_t SampleRate = 0;
	uint32_t Position = 0;

	isPaused = 0;

	// Check RIFF header
	if ((WavArray[Position+0] != 'R') && (WavArray[Position+0] != 'r'))
		return;
	if ((WavArray[Position+1] != 'I') && (WavArray[Position+1] != 'i'))
		return;
	if ((WavArray[Position+2] != 'F') && (WavArray[Position+2] != 'f'))
		return;
	if ((WavArray[Position+3] != 'F') && (WavArray[Position+3] != 'f'))
		return;
	// Move past "RIFF"
	Position += 4;
	// Move past header
	Position += 4;

	// Format header
	if ((WavArray[Position+0] != 'W') && (WavArray[Position+0] != 'w'))
		return;
	if ((WavArray[Position+1] != 'A') && (WavArray[Position+1] != 'a'))
		return;
	if ((WavArray[Position+2] != 'V') && (WavArray[Position+2] != 'v'))
		return;
	if ((WavArray[Position+3] != 'E') && (WavArray[Position+3] != 'e'))
		return;
	// Move past "WAVE"
	Position += 4;

	// Sub header 1 ID
	if ((WavArray[Position+0] != 'F') && (WavArray[Position+0] != 'f'))
		return;
	if ((WavArray[Position+1] != 'M') && (WavArray[Position+1] != 'm'))
		return;
	if ((WavArray[Position+2] != 'T') && (WavArray[Position+2] != 't'))
		return;
	if ((WavArray[Position+3] != ' '))
		return;
	// Move past "fmt "
	Position += 4;

	// Skip ChunkSize (4 bytes), CompressionCode (2 bytes), NumberOfChannels (2 bytes)
	Position += 8;

	// Sample rate
	SampleRate = (WavArray[Position+0] | (WavArray[Position+1] << 8) | (WavArray[Position+2] << 16) | (WavArray[Position+3] << 24));
	Position += 4;

	// Calculate delay
	Delay = (1000000 / SampleRate) / portTICK_RATE_MS;

	// Skip ByteRate (4 bytes), Align (2 bytes), BitsPerSample (2 bytes)
	Position += 8;

	// Sub header 2 ID
	if ((WavArray[Position+0] != 'D') && (WavArray[Position+0] != 'd'))
		return;
	if ((WavArray[Position+1] != 'A') && (WavArray[Position+1] != 'a'))
		return;
	if ((WavArray[Position+2] != 'T') && (WavArray[Position+2] != 't'))
		return;
	if ((WavArray[Position+3] != 'A') && (WavArray[Position+3] != 'a'))
		return;
	Position += 4;

	//Skip chunk size
	Position += 4;

	// Play using timer0
	SongPosition = Position;
	SongLength = Length;
	SongPointer = WavArray;
	Init_Timer0(Delay, 1);
}

uint32_t currChar;
void TIMER0_IRQHandler(void) {
	if(isPaused == 0) {

		if (SongPosition < SongLength)
		{
			currChar = (uint32_t)(SongPointer[SongPosition++]);
			DAC_UpdateValue(LPC_DAC, currChar*4);

			if (((GPIO_ReadValue(1) >> 31) & 0x01) == 0){
				TIM_Cmd(LPC_TIM0, DISABLE);
				NVIC_DisableIRQ(TIMER0_IRQn);
				SongPosition = 0;
				SongLength = 0;
				SongPointer = 0;
			}
		}
		else
		{
			TIM_Cmd(LPC_TIM0, DISABLE);
			NVIC_DisableIRQ(TIMER0_IRQn);
			SongPosition = 0;
			SongLength = 0;
			SongPointer = 0;
		}

	}
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

uint8_t WavPlayer_IsPlaying(void) {
	if (SongPointer != 0)
		return 1;
	return 0;
}

uint8_t getIsPaused(void) {
	return isPaused;
}


void togglePauseSong(void) {
	if(isPaused == 1){
		isPaused = 0;
	}else{
		isPaused = 1;
	}
}

