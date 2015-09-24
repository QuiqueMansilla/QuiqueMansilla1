/* Copyright 2015, Eduardo Filomena, Juan Manuel Reta
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------


 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "Led.h"       /* <= own header */





/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
void InicializarLeds(void)
{
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_SCU_PinMux(2,10,MD_PUP,FUNC0);
	Chip_SCU_PinMux(2,11,MD_PUP,FUNC0);
	Chip_SCU_PinMux(2,12,MD_PUP,FUNC0);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,0,1<<14, 1);  // 100000000000000
	Chip_GPIO_SetDir(LPC_GPIO_PORT,1,(1<<11)|(1<<12), 1);
	Chip_SCU_PinMux(2,0,MD_PUP,FUNC4);
	Chip_SCU_PinMux(2,1,MD_PUP,FUNC4);
	Chip_SCU_PinMux(2,2,MD_PUP,FUNC4);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,5,(1<<0)|(1<<1)|(1<<2), 1);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,5,1<<2, 1);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,5,1<<2, 1);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,1,11);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,1,12);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,0);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,1);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,2);
}


void PrenderLed(void) // Led Amarillo
//uint8_t LED;
//switch LED
//case
{
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,14);
}
void ApagarLed(void)
{
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,1,11);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,1,12);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,2);
}
void InvertirLed1(void) // Led1 = Verde
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,1,12);
}

void InvertirLed2(void) // Led2 = Rojo
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,1,11);
}

void InvertirLed3(void) // Led3 = Amarillo
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,0,14);
}
void InvertirLed4(void) // Led4 = Led0B = RGBB
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,5,2);
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

