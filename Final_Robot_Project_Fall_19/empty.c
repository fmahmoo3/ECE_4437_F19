/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
 #include <ti/drivers/I2C.h>
 #include <ti/drivers/SDSPI.h>
 #include <ti/drivers/SPI.h>
 #include <ti/drivers/UART.h>
 #include <ti/drivers/Watchdog.h>
 #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"

/* Other header files */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"

//global variables
void (*lookUpTable[26][26])(int arg1, int arg2) = {{NULL}};

/*
 * ======= Command Interpreter Functions =======
 */

void toggleRedLED(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;
    UARTPutString(UART_BASE, "\tToggling Red LED\n\r");
    GPIO_toggle(Board_LED2);
}
void toggleGreenLED(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;
    UARTPutString(UART_BASE, "\tToggling Green LED\n\r");
    GPIO_toggle(Board_LED1);
}
void toggleBlueLED(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;
    UARTPutString(UART_BASE, "\tToggling Blue LED\n\r");
    GPIO_toggle(Board_LED0);
}
void distanceSensorRightRead(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;
    uint32_t ui32ADC0Value[1];

    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);

    while(!ADCIntStatus(ADC0_BASE, 1, false)){
        //wait
    }

    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    unsigned char s[50];

    UARTPutString(UART_BASE, "\tRight Distance Sensor = ");
    sprintf(s,"%u\n\r", ui32ADC0Value[0]);
    UARTPutString(UART_BASE, s);

}
void distanceSensorFrontRead(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;
    uint32_t ui32ADC1Value[1];

    ADCIntClear(ADC1_BASE, 2);
    ADCProcessorTrigger(ADC1_BASE, 2);

    while(!ADCIntStatus(ADC1_BASE, 2, false)){
        //wait
    }

    ADCSequenceDataGet(ADC1_BASE, 2, ui32ADC1Value);
    unsigned char s[50];

    UARTPutString(UART_BASE, "\tFront Distance Sensor = ");
    sprintf(s,"%u\n\r", ui32ADC1Value[0]);
    UARTPutString(UART_BASE, s);

}

/*
 * ======= Helper Functions =======
 *
 */

/*
 * Milestone 4: Configure_DistanceSensor_Right()
 */
void Configure_DistanceSensor_Right(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);// Enable the GPIO D peripheral
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);// Set Pin0 as input

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable ADC0
    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0);//IDK what this does tbh
    ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE,1);

    ADCIntDisable(ADC0_BASE,1);
}
/*
 * Milestone 4: Configure_DistanceSensor_Front()
 */
void Configure_DistanceSensor_Front(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);// Enable the GPIO D peripheral
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1);// Set Pin1 as input

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);// Enable ADC1
    ADCSequenceConfigure(ADC1_BASE,2,ADC_TRIGGER_PROCESSOR,0);//IDK what this does tbh
    ADCSequenceStepConfigure(ADC1_BASE,2,0,ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE,2);

    ADCIntDisable(ADC1_BASE,2);
}
/*
 * Milestone 3: Send string to UART
 *
 */
void UARTPutString(uint32_t UART_BASE, unsigned char *stringToSend)// send string to the command line
{
    while(*stringToSend != 0)
        UARTCharPut(UART_BASE, *stringToSend++);// Waiting to send a char from the UART base port to monitor.
}

/*
 * Milestone 3: Get string from UART
 *
 */
void UARTGetString(uint32_t UART_BASE, unsigned char *stringFromUART, unsigned long ulCount)
{
    unsigned long charcount = 0;
    while (charcount <= ulCount)
    {
        stringFromUART[charcount] = UARTCharGet(UART_BASE);//Wating for a char from the UART Base port
        if ((stringFromUART[charcount] == '\r') || (stringFromUART[charcount] == '\n'))// if new line or return, break
            break;
        charcount++;
    }
    stringFromUART[charcount] = 0;
}
/*
 * Milestone 2: Configure_UART1()
 */
void Configure_UART1() // Connection for the Bluetooth
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);// Enable the UART 1 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);// Enable the GPIO B peripheral

    GPIOPinConfigure(GPIO_PB0_U1RX); // Enable UART1 function on GPIO Port B pins 6
    GPIOPinConfigure(GPIO_PB1_U1TX); // Enable UART1 function on GPIO Port B pins 7
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Configure GPIO Port B to pins 0 and 1 to be used as UART

    // Initialize the UART. Set the baud rate, number of data bits, turn off
    // parity, number of stop bits, and stick mode. The UART is enabled by the function call
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}


/*
 *  Setup bluetooth module using PB0 = Tx and PB1 = Rx
 *  Milestone 2
 */
void configureBluetooth(void)
{
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    Configure_UART1();

    // Enables port, sets pins 1-3 (RGB) pins for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

/*
 *  ======= commandInterpreter ========
 *  Get input from UART1, when available, and call lookupTable to interpret command
 *  Milestone 3
 */
void commandInterpreter(void){
    uint32_t UART_BASE = UART1_BASE;
    UARTPutString(UART_BASE, "Beginning the Program...\n\r");

    char inChar1;
    char inChar2;

    while(1){
        if(UARTCharsAvail(UART_BASE))
        {
            inChar1 = UARTCharGet(UART_BASE);

            while(!UARTCharsAvail(UART_BASE)){
                //wait
            }

            inChar2 = UARTCharGet(UART_BASE);

            if(lookUpTable[inChar1-'a'][inChar2-'a'] != NULL){
                UARTPutString(UART_BASE, "\nYour Command Has Been Received:");
                lookUpTable[inChar1-'a'][inChar2-'a'](0,0);
            }
            else{
                UARTPutString(UART_BASE, "\nYour Command Was not understood\n\r");
            }

        }
        Task_sleep(100);
    }
}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0 @ 1Hz.
 *  Milestone 1
 */
void heartBeatFxn(void)
{
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    while (1) {
        Task_sleep(500);
        GPIO_toggle(Board_LED0);
    }
}

/*
 *  ======== main ========
 */

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSDSPI();
    Board_initSPI();
    Board_initUART();
    Board_initUSB(Board_USBDEVICE);
    Board_initWatchdog();
    Board_initWiFi();

    //Configuration Function Calls
    configureBluetooth();
    Configure_DistanceSensor_Right();
    Configure_DistanceSensor_Front();

    //Setup Command Interperter 2D Array
    lookUpTable['t'-'a']['r'-'a'] = toggleRedLED;
    lookUpTable['t'-'a']['g'-'a'] = toggleGreenLED;
    lookUpTable['t'-'a']['b'-'a'] = toggleBlueLED;
    lookUpTable['d'-'a']['r'-'a'] = distanceSensorRightRead;
    lookUpTable['d'-'a']['f'-'a'] = distanceSensorFrontRead;

    /* Start BIOS */
    BIOS_start();

    return (0);
}
