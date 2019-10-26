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
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"

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

/*
 * Milestone 5: ConfigureMotorRight()
 */

//variables used to program PWM
#define PWM_FREQUENCY 55
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust1;
volatile uint8_t ui8Adjust2;

void ConfigureMotorRightAndLeft(){
    ui8Adjust1 = 30; //
    ui8Adjust2 = 30;
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //setting cpu to run at 40MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //setting PWM module clock, clocked by system clock through a divider, inputing 64 runs at 40Mhz/64 (625kHz)

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //enabling pwm1 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //enabling I/O for port A

    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6); //configuring pin pa6 as pwm output (right motor)
    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7); //configuring pin pa7 as pwm output (left motor)
    GPIOPinConfigure(GPIO_PA7_M1PWM3);

    ui32PWMClock = SysCtlClockGet() / 64; //setting our PWM clock to a variable
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1; //the count to be loaded to the load register
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);//configure module 1 PWM generator 0 as down counter
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load); //setting the count value

    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true); //enabling module 1 gen 1 as output and enabling it on out pin 2
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true); //enabling module 1 gen 1 as output and enabling it on out pin 3
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);//control for p6 pwm
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);//control for p7 pwm
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
}
void moveForward(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    ui8Adjust1 = 255;
    ui8Adjust2 = 255;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    UARTPutString(UART_BASE, "\t Moving forward\n\r");
}

void moveBackward(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    ui8Adjust1 = 255;
    ui8Adjust2 = 255;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    UARTPutString(UART_BASE, "\t Moving backwards\n\r");
}

void stopMoving(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    ui8Adjust1 = 30;
    ui8Adjust2 = 30;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    UARTPutString(UART_BASE, "\t Stop Moving\n\r");
}

void rotateRight(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    ui8Adjust1 = 255;
    ui8Adjust2 = 255;

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);//right motor moves backwards
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);//left motor moves forwards

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    UARTPutString(UART_BASE, "\t Rotating towards right\n\r");
}

void rotateLeft(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    ui8Adjust1 = 255;
    ui8Adjust2 = 255;

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);//right motor moves forwards
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);//left motor moves backwards

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

    UARTPutString(UART_BASE, "\t Rotating towards left\n\r");
}

void decreaseSpeed(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    if (ui8Adjust1 - 10 > 30){
        ui8Adjust1 -= 10;
        ui8Adjust2 -= 10;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

        UARTPutString(UART_BASE, "\t Slowing Down\n\r");
    }
    else if(ui8Adjust1 != 30){
        ui8Adjust1 = 30;
        ui8Adjust2 = 30;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);
        UARTPutString(UART_BASE, "\t Slowing Down\n\r");
    }
    else{
        UARTPutString(UART_BASE, "\t Already Stopped\n\r");
    }

}

void increaseSpeed(int arg1, int arg2){
    uint32_t UART_BASE = UART1_BASE;

    if (ui8Adjust1 + 10 < 255){
        ui8Adjust1 += 10;
        ui8Adjust2 += 10;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);

        UARTPutString(UART_BASE, "\t Speeding up \n\r");
    }
    else if(ui8Adjust1 != 255){
        ui8Adjust1 = 255;
        ui8Adjust2 = 255;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);
        UARTPutString(UART_BASE, "\t Speeding up \n\r");
    }
    else{
        UARTPutString(UART_BASE, "\t Top speed \n\r");
    }

}


/*
 * Milestone 6: Right_PIDcontrols()
 */
void right_PIDcontrols(int arg1, int arg2){
    int pid_error;
    double kp = .25;
    double ki = .25;
    double kd = 0;

    double p = 0;
    double i = 0;
    double d = 0;

    uint32_t middle = 2545; // constant value representing distance from right when in desired middle of corridor
    uint32_t rIRs;
    int last_error = 0;
    double pwm_pid;

    uint32_t UART_BASE = UART1_BASE;
    uint32_t ui32ADC0Value[1];

    while(1){
        Task_sleep(50);// simulating a 50ms wait time

        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);

        while(!ADCIntStatus(ADC0_BASE, 1, false)){
            //wait
        }

        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
        rIRs=ui32ADC0Value[0];

        pid_error = middle-rIRs;

        p = (kp*pid_error);
        i = ki*(pid_error+last_error);
        d = kd*(pid_error-last_error);

        last_error = pid_error;
        pwm_pid = p+i+d;

        if(pid_error<=500 && pid_error>=70){
            //no error, keep moving forward

            ui8Adjust2 -= pwm_pid;
            ui8Adjust1 = 255;

            UARTPutString(UART_BASE," keep moving forward\n\r " );
        }
        else if(pid_error < 70){
            // Too close to the wall, needs to move left
            // steer to left: slow down left motor and set right motor to 255 adjust

            ui8Adjust2 -= pwm_pid;
            ui8Adjust1 = 255;

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);



            // no need to set directions because will be moving forward already :)
            UARTPutString(UART_BASE," too close from wall \n\r" );
        }
        else{
            // Too far from the wall, needs to move right
            // steer to right: slow down right motor and set left motor to 255 adjust

            ui8Adjust1 -= pwm_pid;
            ui8Adjust2 = 255;

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust1 * ui32Load / 1000);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui8Adjust2 * ui32Load / 1000);



            // no need to set directions because will be moving forward already :)
            UARTPutString(UART_BASE," too far from wall \n\r" );
        }
    }

}



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
    ConfigureMotorRightAndLeft();

    //Setup Command Interpreter 2D Array
    lookUpTable['t'-'a']['r'-'a'] = toggleRedLED;
    lookUpTable['t'-'a']['g'-'a'] = toggleGreenLED;
    lookUpTable['t'-'a']['b'-'a'] = toggleBlueLED;
    lookUpTable['d'-'a']['r'-'a'] = distanceSensorRightRead;
    lookUpTable['d'-'a']['f'-'a'] = distanceSensorFrontRead;
    lookUpTable['g'-'a']['o'-'a'] = moveForward; // go = forward
    lookUpTable['r'-'a']['e'-'a'] = moveBackward;//re = reverse
    lookUpTable['s'-'a']['t'-'a'] = stopMoving;//st = stop
    lookUpTable['r'-'a']['r'-'a'] = rotateRight;//rr = rotate right
    lookUpTable['r'-'a']['l'-'a'] = rotateLeft;//rl = rotate left
    lookUpTable['i'-'a']['s'-'a'] = increaseSpeed;//is = increase Speed
    lookUpTable['d'-'a']['s'-'a'] = decreaseSpeed;//rl = decrease Speed
    lookUpTable['r'-'a']['p'-'a'] =  right_PIDcontrols;// PID CONTROL


    /* Start BIOS */
    BIOS_start();

    return (0);
}
