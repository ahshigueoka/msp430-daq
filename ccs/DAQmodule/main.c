/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/*  
 * ======== main.c ========
 * Efficient Sending Using USBCDC_sendDataInBackground Demo:
 *
 * The example shows how to implement efficient, high-bandwidth sending using 
 * background operations.  It prompts for any key to be pressed, and when this 
 * happens, the application sends a large amount of data to the host.  
 *
 * ----------------------------------------------------------------------------+
 * Please refer to the Examples Guide for more details.
 * ---------------------------------------------------------------------------*/

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

/*******************************************************************************
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"

/*******************************************************************************
 * Global variables
 */
// Variables from other files
const uint8_t SIZE_DAQ_DATAPACKET = sizeof(DAQ_dataPacket);

// Real-time clock (RTC) values
// volatile uint8_t  hour = 0, min = 0, sec = 0;
// Signalizes that the analog channel 0 reading has finished
// Flag variables
volatile uint8_t flagAN0_ready = FALSE;
volatile uint8_t flagSentData  = FALSE;
volatile uint8_t sendDataReturnValue = FALSE;
volatile uint8_t encoder0state = 0x00;

volatile uint16_t numNotSent = 0;
volatile DAQ_dataPacket dataVar;

/*******************************************************************************
 * Auxiliary function declarations
 */
void initRTC(void);

/*******************************************************************************
 * Main loop
 */
void main (void)
{
    uint8_t flagUSB = 0;
    WDT_A_hold(WDT_A_BASE); //Stop watchdog timer

    // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
    PMM_setVCore(PMM_CORE_LEVEL_2);
    USBHAL_initPorts();           // Config GPIOS for low-power (output low)
    USBHAL_initClocks();   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
    initRTC();

    // Initialize the encoder state according to the current reading
    encoder0state = 0x30 & P1IN;

    USBHAL_initADC12();
    flagUSB = USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

    __enable_interrupt();  // Enable interrupts globally
    
    while (1)
    {
        flagUSB = USB_getConnectionState();
        if(flagAN0_ready)
        {
            // Note: USB communication only works during either
            //       active state or LPM0
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        }
        switch (flagUSB)
        {
        // This case is executed while your device is enumerated on the
        // USB host
        case ST_ENUM_ACTIVE:
            // Check if the analog value is ready to be sent
            // flag is set by the ADC12 ISR
            if(flagAN0_ready)
            {
                // Reset the flag
                flagAN0_ready = FALSE;

                // Try to send the current state through USB
                sendDataReturnValue = USBCDC_sendDataInBackground((uint8_t *) &dataVar, SIZE_DAQ_DATAPACKET, CDC0_INTFNUM, 10);

                if(sendDataReturnValue == 0)
                {
                    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
                }
                else
                {
                    if(sendDataReturnValue == 2)
                    {
                        // Bus not available
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    }
                    else
                    {
                        // Another error
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    }
                    //Operation may still be open; cancel it
                    USBCDC_abortSend((uint16_t *) &numNotSent, CDC0_INTFNUM);
                }
            }
            break;
        //------------------------------------------------------------------
        // These cases are executed while your device is disconnected from
        // the host (meaning, not enumerated); enumerated but suspended
        // by the host, or connected to a powered hub without a USB host
        // present.
        case ST_PHYS_DISCONNECTED:
        case ST_ENUM_SUSPENDED:
        case ST_PHYS_CONNECTED_NOENUM_SUSP:
            __bis_SR_register(LPM3_bits + GIE);
            __no_operation();
            break;

        // The default is executed for the momentary state
        // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
        // seconds.  Be sure not to enter LPM3 in this state; USB
        // communication is taking place here, and therefore the mode must
        // be LPM0 or active-CPU.
        case ST_ENUM_IN_PROGRESS:
            __no_operation();
        default:
            __no_operation();;
        }
    }  //while(1)
} //main()

/*******************************************************************************
 * Starts a real-time clock on TimerA_0.
 *
 * Set the timer to count up to 32768 and roll over.
 * Generates an interrupt when it rolls over.
 * Since the ACLK is fed by the 32768Hz oscillator, one interrupt is generated
 * after each second.
 */
void initRTC(void)
{
    //Start TIMER_A
    Timer_A_initUpModeParam initUpMode = {0};
    initUpMode.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    initUpMode.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initUpMode.timerPeriod = 0x8000;
    initUpMode.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    initUpMode.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initUpMode.timerClear = TIMER_A_DO_CLEAR;
    initUpMode.startTimer = false;

    Timer_A_initUpMode(TIMER_A0_BASE, &initUpMode);
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}


/*******************************************************************************
 * Timer0 A0 interrupt service routine
 *
 * Generated when TimerA_0 (real-time clock) rolls over from 32768 to 0,
 * every second.
 *
 * Timer0, CCR0 interrupt
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
    // Start the ADC12 conversion at channel 0
    //Reset the ENC bit to set the starting memory address and conversion mode
    //sequence
    HWREG8(ADC12_A_BASE + OFS_ADC12CTL0_L) &= ~(ADC12ENC);
    //Reset the bits about to be set
    HWREG16(ADC12_A_BASE +
            OFS_ADC12CTL1) &= ~(ADC12CSTARTADD_15 + ADC12CONSEQ_3);
    HWREG8(ADC12_A_BASE + OFS_ADC12CTL1_H) |= (ADC12_A_MEMORY_0 << 4);
    HWREG8(ADC12_A_BASE + OFS_ADC12CTL1_L) |= ADC12_A_SINGLECHANNEL;
    HWREG8(ADC12_A_BASE + OFS_ADC12CTL0_L) |= ADC12ENC + ADC12SC;
}

/*******************************************************************************
 * ADC12 interrupt service routine
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(ADC12IV, 0x24))
    {
    case 0x00: break;   //Vector  0:  No interrupt
    case 0x02: break;   //Vector  2:  ADC overflow
    case 0x04: break;   //Vector  4:  ADC timing overflow
    case 0x06:          //Vector  6:  ADC12IFG0
        // Warn that the reading is complete
        dataVar.analog0 = ADC12MEM0;
        flagAN0_ready = TRUE;
        //Clear interrupt flag ADC12IFG0
        ADC12IFG &= 0xFFFE;
        //Exit active CPU
        //__bic_SR_register_on_exit(LPM0_bits);
        break;
    case 0x08: break;   //Vector  8:  ADC12IFG1
    case 0x0A: break;   //Vector 10:  ADC12IFG2
    case 0x0C: break;   //Vector 12:  ADC12IFG3
    case 0x0E: break;   //Vector 14:  ADC12IFG4
    case 0x10: break;   //Vector 16:  ADC12IFG5
    case 0x12: break;   //Vector 18:  ADC12IFG6
    case 0x14: break;   //Vector 20:  ADC12IFG7
    case 0x16: break;   //Vector 22:  ADC12IFG8
    case 0x18: break;   //Vector 24:  ADC12IFG9
    case 0x1A: break;   //Vector 26:  ADC12IFG10
    case 0x1C: break;   //Vector 28:  ADC12IFG11
    case 0x1E: break;   //Vector 30:  ADC12IFG12
    case 0x20: break;   //Vector 32:  ADC12IFG13
    case 0x22: break;   //Vector 34:  ADC12IFG14
    case 0x24: break;   //Vector 36:  ADC12IFG15
    default:
        _never_executed();
    }
}

/*******************************************************************************
 * PORT1 ISR for encoder
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT1_VECTOR)))
#endif
void port1_ISR (void)
{
    switch (__even_in_range(P1IV, 0x10))
    {
    case 0x00: break;   //Vector  0:  No interrupt
    case 0x02: break;   //Vector  2:  Pin 1.0
    case 0x04: break;   //Vector  4:  Pin 1.1
    case 0x06:          //Vector  6:  Pin 1.2
        // Either going from
        //   BA    BA
        //   00 to 01 or
        //   10 to 11
        if(encoder0state & 0x20)
        {
            // From 10 to 11
            (dataVar.encoder0position)--;
            dataVar.encoder0step = -1;
        }
        else
        {
            // From 00 to 01
            (dataVar.encoder0position)++;
            dataVar.encoder0step = 1;
        }
        //Exit active CPU
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case 0x08:          //Vector  8:  Pin 1.3
        // Either going from
        //   BA    BA
        //   00 to 10 or
        //   01 to 11
        if(encoder0state & 0x10)
        {
            // From 01 to 11
            (dataVar.encoder0position)++;
            dataVar.encoder0step = 1;
        }
        else
        {
            // From 00 to 10
            (dataVar.encoder0position)--;
            dataVar.encoder0step = -1;
        }
        //Exit active CPU
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case 0x0A:          //Vector 10:  Pin 1.4
        // Either going from
        //   BA    BA
        //   01 to 00 or
        //   11 to 10
        if(encoder0state & 0x20)
        {
            // From 11 to 10
            (dataVar.encoder0position)++;
            dataVar.encoder0step = 1;
        }
        else
        {
            // From 01 to 00
            (dataVar.encoder0position)--;
            dataVar.encoder0step = -1;
        }
        //Exit active CPU
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case 0x0C:          //Vector 12:  Pin 1.5
        // Either going from
        //   BA    BA
        //   10 to 00 or
        //   11 to 01
        if(encoder0state & 0x10)
        {
            // From 11 to 01
            (dataVar.encoder0position)--;
            dataVar.encoder0step = -1;
        }
        else
        {
            // From 10 to 00
            (dataVar.encoder0position)++;
            dataVar.encoder0step = 1;
        }
        //Exit active CPU
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case 0x0E: break;   //Vector 14:  Pin 1.6
    case 0x10: break;   //Vector 16:  Pin 1.7
    default:
        _never_executed();
    }
    // Based on the current configuration, change counter
}

/*******************************************************************************
 * UNMI_ISR
 *
 * Actions to be performed in case of non-maskable interrupts:
 *     - oscillator fault
 *     - flash memory access violation
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
            USB_disable(); //Disable
    }
}

//Released_Version_5_20_06_02
