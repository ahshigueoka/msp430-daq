#include <driverlib.h>
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

/*******************************************************************************
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hwSpecificConfig.h"

/*******************************************************************************
 * Auxiliary function declarations
 */
void initRTC(void);

/*******************************************************************************
 * Global variables
 */
// Real-time clock (RTC) values
volatile uint8_t  hour = 0, min = 0, sec = 0;
// Signalizes that the analog channel 0 reading has finished
volatile uint8_t  flagAN0_start = FALSE, flagAN0_ready = FALSE;
// Stores the 12-bit value obtained from ADC12
volatile uint16_t analogBuffer0;

/*******************************************************************************
 * Main loop
 */
void main(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2
    PMM_setVCore(PMM_CORE_LEVEL_2);

    // Configure GPIOs for low-power (output low)
    USBHAL_initPorts();

    // Configure clocks
    // MCLK = 8MHz
    // SMCLK = 4MHz
    // ACLK = XT1 = FLL = 32768Hz
    USBHAL_initClocks();

    USBHAL_initADC12();

    initRTC();

    __enable_interrupt();
    while(1)
    {
        if(flagAN0_start)
        {
            flagAN0_start = FALSE;
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
            ADC12_A_startConversion(ADC12_A_BASE, ADC12_A_MEMORY_0, ADC12_A_SINGLECHANNEL);
        }
        if(flagAN0_ready)
        {
            flagAN0_ready = FALSE;
            // Get the value from ADC12, channel 0
            analogBuffer0 = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        }
    }
}

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
    // Blink LED to show that analog to digital conversion will take place
    P1OUT ^= 0x01;
    // Start ADC12, channel 0
    flagAN0_start = TRUE;
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
    switch (__even_in_range(ADC12IV,34))
    {
    case  0: break;   //Vector  0:  No interrupt
    case  2: break;   //Vector  2:  ADC overflow
    case  4: break;   //Vector  4:  ADC timing overflow
    case  6:          //Vector  6:  ADC12IFG0
        // Warn that the reading is complete
        P1OUT ^= 0x04;
        flagAN0_ready = TRUE;

        //Exit active CPU
        //__bic_SR_register_on_exit(LPM0_bits);
        //Clear interrupt flag ADC12IFG0
        ADC12IFG &= 0xFFFE;
        break;
    case  8: break;   //Vector  8:  ADC12IFG1
    case 10: break;   //Vector 10:  ADC12IFG2
    case 12: break;   //Vector 12:  ADC12IFG3
    case 14: break;   //Vector 14:  ADC12IFG4
    case 16: break;   //Vector 16:  ADC12IFG5
    case 18: break;   //Vector 18:  ADC12IFG6
    case 20: break;   //Vector 20:  ADC12IFG7
    case 22: break;   //Vector 22:  ADC12IFG8
    case 24: break;   //Vector 24:  ADC12IFG9
    case 26: break;   //Vector 26:  ADC12IFG10
    case 28: break;   //Vector 28:  ADC12IFG11
    case 30: break;   //Vector 30:  ADC12IFG12
    case 32: break;   //Vector 32:  ADC12IFG13
    case 34: break;   //Vector 34:  ADC12IFG14
    default:
        _never_executed();
    }
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
    switch(__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
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
        SYSBERRIV = 0;  // Clear bus error flag
        USB_disable();  // Disable
    }
}
