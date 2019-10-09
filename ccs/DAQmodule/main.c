#include <driverlib.h>
#include "hwSpecificConfig.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

/*******************************************************************************
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */

/*******************************************************************************
 * Auxiliary function declarations
 */
void initRTC(void);

/*******************************************************************************
 * Global variables
 */
volatile uint8_t  hour = 0, min = 0, sec = 0; // Real-time clock (RTC) values.
volatile uint8_t  flagSendValue = FALSE;    // Signal that the analog read must be sent
volatile uint16_t analogBuffer;                 // Stores the 12-bit value obtained from ADC12

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
    USBHAL_initClocks(8000000);

    initRTC();

    __enable_interrupt();
    while(1)
    {
        _NOP();
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
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not found!
#endif
{
    P1OUT ^= 0x01;
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
