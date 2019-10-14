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
 * ======== hwSpecificConfig.h ========
 *
 * Device and board specific pins need to be configured here
 *
 */

/*----------------------------------------------------------------------------
 * The following function names are deprecated.  These were updated to new 
 * names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/

#define XT1_FREQ_HZ    32768
#define XT2_FREQ_HZ  4000000
#define MCLK_FREQ_HZ 8000000
#define MCLK_FLLREF_RATIO (MCLK_FREQ_HZ / XT1_FREQ_HZ)

/*******************************************************************************
 * User-defined types
 */
typedef struct
{
    uint16_t analog0;
    int16_t  encoder0position;
    int8_t   encoder0step;
} DAQ_dataPacket;
// Size of type DAQ_dataPacket, in bytes
extern const uint8_t SIZE_DAQ_DATAPACKET;

#ifndef DEPRECATED
#define   initPorts       USBHAL_initPorts
#define   initClocks      USBHAL_initClocks
#endif

void USBHAL_initPorts(void);
void USBHAL_initClocks(void);
void USBHAL_initADC12(void);
//Released_Version_5_20_06_02
