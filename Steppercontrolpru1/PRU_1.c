/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_iep.h>
#include "resource_table_empty.h"
#include "pru1_pinmap.h"

/* Mapping Constant Table (CT) registers to variables */
volatile far uint8_t CT_MCSPI0 __attribute__((cregister("MCSPI0", near), peripheral));

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", near)))
#endif

#ifndef PRU_DMEM0
#define PRU_DMEM0 far __attribute__((cregister("PRU_DMEM_0_1",  near)))
#endif

#ifndef PRU_DMEM1
#define PRU_DMEM1 far __attribute__((cregister("PRU_DMEM_1_0",  near)))
#endif

#define BitVal(data,y)     ( (data>>y) & 1)       /** Return Data.Y value   **/
#define SetBit(data,y)     data |= (1 << y)       /** Set Data.Y   to 1     **/
#define ClearBit(data,y)   data &= ~(1 << y)      /** Clear Data.Y to 0     **/
#define TogleBit(data,y)   (data ^=BitVal(y))     /** Togle Data.Y  value   **/
#define Togle(data)        (data =~data )         /** Togle Data value      **/

// ************************************************************************
/* NOTE:  Allocating shared_freq_x to PRU Shared Memory means that other PRU cores on
 *        the same subsystem must take care not to allocate data to that memory.
 *        Users also cannot rely on where in shared memory these variables are placed
 *        so accessing them from another PRU core or from the ARM is an undefined behavior.
*/

// PRU_SRAM volatile uint32_t shared_0; (reference declaration)
//PRU_DMEM0 volatile uint32_t shared_2; (reference declaration)

/* NOTE:  Here we pick where in memory to store shared_5.  The stack and
 *        heap take up the first 0x200 words, so we must start after that.
 *        Since we are hardcoding where things are stored we can share
 *        this between the PRUs and the ARM.
*/
// Skip the first 0x200 bytes of DRAM since the Makefile allocates
// 0x100 for the STACK and 0x100 for the HEAP.

PRU_DMEM1 volatile uint32_t *pru1_command_bits = (uint32_t*)(0x200); // buffer for command bits not sure how to size (malloc?)
PRU_DMEM1 volatile uint32_t *pru_1_axis_databuffer = (uint32_t*)(0x300); // buffer for axis data place holder
//  [0]A4steps [1]A4PulseLength (nanoseconds)
//  [2]A5steps [3]A5PulseLength
//  [4]A6steps [5]A6PulseLength
//  [7]A4steps@reset command
//  [8]A5steps@reset command
//  [9]A6steps@reset command
//
// Set up commandbits at offset 10300 (sharedmem)

/* PRU0
 * bit 0 - 0=ok to write new values for axis 1 (pru0)
 * bit 1 - 0=ok to write new values for axis 2 (pru0)
 * bit 2 - 0=ok to write new values for axis 3 (pru0)
 * bit 3 - Axis 1 direction
 * bit 4 - Axis 2 direction
 * bit 5 - Axis 3 direction
 * bit 6 - Axis 1 limit input set
 * bit 7 - Axis 2 limit input set
 * bit 8 - Axis 3 limit input set
 * bit 9 - lock Axis 1 (arm program cannot write new instruction until cleared (set to 0)
 * bit 10 - lock Axis 2 (arm program cannot write new instruction until cleared (set to 0)
 * bit 11 - lock Axis 3 (arm program cannot write new instruction until cleared (set to 0)
 * bit 12 - reset pru 0
 * bit
 *
 * PRU1
 * bit 0 - 0=ok to write new values for axis 1 (pru0)
 * bit 1 - 0=ok to write new values for axis 2 (pru0)
 * bit 2 - 0=ok to write new values for axis 3 (pru0)
 * bit 3 - Axis 4 direction
 * bit 4 - Axis 5 direction
 * bit 5 - Axis 6 direction
 * bit 6 - Axis 4 limit input set
 * bit 7 - Axis 5 limit input set
 * bit 8 - Axis 6 limit input set
 * bit 9 - lock Axis 1 (arm program cannot write new instruction until cleared (set to 0)
 * bit 10 - lock Axis 2 (arm program cannot write new instruction until cleared (set to 0)
 * bit 11 - lock Axis 3 (arm program cannot write new instruction until cleared (set to 0)
 * bit 12 - reset pru 0
 * bit
  */
void reset_iep(void)  //https://catch22eu.github.io/website/beaglebone/beaglebone-pru-c/
{                     //https://e2e.ti.com/support/processors-group/processors/f/processors-forum/995643/ct_iep-tmr_glb_sts_bit-cnt_ovf-behavior-not-working-as-expected
        // Set counter to 0
        CT_IEP.TMR_CNT = 0x0;
        // clear over flow
        CT_IEP.TMR_GLB_STS_bit.CNT_OVF = 0x1;
        // Enable counter
        CT_IEP.TMR_GLB_CFG = 0x11;
}

int read_iep(void)
        {
                // Return counter content
                return CT_IEP.TMR_CNT;
        }
void reset_pru(unsigned int A4pulsesatreset, unsigned int A5pulsesatreset, unsigned int A6pulsesatreset)
{
        pru_1_axis_databuffer[7]=A4pulsesatreset;
        pru_1_axis_databuffer[8]=A5pulsesatreset;
        pru_1_axis_databuffer[9]=A6pulsesatreset;
        PRU1_CTRL.CTRL_bit.SOFT_RST_N=0;
}


// *************************************************************************

/* PRCM Registers */ //not sure how these are used yet
#define CM_PER_BASE ((volatile uint8_t *)(0x44E00000))
#define SPI0_CLKCTRL  (0x4C)
#define ON (0x2)

#define MCSPI0_MODULCTRL (*((volatile uint32_t*)(&CT_MCSPI0 + 0x128)))

/* This is a char so that I can force access to R31.b0 for the host interrupt */

/* PRU-to-ARM interrupt */
#define PRU_ARM_INTERRUPT (19+16)

volatile register uint32_t __R30;
volatile register uint32_t __R31;

int main(void)
{
    uint32_t result;

    /*****************************************************************/
    /* Access PRU peripherals using Constant Table & PRU header file */
    /*****************************************************************/

    /* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Read IEPCLK[OCP_EN] for IEP clock source */
    result = CT_CFG.IEPCLK_bit.OCP_EN;


    /*****************************************************************/
    /* Access PRU Shared RAM using Constant Table                    */
    /*****************************************************************/

    /* C28 defaults to 0x00000000, we need to set bits 23:8 to 0x0100 in order to have it point to 0x00010000    */
    PRU1_CTRL.CTPPR0_bit.C28_BLK_POINTER = 0x0100;
    // set up

    uint8_t member_number =0;
    pru1_command_bits[0]=0; //make sure nothing starts moving at start
    uint32_t A4Steps=0;
    uint32_t A5Steps=0;
    uint32_t A6Steps=0;
    uint32_t A4pulselength=0;
    uint32_t A5pulselength=0;
    uint32_t A6pulselength=0;

    int interupt_count;  // only check the command bits and inputs ever so many cycles
    short cmd_bit_holder=0;
    uint32_t A4stepcounter=0;
    uint32_t A5stepcounter=0;
    uint32_t A6stepcounter=0;
    uint32_t A4time=0;
    uint32_t A5time=0;
    uint32_t A6time=0;
    for (member_number=0; member_number <16; member_number ++){ // zero out 16 words of the data buffer helps to trouble shoot
        pru_1_axis_databuffer[member_number]=0x0000;
    }


while (1) {
    // command bit 12 resets program this will be tied to e-stop possibly end limits.

    if (BitVal(pru1_command_bits[0],12)==1) {  // resets pru1 and restarts program
        ClearBit(pru1_command_bits[0],12);
        reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
        }

    // check end limits P8_39, P8_40, P8_27  PRUIN

    if((__R31&P8_39)) { //bit 25 - Axis 4 limit input set
        SetBit(pru1_command_bits[0],6);
        reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
    }
    if((__R31&P8_40)) { //bit 26 - Axis 5 limit input set
            SetBit(pru1_command_bits[0],7);
            reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
    }
    if((__R31&P8_27)) { //bit 27 - Axis 6 limit input set
                SetBit(pru1_command_bits[0],8);
                reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
    }

    // set up axis 4  P8_46 direction pin, P9_45 pulse
    if (BitVal(pru1_command_bits[0],0)==1) {  //check bit 0 if its ok to set axis 4 data
                    A4Steps=pru_1_axis_databuffer[0]*2; // two pulses is on wavelength
                    A4pulselength=(pru_1_axis_databuffer[1]/2); // pulse+pulse = step

                    pru_1_axis_databuffer[0]=0;
                    pru_1_axis_databuffer[1]=0;
                    if (BitVal(pru1_command_bits[0],3)==0) {// set direction right away
                    __R30 &= ~P8_46; // P8_46 is Axis 4 Direction
                    } else {
                    __R30 |= P8_46;
                        }
                    ClearBit(pru1_command_bits[0],0);
                    SetBit(cmd_bit_holder,0);
                    SetBit(pru1_command_bits[0],9); // send message to arm program to not write new commands to axis4
    }
       // set up axis 5  P8_44 direction pin, P8_43 pulse
    if (BitVal(pru1_command_bits[0],1)==1) {  //check bit 0 if its ok to set axis 5 data
                    A5Steps=pru_1_axis_databuffer[2]*2; // two pulses is on wavelength
                    A5pulselength=(pru_1_axis_databuffer[3]/2); // pulse+pulse = step
                    pru_1_axis_databuffer[2]=0;
                    pru_1_axis_databuffer[3]=0;

                    if (BitVal(pru1_command_bits[0],4)==0) {// set direction right away
                    __R30 &= ~P8_44; // P8_44 is Axis 5 Direction
                    } else {
                    __R30 |= P8_44;
                    }
                     ClearBit(pru1_command_bits[0],1);
                     SetBit(cmd_bit_holder,1);
                     SetBit(pru1_command_bits[0],10); // send message to arm program to not write new commands to axis2
               }

        // set up axis 6  P8_42 direction pin, P8_41 pulse
        if (BitVal(pru1_command_bits[0],2)==1) {  //check bit 0 if its ok to set axis 6 data
                     A6Steps=pru_1_axis_databuffer[4]*2; // two pulses is on wavelength;;
                     A6pulselength=(pru_1_axis_databuffer[5]/2); // pulse+pulse = step
                     pru_1_axis_databuffer[4]=0;
                     pru_1_axis_databuffer[5]=0;

                     if (BitVal(pru1_command_bits[0],5)==0) {// set direction right away
                     __R30 &= ~P8_42; // P8_42 is Axis 3 Direction
                     } else {
                     __R30 |= P8_42;
                     }
                     ClearBit(pru1_command_bits[0],2);
                     SetBit(cmd_bit_holder,2);
                     SetBit(pru1_command_bits[0],11); // send message to arm program to not write new commands to axis3
                 }



       while (cmd_bit_holder!=0) {

           // check end limits P8_39, P8_40, P8_27  PRUIN

               if((__R31&P8_39)) { //bit 25 - Axis 4 limit input set
                   SetBit(pru1_command_bits[0],6);
                   reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
               }
               if((__R31&P8_40)) { //bit 26 - Axis 5 limit input set
                       SetBit(pru1_command_bits[0],7);
                       reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
               }
               if((__R31&P8_27)) { //bit 27 - Axis 6 limit input set
                           SetBit(pru1_command_bits[0],8);
                           reset_pru(A4stepcounter,A5stepcounter,A6stepcounter);
               }


           // Axis 4 pulse generator
        if (A4Steps > 0) {

                    if (A4stepcounter==0){
                        A4time=0;
                    }

                    if ((read_iep()-A4time)>= A4pulselength){ // toggle the pulse every iep time interval that equals pulse length
                            __R30 ^= P8_45;
                            A4time=read_iep();  // not perfect when the timer rolls over may get a weird pulse (timer rolls over every 21 sec or so)
                            A4stepcounter++;
                    }
                    if (A4stepcounter==A4Steps){
                        A4stepcounter=0;
                        A4Steps=0;
                        __R30 &= ~(P8_45);
                        ClearBit(cmd_bit_holder,0);
                        ClearBit(pru1_command_bits[0],9); // send message to arm program ok write new commands to axis1
                    }
        }
        else {
            ClearBit(cmd_bit_holder,0);
            ClearBit(pru1_command_bits[0],9); // send message to arm program ok write new commands to axis1
        }
        // Axis 5 pulse generator
        if (A5Steps > 0) {

                    if (A5stepcounter==0){
                        A5time=0;
                    }

                    if ((read_iep()-A5time)>= A5pulselength){ // toggle the pulse every iep time interval that equals pulse length
                        __R30 ^= P8_43;
                        A5time=read_iep();  // not perfect when the timer rolls over may get a weird pulse (timer rolls over every 21 sec or so)
                        A5stepcounter++;
                        }
                    if (A5stepcounter==A5Steps){
                        A5stepcounter=0;
                        A5Steps=0;
                        __R30 &= ~(P8_43);
                        ClearBit(cmd_bit_holder,1);
                        ClearBit(pru1_command_bits[0],10); // send message to arm program ok write new commands to axis2
                    }}
        else {
            ClearBit(cmd_bit_holder,1);
            ClearBit(pru1_command_bits[0],10); // send message to arm program ok write new commands to axis2
        }
        // Axis 6 pulse generator (P8_41)
                if (A6Steps > 0) {

                    if (A6stepcounter==0){
                        A6time=0;
                    }

                    if ((read_iep()-A6time)>= A6pulselength){ // toggle the pulse every iep time interval that equals pulse length
                        __R30 ^= P8_41;
                        A6time=read_iep();
                        A6stepcounter++;
                    }
                    if (A6stepcounter==A6Steps){
                        A6stepcounter=0;
                        A6Steps=0;
                        __R30 &= ~(P8_41);
                        ClearBit(cmd_bit_holder,2);
                        ClearBit(pru1_command_bits[0],11); // send message to arm program ok write new commands to axis3
                    }}
                else {
                    ClearBit(cmd_bit_holder,2);
                    ClearBit(pru1_command_bits[0],11); // send message to arm program ok write new commands to axis3
        }

    }
       __delay_cycles(1000000);
} //end of interupt loop
} //end of main
