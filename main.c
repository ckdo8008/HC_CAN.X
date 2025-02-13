/*
 * File:   main.c
 * Author: cgdo
 *
 * Created on: 2025년 2월 12일 (수), 오전 11:29
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include "EEPROM.h"
#include "ecan1_config.h"
#include "common.h"

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = OFF             // PLL Lock Enable bit (Clock switch will not wait for the PLL lock signal.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select bits (HS Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF             // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF                // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Define ECAN Message Buffers
__eds__ ECAN1MSGBUF ecan1msgBuf __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));
// CAN Messages in RAM
mID rx_ecan1message;

#define TRUE    1
#define FALSE   0
#define TJA_RS LATBbits.LATB10

uint8_t s3flag;


// *****************************************************************************
// Section: Static Function declaration
// *****************************************************************************
void OscConfig(void);
void ClearIntrflags(void);
void Ecan1WriteMessage(void);

// *****************************************************************************
// Section: Function definition
// *****************************************************************************

int main(void) {
    /* Configure Oscillator Clock Source     */
    OscConfig();

    /* Clear Interrupt Flags                 */
    ClearIntrflags();
    /* ECAN1 Initialisation
   Configure DMA Channel 0 for ECAN1 Transmit
   Configure DMA Channel 2 for ECAN1 Receive */

    TRISBbits.TRISB10 = 0;
    TRISAbits.TRISA1 = 0;
    TJA_RS = 0;
    
    Ecan1Init();
    DMA0Init();
    DMA2Init();

    /* Enable ECAN1 Interrupt */
    IEC2bits.C1IE = 1;
    C1INTEbits.TBIE = 1;
    C1INTEbits.RBIE = 1;
    C1INTEbits.ERRIE = 1;

    Ecan1WriteMessage();
    C1TR01CONbits.TXREQ0 = 1;


    /* Loop infinitely */
    while (1) {
//        if (PORTDbits.RD6 == FALSE) {
//            if (s3flag == FALSE) {
//                s3flag = TRUE;
//                Ecan1WriteMessage();
//                C1TR01CONbits.TXREQ0 = 1;
//            }
//        } else {
//            s3flag = FALSE;
//        }
    }
}

void Ecan1WriteMessage(void) {
    /* Writing the message for Transmission
Ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
Ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each

     */
    Ecan1WriteTxMsgBufId(0, 0x0, 0, 0);
    Ecan1WriteTxMsgBufData(0, 8, 0x1111, 0x2222, 0x3333, 0x4444);
}

void RxECAN1(mID *message) {
    unsigned int ide = 0;
    unsigned int srr = 0;
    unsigned long id = 0;

    /*
    Standard Message Format:
    Word0 : 0bUUUx xxxx xxxx xxxx
                 |____________|||
                     SID10:0   SRR IDE(bit 0)
    Word1 : 0bUUUU xxxx xxxx xxxx
                      |____________|
                        EID17:6
    Word2 : 0bxxxx xxx0 UUU0 xxxx
              |_____||         |__|
              EID5:0 RTR         DLC
    word3-word6: data bytes
    word7: filter hit code bits

    Substitute Remote Request Bit
    SRR->    "0"     Normal Message
            "1"  Message will request remote transmission

    Extended  Identifier Bit
    IDE->     "0"  Message will transmit standard identifier
               "1"  Message will transmit extended identifier

    Remote Transmission Request Bit
    RTR->     "0"  Message transmitted is a normal message
            "1"  Message transmitted is a remote message
     */
    /* read word 0 to see the message type */
    ide = ecan1msgBuf[message->buffer][0] & 0x0001;
    srr = ecan1msgBuf[message->buffer][0] & 0x0002;

    /* check to see what type of message it is */
    /* message is standard identifier */
    if (ide == 0) {
        message->id = (ecan1msgBuf[message->buffer][0] & 0x1FFC) >> 2;
        message->frame_type = CAN_FRAME_STD;
    }
        /* mesage is extended identifier */
    else {
        id = ecan1msgBuf[message->buffer][0] & 0x1FFC;
        message->id = id << 16;
        id = ecan1msgBuf[message->buffer][1] & 0x0FFF;
        message->id = message->id + (id << 6);
        id = (ecan1msgBuf[message->buffer][2] & 0xFC00) >> 10;
        message->id = message->id + id;
        message->frame_type = CAN_FRAME_EXT;
    }

    /* check to see what type of message it is */
    /* RTR message */
    if (srr == 1) {
        message->message_type = CAN_MSG_RTR;
    }
        /* normal message */
    else {
        message->message_type = CAN_MSG_DATA;
        message->data[0] = (unsigned char) ecan1msgBuf[message->buffer][3];
        message->data[1] = (unsigned char) ((ecan1msgBuf[message->buffer][3] & 0xFF00) >> 8);
        message->data[2] = (unsigned char) ecan1msgBuf[message->buffer][4];
        message->data[3] = (unsigned char) ((ecan1msgBuf[message->buffer][4] & 0xFF00) >> 8);
        message->data[4] = (unsigned char) ecan1msgBuf[message->buffer][5];
        message->data[5] = (unsigned char) ((ecan1msgBuf[message->buffer][5] & 0xFF00) >> 8);
        message->data[6] = (unsigned char) ecan1msgBuf[message->buffer][6];
        message->data[7] = (unsigned char) ((ecan1msgBuf[message->buffer][6] & 0xFF00) >> 8);
        message->data_length = (unsigned char) (ecan1msgBuf[message->buffer][2] & 0x000F);
//
//        Ecan1WriteMessage();
//        C1TR01CONbits.TXREQ0 = 1;
    }
}

void ClearIntrflags(void) {
    /* Clear Interrupt Flags */
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
}

void OscConfig(void) {
//    PLLFBDbits.PLLDIV = 38;
//    CLKDIVbits.PLLPOST = 0;
//    CLKDIVbits.PLLPRE = 3;
    PLLFBD = 34 - 2;           // M = 34 (PLL Feedback Divisor)
    CLKDIVbits.PLLPRE = 4 - 2; // N1 = 4 (PLL Pre-scaler)
    CLKDIVbits.PLLPOST = 0;    // N2 = 2 (PLL Post-scaler, 0 = Divide by 2)

    RCONbits.SWDTEN = 0;
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    while (OSCCONbits.COSC != 0b011);
    while (OSCCONbits.LOCK != 1);
}

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {

    if (C1INTFbits.ERRIF) {
        C1INTFbits.ERRIF = 0;
    }
    if (C1INTFbits.IVRIF) {
        C1INTFbits.IVRIF = 0;
    }
    if (C1INTFbits.TXBP) {
        C1INTFbits.TXBP = 0;
    }
    if (C1INTFbits.TBIF) {
        C1INTFbits.TBIF = 0;
    }

    if (C1INTFbits.RBIF) {
        // read the message
        if (C1RXFUL1bits.RXFUL1 == 1) {
            rx_ecan1message.buffer = 1;
            C1RXFUL1bits.RXFUL1 = 0;
        }

        RxECAN1(&rx_ecan1message);
        C1INTFbits.RBIF = 0;
    }
    IFS2bits.C1IF = 0; // clear interrupt flag
}


//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}


/*******************************************************************************
 End of File
 */