/*
 * File:   main.c
 * Author: cgdo
 *
 * Created on: 2025년 2월 12일 (수), 오전 11:29
 *
 */
//#define FCY 70000000UL
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include "EEPROM.h"

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
#pragma config FNOSC = PRIPLL              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF             // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF                // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

#define FirmwareVirsion 0x01    // FirmwareVirsion
#define CAN_MSG_DATA 0x01    // message type
#define CAN_MSG_RTR 0x02    // data or RTR
#define CAN_FRAME_EXT 0x03    // Frame type
#define CAN_FRAME_STD 0x04    // extended or standard
#define ECAN1_MSG_BUF_LENGTH 2

// ============================================================================
// 전역 변수 및 자료구조 (필요한 CAN 관련 변수만 남김)
// ============================================================================
volatile uint8_t iRxReadFlag = 0;
volatile uint8_t txflag1 = 0;
volatile uint8_t byId = 0;
volatile uint8_t byTxData[8];
volatile uint8_t byFeedMode = 0; // 예: 피드백 메시지 종류

typedef uint16_t ECAN1MSGBUF[ECAN1_MSG_BUF_LENGTH][8];
__eds__ ECAN1MSGBUF ecan1msgBuf __attribute__((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));

// 간단한 CAN 메시지 구조체 (원본 코드의 RxMsgStruct 일부만 남김)

typedef struct {
    uint8_t data_length;
    uint8_t data[8];
    uint32_t id;
    uint8_t frame_type;
    uint8_t message_type;
    uint8_t buffer; // 메시지 버퍼 번호 등
} RxMsgStruct;

RxMsgStruct RxMsgStruct1;
RxMsgStruct RxMsgStruct1Buf;

// CAN 메시지 DMA 버퍼 (원본 코드와 동일한 구조)
//volatile uint16_t ecan1msgBuf[8][8];
volatile int tmcnt1 = 0;

// ============================================================================
// TJA1051T/3 RS 제어 핀 : RP42가 여기서 사용됩니다.
// (아래 예에서는 RP42가 PORTB의 10번 핀으로 가정)
#define TJA_RS LATBbits.LATB10

#define DIR_PIN LATAbits.LATA1
#define RS485_TRANSMIT 1
#define RS485_RECEIVE 0
volatile long lBaudrate = 1000000;
volatile unsigned char byRx2Buf[20] = {0,};

// ============================================================================
// 함수 원형
// ============================================================================
void RegINIT(void);
void VariableINIT(void);
void ECANINIT(void);

//void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
//void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void);
void ClearIntrflags(void);

// CAN 관련 함수들 (수신 필터, 마스크, 전송 버퍼 관련)
void Ecan1WriteRxAcptFilter(int16_t n, int32_t identifier, uint16_t exide, uint16_t bufPnt, uint16_t maskSel);
void Ecan1WriteRxAcptMask(int16_t m, int32_t identifier, uint16_t mide, uint16_t exide);
void Ecan1WriteTxMsgBufId(uint16_t buf, int32_t txIdentifier, uint16_t ide, uint16_t remoteTransmit);
void Ecan1WriteTxMsgBufData(uint8_t buf, uint16_t dataLength,
        uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4,
        uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8);
void SetRS485_Direction(int direction);
void DelayUSu(volatile unsigned int uDelayUs);
void Uart1TXby(volatile unsigned char byUart1Tx);
void RxECAN1(RxMsgStruct *message);

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    IFS1bits.CNIF = 0;
}

void __attribute__((interrupt, no_auto_psv))_U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;

    while (U1STAbits.OERR == 1);
    U1STAbits.OERR = 0;
    if (U1STAbits.URXDA == 1) {
        byRx2Buf[0] = U1RXREG;
    }
}

// ============================================================================
// 타이머 인터럽트: LED 토글 등 불필요한 기능 제거 (현재는 빈 인터럽트)
// ============================================================================

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // 인터럽트 플래그 클리어
    // 추가 작업이 없다면 빈 루틴으로 둡니다.

    if (tmcnt1 > 1000) {
        Ecan1WriteTxMsgBufId(0, byId, 0, 0);
        Ecan1WriteTxMsgBufData(0, 8, 0, 0, 0, 0, 3, 2, 1, 0);
        C1TR01CONbits.TXREQ0 = 1;

        //        SetRS485_Direction(RS485_TRANSMIT);
        //        DelayUSu(20);
        //        Uart1TXby(0xFF);
        //        Uart1TXby(0xFE);
        //        Uart1TXby(0x0A);
        //        SetRS485_Direction(RS485_RECEIVE);
        tmcnt1 = 0;
    } else tmcnt1++;
}

// ============================================================================
// CAN 인터럽트: 수신 메시지가 있으면 RxMsgStruct1Buf에 저장하고 플래그 설정
// ============================================================================

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
        if (C1RXFUL1bits.RXFUL1 == 1) {
            // 실제 DMA에 저장된 CAN 메시지 버퍼를 RxMsgStruct1Buf에 복사하는 코드를
            // 작성할 수 있으나 여기서는 간단히 플래그만 설정
            RxMsgStruct1Buf.buffer = 1; // 더미값
            C1RXFUL1bits.RXFUL1 = 0;
        }
        // 메시지 처리 후 플래그 설정 (실제 메시지 파싱은 별도 구현)
        //        iRxReadFlag = 1;
        RxECAN1(&RxMsgStruct1Buf);

        C1INTFbits.RBIF = 0;
    }
    IFS2bits.C1IF = 0; // CAN 인터럽트 플래그 클리어
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}

void Uart1TXby(volatile unsigned char byUart1Tx) {
    while (U1STAbits.UTXBF);
    U1TXREG = byUart1Tx;
    while (!U1STAbits.TRMT);
}

void SetRS485_Direction(int direction) {
    if (direction == RS485_TRANSMIT) {
        DIR_PIN = 1; // Set DIR_PIN high to transmit
    } else {
        DIR_PIN = 0; // Set DIR_PIN low to receive
    }
}

void DelayUSu(volatile unsigned int uDelayUs) {
    volatile int iDelayUsCnt = 0;
    for (iDelayUsCnt = 0; iDelayUsCnt < uDelayUs; iDelayUsCnt++) {
        __asm__ volatile ("repeat #39");
        __asm__ volatile ("nop");
    }
}

// ============================================================================
// main 함수 : 모터 제어, ADC, LED 출력 로직이 제거된 상태로 CAN 통신 및 기본 초기화만 수행함
// ============================================================================

int main(void) {
    RegINIT(); // 클럭, I/O, 타이머 등 기본 초기화

    VariableINIT(); // 변수 초기화 및 ID 설정
    ECANINIT(); // CAN 모듈 및 DMA 초기화


    SetRS485_Direction(RS485_TRANSMIT);
    DelayUSu(20);
    Uart1TXby(0xFF);
    Uart1TXby(0xFE);
    Uart1TXby(0x0A);
    SetRS485_Direction(RS485_RECEIVE);

    // 메인 루프에서는 수신된 CAN 메시지가 있을 경우 간단한 예로 응답 메시지를 전송한다.
    while (1) {
        if (iRxReadFlag) {
            // 예시로 수신된 메시지(RxMsgStruct1)를 처리한 후,
            // 피드백 메시지(예: 펌웨어 버전 0xFFAA)를 CAN으로 전송한다.
            byTxData[0] = byId;
            byFeedMode = 0xCE; // 예시: 피드백 명령 코드 0xCE

            Ecan1WriteTxMsgBufId(0, byTxData[0], 0, 0);
            Ecan1WriteTxMsgBufData(0, 2, 0xFF, 0xAA, 0, 0, 0, 0, 0, 0);
            C1TR01CONbits.TXREQ0 = 1; // 전송 요청

            // 메시지 처리 후 플래그 클리어
            iRxReadFlag = 0;
        }
    }

    return 0;
}

// ============================================================================
// RegINIT: 클럭, I/O, 타이머 등 초기화 (모터제어, ADC, LED 관련 설정은 제거)
// ============================================================================

void RegINIT(void) {
    // 전역 인터럽트 비활성화
    INTCON2bits.GIE = 0;

    // PLL 및 클럭 설정: 외부 30 MHz → Fosc = 140 MHz
    // 공식: Fosc = Fin * (PLLFBD+2) / ((PLLPRE+2) * (PLLPOST+1) * 2)
    // 예) PLLFBD = 26, PLLPRE = 1, PLLPOST = 0  →  Fosc = 30×(26+2)/( (1+2)*1*2 ) = 30×28/6 = 140 MHz
    //    PLLFBDbits.PLLDIV = 26;
    //    CLKDIVbits.PLLPRE = 1;
    //    CLKDIVbits.PLLPOST = 0;
    PLLFBDbits.PLLDIV = 38;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 3;
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    while (OSCCONbits.COSC != 0b011);
    while (OSCCONbits.LOCK != 1);

    // 아날로그 핀 모두 디지털 모드로 설정 (ADC 제거)
    ANSELA = 0;
    ANSELB = 0;

    // I/O 포트 설정 (필요한 경우에만 설정)
    // --- LED 출력 제거 (기존 LED 출력은 제거됨) ---

    TRISBbits.TRISB10 = 0;
    TRISAbits.TRISA1 = 0;
    TJA_RS = 1;


    // 타이머1 초기화 (주기적 인터럽트, 200µs)
    // Fosc = 140MHz → FCY = Fosc/2 = 70MHz.
    // 200µs 주기 → PR1 = 70e6 × 200e-6 = 14,000
    T1CONbits.TCKPS = 0; // Prescaler 1:1
    //    PR1 = 14000;
    PR1 = 12000;
    IPC0bits.T1IP = 1;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;

    // Change Notification (CN) 인터럽트 설정 (CAN RX 관련 필요 시)
    CNENBbits.CNIEB6 = 1;
    IEC1bits.CNIE = 1;
    IFS1bits.CNIF = 0;

    //RS485
    U1MODEbits.STSEL = 0;
    U1MODEbits.PDSEL = 0;
    U1MODEbits.ABAUD = 0;
    if (lBaudrate > 250000) {
        U1MODEbits.BRGH = 1;
        U1BRG = (120000000 / lBaudrate / 8) - 1; //(Fosc/(4*baudrate))-1
    } else {
        U1MODEbits.BRGH = 0;
        U1BRG = (120000000 / lBaudrate / 32) - 1; //(Fosc/2/(16*baudrate))-1
    }
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1RXIE = 1;
    IPC2bits.U1RXIP = 2;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;

    ANSELA = 0x0013;
    ANSELB = 0x0101;

    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR0bits.RP20R = 1; // RP20 as U1Tx
    RPINR18bits.U1RXR = 45; // RPI45 as U1Rx
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS

    //    TRISAbits.TRISA0 = 0;
    // 전역 인터럽트 재활성화
    INTCON2bits.GIE = 1;
}

// ============================================================================
// VariableINIT: 변수 초기화 (EEPROM 로드 등 필요시 처리)
// ============================================================================

void VariableINIT(void) {
    // 예시로 byId를 외부 핀 상태로 설정 (원래 코드와 유사)
    byId = 0;
}

// ============================================================================
// ECANINIT: CAN 모듈 및 DMA 초기화 (모터제어 등 불필요한 기능 제거)
// ============================================================================

void ECANINIT(void) {
    // CAN 모듈을 구성 모드로 전환
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    //    C1CTRL1bits.CANCKS = 0x1;
    //    // CAN Baudrate 1M 설정:
    //    // FCAN = Fosc/2 = 70MHz, 원하는 Tq = 100 ns (즉, BRP+1 = 7 → BRP = 6),
    //    // 전체 Bit Time = 10 Tq (즉, 1µs)
    //    C1CFG1 = 0x0006;  // BRP = 6 (BRP+1 = 7)
    //    C1CFG2 = 0x0051;
    C1CTRL1bits.CANCKS = 0x1;
    //    C1CFG1 = 0x000B;  //Baudrate
    //    C1CFG2 = 0x02A0;

    C1CFG1 = 0x0005; // BRP = 5 → (BRP+1)=6, SJW = 1 (SJW field = 0)
    // 타이밍 세그먼트 설정: (총 Bit Time = 10Tq, Tseg1 = 7Tq, Tseg2 = 2Tq)
    // PROPSEG = 3, PHSEG1 = 4, PHSEG2 = 2
    // 내부 저장 값: PROPSEG-1 = 2, PHSEG1-1 = 3, PHSEG2-1 = 1, SJW = 1 (stored = 0)
    // => C1CFG2 = (PROPSEG<<6) | (PHSEG1<<3) | (PHSEG2)
    //             = (2<<6) | (3<<3) | (1) = 0x80 + 0x18 + 0x01 = 0x99
    C1CFG2 = 0x0099;

    C1FCTRLbits.DMABS = 0;

    // 수신 필터 및 마스크 설정 (예: byId와 일치하는 메시지)
    Ecan1WriteRxAcptFilter(0, byId, 0, 1, 0);
    Ecan1WriteRxAcptMask(0, 0x7FF, 1, 0);

    // 정상 동작 모드로 전환
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);

    // 수신 플래그 클리어
    C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0;

    // 전송 버퍼 0 활성화
    C1TR01CONbits.TXEN0 = 1; /* ECAN1, Buffer 0 is a Transmit Buffer */
    C1TR01CONbits.TXEN1 = 0; /* ECAN1, Buffer 1 is a Receive Buffer */
    C1TR01CONbits.TX0PRI = 0b11; /* Message Buffer 0 Priority Level */
    C1TR01CONbits.TX1PRI = 0b11; /* Message Buffer 1 Priority Level */

    // CAN 핀 맵핑
    // CAN RX는 RPI44로 설정 (원래 42에서 44로 변경)
    RPINR26bits.C1RXR = 44;
    // CAN TX는 RP43로 설정 (변경 없음)
    RPOR4bits.RP43R = 14;

    DMAPWC = 0;
    DMARQC = 0;
    DMA0CON = 0x2020;
    DMA2CON = 0x0020;

    DMA0PAD = (int) &C1TXD; /* ECAN 1 (C1TXD) */
    DMA2PAD = (int) &C1RXD; /* ECAN 1 (C1RXD) */

    DMA0CNT = 0x0007;
    DMA2CNT = 0x0007;

    DMA0REQ = 0x0046; /* ECAN 1 Transmit */
    DMA2REQ = 0x0022; /* ECAN 1 Receive */

    //    #ifdef _HAS_DMA_
    //    DMA0STAL = __builtin_dmaoffset( ecan1msgBuf );
    //    DMA0STAH = __builtin_dmapage( ecan1msgBuf );
    //
    //    #else
    //    DMA0STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    //    DMA0STAH = 0;
    //
    //    #endif
    //    DMA0CONbits.CHEN = 1;
    //
    //    #ifdef _HAS_DMA_
    //    DMA2STAL = __builtin_dmaoffset( ecan1msgBuf );
    //    DMA2STAH = __builtin_dmapage( ecan1msgBuf );
    //    #else
    //    DMA2STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    //    DMA2STAH = 0;
    //    #endif
    //    DMA2CONbits.CHEN = 1;
    DMA0STAL = __builtin_dmaoffset(ecan1msgBuf);
    DMA0STAH = __builtin_dmapage(ecan1msgBuf);
    DMA0CONbits.CHEN = 1;
    DMA2STAL = __builtin_dmaoffset(ecan1msgBuf);
    DMA2STAH = __builtin_dmapage(ecan1msgBuf);
    DMA2CONbits.CHEN = 1;

    //    IEC0bits.DMA0IE = 1;   // DMA 채널 0 인터럽트 enable
    //    IEC1bits.DMA2IE = 1;   // DMA 채널 2 인터럽트 enable

    // CAN 인터럽트 활성화
    IEC2bits.C1IE = 1; // ECAN1 인터럽트 활성화
    C1INTEbits.TBIE = 1; // TX 버퍼 인터럽트 활성화
    C1INTEbits.RBIE = 1; // RX 버퍼 인터럽트 활성화
    C1INTEbits.ERRIE = 1;
}



// ============================================================================
// ClearIntrflags: 모든 인터럽트 플래그 클리어 (필요시 호출)
// ============================================================================

void ClearIntrflags(void) {
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
}

// ============================================================================
// 아래는 CAN 메시지 수신/전송 관련 함수들 (원본 코드와 동일한 구조)
// ============================================================================

void Ecan1WriteRxAcptFilter(int16_t n, int32_t identifier, uint16_t exide, uint16_t bufPnt, uint16_t maskSel) {
    uint32_t sid10_0 = 0;
    uint32_t eid15_0 = 0;
    uint32_t eid17_16 = 0;
    uint16_t *sidRegAddr;
    uint16_t *bufPntRegAddr;
    uint16_t *maskSelRegAddr;
    uint16_t *fltEnRegAddr;

    C1CTRL1bits.WIN = 1;

    sidRegAddr = (uint16_t *) (&C1RXF0SID + (n << 1));
    bufPntRegAddr = (uint16_t *) (&C1BUFPNT1 + (n >> 2));
    maskSelRegAddr = (uint16_t *) (&C1FMSKSEL1 + (n >> 3));
    fltEnRegAddr = (uint16_t *) (&C1FEN1);

    if (exide == 1) {
        eid15_0 = (identifier & 0xFFFF);
        eid17_16 = (identifier >> 16) & 0x3;
        sid10_0 = (identifier >> 18) & 0x7FF;
        *sidRegAddr = ((sid10_0 << 5) + 0x8) + eid17_16;
        *(sidRegAddr + 1) = eid15_0;
    } else {
        sid10_0 = (identifier & 0x7FF);
        *sidRegAddr = (sid10_0 << 5);
        *(sidRegAddr + 1) = 0;
    }

    *bufPntRegAddr = (*bufPntRegAddr) & (0xFFFF - (0xF << (4 * (n & 3))));
    *bufPntRegAddr = ((bufPnt << (4 * (n & 3))) | (*bufPntRegAddr));
    *maskSelRegAddr = (*maskSelRegAddr) & (0xFFFF - (0x3 << ((n & 7) * 2)));
    *maskSelRegAddr = ((maskSel << (2 * (n & 7))) | (*maskSelRegAddr));
    *fltEnRegAddr = ((0x1 << n) | (*fltEnRegAddr));

    C1CTRL1bits.WIN = 0;
}

void Ecan1WriteRxAcptMask(int16_t m, int32_t identifier, uint16_t mide, uint16_t exide) {
    uint32_t sid10_0 = 0;
    uint32_t eid15_0 = 0;
    uint32_t eid17_16 = 0;
    uint16_t *maskRegAddr;

    C1CTRL1bits.WIN = 1;

    maskRegAddr = (uint16_t *) (&C1RXM0SID + (m << 1));

    if (exide == 1) {
        eid15_0 = (identifier & 0xFFFF);
        eid17_16 = (identifier >> 16) & 0x3;
        sid10_0 = (identifier >> 18) & 0x7FF;
        if (mide == 1)
            *maskRegAddr = ((sid10_0 << 5) + 0x0008 + eid17_16);
        else
            *maskRegAddr = ((sid10_0 << 5) + eid17_16);
        *(maskRegAddr + 1) = eid15_0;
    } else {
        sid10_0 = (identifier & 0x7FF);
        if (mide == 1)
            *maskRegAddr = ((sid10_0 << 5) + 0x0008);
        else
            *maskRegAddr = (sid10_0 << 5);
        *(maskRegAddr + 1) = 0;
    }

    C1CTRL1bits.WIN = 0;
}

void Ecan1WriteTxMsgBufId(uint16_t buf, int32_t txIdentifier, uint16_t ide, uint16_t remoteTransmit) {
    uint32_t word0 = 0;
    uint32_t word1 = 0;
    uint32_t word2 = 0;
    uint32_t sid10_0 = 0;
    uint32_t eid5_0 = 0;
    uint32_t eid17_6 = 0;

    if (ide) {
        eid5_0 = (txIdentifier & 0x3F);
        eid17_6 = (txIdentifier >> 6) & 0xFFF;
        sid10_0 = (txIdentifier >> 18) & 0x7FF;
        word1 = eid17_6;
    } else {
        sid10_0 = (txIdentifier & 0x7FF);
    }

    if (remoteTransmit == 1) {
        word0 = ((sid10_0 << 2) | ide | 0x2);
        word2 = ((eid5_0 << 10) | 0x0200);
    } else {
        word0 = ((sid10_0 << 2) | ide);
        word2 = (eid5_0 << 10);
    }

    if (ide)
        ecan1msgBuf[buf][0] = (word0 | 0x0002);
    else
        ecan1msgBuf[buf][0] = word0;

    ecan1msgBuf[buf][1] = word1;
    ecan1msgBuf[buf][2] = word2;
}

void Ecan1WriteTxMsgBufData(uint8_t buf, uint16_t dataLength,
        uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4,
        uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8) {
    ecan1msgBuf[buf][2] = ((ecan1msgBuf[buf][2] & 0xFFF0) + dataLength);
    ecan1msgBuf[buf][3] = ((uint16_t) data2 << 8) + data1;
    ecan1msgBuf[buf][4] = ((uint16_t) data4 << 8) + data3;
    ecan1msgBuf[buf][5] = ((uint16_t) data6 << 8) + data5;
    ecan1msgBuf[buf][6] = ((uint16_t) data8 << 8) + data7;
}

void RxECAN1(RxMsgStruct *message) {
    unsigned int ide = 0;
    unsigned int srr = 0;
    unsigned long id = 0;

    ide = ecan1msgBuf[message -> buffer][0] & 0x0001;
    srr = ecan1msgBuf[message -> buffer][0] & 0x0002;

    if (ide == 0) {
        message -> id = (ecan1msgBuf[message -> buffer][0] & 0x1FFC) >> 2;
        message -> frame_type = CAN_FRAME_STD;
    } else {
        id = ecan1msgBuf[message->buffer][0] & 0x1FFC;
        message->id = id << 16;
        id = ecan1msgBuf[message->buffer][1] & 0x0FFF;
        message->id = message->id + (id << 6);
        id = (ecan1msgBuf[message->buffer][2] & 0xFC00) >> 10;
        message->id = message->id + id;
        message->frame_type = CAN_FRAME_EXT;
    }

    if (srr == 1) {
        message->message_type = CAN_MSG_RTR;
    }
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
    }
    iRxReadFlag = 1;
    txflag1 = 1;
}