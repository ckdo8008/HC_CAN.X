****

# dspic33ep에서 CAN 활성화 중요 설정

## 기본 환경

### dspic33ep 동작 주파수 설정

- 120MHz
- FCY : 60MHz
- 외부 클럭 : 30MHz

``` c
    PLLFBDbits.PLLDIV = 38;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 3;
    RCONbits.SWDTEN = 0;
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    while (OSCCONbits.COSC != 0b011);
    while (OSCCONbits.LOCK != 1);
```

### CAN 드라이버

- TJA1051T/3

#### IO 정보

- Silent(Pin 8) : RB10 (RP42, Low : 일반모드, High : 사일런트 모드 수신만)
- TXD(Pin 1) : RB11(RP43)
- RXD(Pin 4) : RB12(RPI44)

## CAN 동작 속도 설정

### 500K (20TQ)

```c
#define FCAN    60000000UL
#define BITRATE 500000UL
#define NTQ     20  // 20 Time Quanta in a Bit Time
#define BRP_VAL ( (FCAN / (2 * NTQ * BITRATE)) - 1 )
```

``` c
   C1CFG1bits.SJW = 0x3;
   C1CFG1bits.BRP = BRP_VAL; // BRP = 2 (계산결과)
   C1CFG2bits.SEG1PH = 0x7;
   C1CFG2bits.SEG2PHTS = 0x1;
   C1CFG2bits.SEG2PH = 0x5;
   C1CFG2bits.PRSEG = 0x4;
   C1CFG2bits.SAM = 0x1;
```

### 1M (15TQ)

```c
#define FCAN    60000000UL
#define BITRATE 1000000UL
#define NTQ     15  
#define BRP_VAL ( (FCAN / (2 * NTQ * BITRATE)) - 1 )
```

``` c
    C1CFG1bits.SJW = 0x0;      // SJW = 1TQ (저장값 0)
    C1CFG1bits.BRP = 0x1;      // BRP = 1 (계산 결과)
    C1CFG2bits.SEG1PH = 0x6;   // Phase Segment 1 = 6+1 = 7TQ
    C1CFG2bits.SEG2PHTS = 0x1; // Phase Segment 2 선택 (사용 방식 그대로)
    C1CFG2bits.SEG2PH = 0x2;   // Phase Segment 2 = 2+1 = 3TQ
    C1CFG2bits.PRSEG = 0x3;    // Propagation Segment = 3+1 = 4TQ
    C1CFG2bits.SAM = 0x1;      // SAM 그대로
```

## IO 핀 설정

``` c
    //clear register
    RPINR26 = 0;                    
    // CAN RX는 RPI44로 설정
    RPINR26bits.C1RXR = 44;
    // CAN TX는 RP43로 설정
    RPOR4bits.RP43R = 14;

```

### DMA 메모리 설정

``` c
#define ECAN1_MSG_BUF_LENGTH    2
    typedef uint16_t ECAN1MSGBUF[ECAN1_MSG_BUF_LENGTH][8];

    extern __eds__ uint16_t ecan1msgBuf[ECAN1_MSG_BUF_LENGTH][8] __attribute__ ((space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)));
```

## DMA 설정

``` c
    DMAPWC = 0;
    DMARQC = 0;
    DMA0CON = 0x2020;
    DMA0PAD = ( int ) &C1TXD;   /* ECAN 1 (C1TXD) */
    DMA0CNT = 0x0007;
    DMA0REQ = 0x0046;           /* ECAN 1 Transmit */

    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( ecan1msgBuf );
    DMA0STAH = __builtin_dmapage( ecan1msgBuf );
    #else
    DMA0STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    DMA0STAH = 0;
    #endif
    DMA0CONbits.CHEN = 1;

    DMAPWC = 0;
    DMARQC = 0;
    DMA2CON = 0x0020;
    DMA2PAD = ( int ) &C1RXD;   /* ECAN 1 (C1RXD) */
    DMA2CNT = 0x0007;
    DMA2REQ = 0x0022;           /* ECAN 1 Receive */

    #ifdef _HAS_DMA_
    DMA2STAL = __builtin_dmaoffset( ecan1msgBuf );
    DMA2STAH = __builtin_dmapage( ecan1msgBuf );
    #else
    DMA2STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    DMA2STAH = 0;
    #endif
    DMA2CONbits.CHEN = 1;    
```

