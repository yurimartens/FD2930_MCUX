
#ifndef __BOARD_H
#define __BOARD_H

#define PHOENIX_IRUV				1
#define PHOENIX_IR4					4

//***********PINS CONNECTIONS**************
#define TXD0         (1UL << 2)    //P0.2 TXD0 TX UART0 (RS232 for ISP)
#define RXD0         (1UL << 3)    //P0.3 RXD0 RX UART0
#define PSEN         (1UL << 10)    //P2.10 NMI

#define SSEL1         (1UL << 6)    //P0.6 SSEL1 SPI1
#define SCK1          (1UL << 7)    //P0.7 SCK1 SPI1
#define MISO1         (1UL << 8)    //P0.8 MISO1 SPI1
#define MOSI1         (1UL << 9)    //P0.9 MOSI1 SPI1

#define IR_PORT_PIN    (1 << 11)

#define TX_485         (1UL << 15)    //P0.15 TXD1 TX UART1 (Modbus to the high level controller)
#define RX_485        (1UL << 16)    //P0.16 RXD1 RX UART1
#define RTS_485         (1UL << 20)    //P0.20 DTR1 RX UART1

#define ADC24V         (1UL << 23)    //P0.23 AD0.0 Power Supply Voltage 24V (0 - 3000 mV)
#define TEMP           (1UL << 24)    //P0.24 AD0.1 Temperature of optical system (0 - 3000 mV)
#define HV             (1UL << 25)    //P0.25 AD0.2 UV channel supply voltage (0 - 3000 mV)

#define MISO0           (1UL << 23)   //P1.23 MISO0 SPI0 (DAC4-20 AD5421 and SD card) 
#define MOSI0           (1UL << 24)    //P1.24 MOSI0 SPI0
#define SCK0            (1UL << 20)     //P1.20 SCK0 SPI0
#define SSEL0           (1UL << 21)      //P1.21 SSEL0 SPI0 for SD card
#define SYNC            (1UL << 22)     //P1.22 SYNC AD5421 control - positive pulse > 40ns latch write data to the AD5421

#define HEATON          (1UL << 0)      //P2.0 PWM1_1 optical system heater control
#define LED1            (1UL << 1)     //P2.1 LED 1, 0 - ON, 1 - OFF
#define LED2            (1UL << 2)     //P2.2 LED 2, 0 - ON, 1 - OFF
#define LED3            (1UL << 3)     //P2.3 LED 3, 0 - ON, 1 - OFF
#define IR_TEST         (1UL << 4)     //P2.4 Switch IR test light, 0 - ON, 1 - OFF
#define UV_TEST         (1UL << 5)      //P2.5 Switch UV test light, 0 - ON, 1 - OFF
//#define UV_TEST2        (1UL << 11)    //P0.11 Switch UV test light, 0 - ON, 1 - OFF// ������ �����, �� ����� ������������ ��������� �� UV_TEST P2.5 � P0.11, ��� ���������� �������� ���������� ���� ����� ��
#define R_FIRE          (1UL << 6)     //P2.6 Relay FIRE, 0 - ON, 1 - OFF
#define R_WORK          (1UL << 7)     //P2.7 Relay WORK, 0 - ON, 1 - OFF
#define R_DUST          (1UL << 8)     //P2.8 Relay DUST, 0 - ON, 1 - OFF
#define R_FIRE_MVES     (1UL << 9)      //P2.9 Relay FIRE for MVES, 0 - ON, 1 - OFF

#define HALL_PORT       (4)
#define HALL1           (1UL << 29)     //P4.29 Hall Sensor 1, 0 - magnet, 1 - no magnet
#define HALL2           (1UL << 28)     //P4.28 Hall Sensor 2

//#define HEATON          (1UL << 12)      //P2.12 PWM1_1 optical system heater control
//#define LED1            (1UL << 11)     //P2.11 LED 1, 0 - ON, 1 - OFF  RED
//#define LED2            (1UL << 13)     //P2.13 LED 2, 0 - ON, 1 - OFF  GREEN
//#define LED3            (1UL << 22)     //P0.22 LED 3, 0 - ON, 1 - OFF  BLUE
//#define IR_TEST         (1UL << 21)     //P0.21 Switch IR test light, 0 - ON, 1 - OFF
//#define UV_TEST         (1UL << 5)      //P2.5 Switch UV test light, 0 - ON, 1 - OFF
//#define R_FIRE          (1UL << 19)     //P0.19 Relay FIRE, 0 - ON, 1 - OFF
//#define R_WORK          (1UL << 7)     //P2.7 Relay WORK, 0 - ON, 1 - OFF
//#define R_WORK          (1UL << 18)     //P0.18 Relay WORK, 0 - ON, 1 - OFF
//#define R_DUST          (1UL << 17)     //P0.17 Relay DUST, 0 - ON, 1 - OFF
//#define R_FIRE_MVES     (1UL << 9)      //P2.9 Relay FIRE for MVES, 0 - ON, 1 - OFF

//#define HALL_PORT       (1)
//#define HALL1           (1UL << 9)     //P1.9 Hall Sensor 1, 0 - magnet, 1 - no magnet
//#define HALL2           (1UL << 1)     //P1.1 Hall Sensor 2
//*****************************************

// PCLK offset PCLKSEL0
#define WDT_PCLK_OFFSET      0
#define TIMER0_PCLK_OFFSET   2
#define TIMER1_PCLK_OFFSET   4
#define UART0_PCLK_OFFSET    6
#define UART1_PCLK_OFFSET    8
#define PWM0_PCLK_OFFSET    10
#define PWM1_PCLK_OFFSET    12
#define I2C0_PCLK_OFFSET    14
#define SPI_PCLK_OFFSET     16
#define RTC_PCLK_OFFSET     18
#define SSP1_PCLK_OFFSET    20
#define DAC_PCLK_OFFSET     22
#define ADC_PCLK_OFFSET     24
#define CAN1_PCLK_OFFSET    26
#define CAN2_PCLK_OFFSET    28
#define ACF_PCLK_OFFSET     30

// PCLK offset PCLKSEL1
#define QEI_PCLK_OFFSET      0
#define GPIO_INT_PCLK_OFFSET 2
#define PCB_PCLK_OFFSET      4
#define I2C1_PCLK_OFFSET     6
//#define
#define SSP0_PCLK_OFFSET    10
#define TIMER2_PCLK_OFFSET  12
#define TIMER3_PCLK_OFFSET  14
#define UART2_PCLK_OFFSET   16
#define UART3_PCLK_OFFSET   18
#define I2C2_PCLK_OFFSET    20
#define I2S_PCLK_OFFSET     22
//#define 
#define RIT_PCLK_OFFSET     26
#define SYSCON_PCLK_OFFSET  28
#define MC_PCLK_OFFSET      30

// PCLK Dividers Values
#define PCLCK_U0_CLK_8      (3 << UART0_PCLK_OFFSET)
#define PCLCK_U0_CLK_1      (1 << UART0_PCLK_OFFSET)

#define PCLCK_U1_CLK_1      (1 << UART1_PCLK_OFFSET)
#define PCLCK_U1_CLK_8      (3 << UART1_PCLK_OFFSET)

#define PCLCK_U2_CLK_1      (1 << UART2_PCLK_OFFSET)
#define PCLCK_U2_CLK_8      (3 << UART2_PCLK_OFFSET)

#define PCLCK_U3_CLK_8      (3 << UART3_PCLK_OFFSET)

#define PCLCK_T0_CLK_1      (1 << TIMER0_PCLK_OFFSET)
#define PCLCK_T0_CLK_8      (3 << TIMER0_PCLK_OFFSET)

#define PCLCK_T1_CLK_1      (1 << TIMER1_PCLK_OFFSET)
#define PCLCK_T1_CLK_8      (3 << TIMER1_PCLK_OFFSET)

#define PCLCK_T2_CLK_1      (1 << TIMER2_PCLK_OFFSET)
#define PCLCK_T2_CLK_8      (3 << TIMER2_PCLK_OFFSET)

#define PCLCK_T3_CLK_1      (1 << TIMER3_PCLK_OFFSET)
#define PCLCK_T3_CLK_8      (3 << TIMER3_PCLK_OFFSET)

#define PCLCK_SSP0_CLK_1    (1 << SSP0_PCLK_OFFSET)
#define PCLCK_SSP0_CLK_2    (2 << SSP0_PCLK_OFFSET)
#define PCLCK_SSP0_CLK_8    (3 << SSP0_PCLK_OFFSET)

#define PCLCK_SSP1_CLK_1    (1 << SSP1_PCLK_OFFSET)
#define PCLCK_SSP1_CLK_8    (3 << SSP1_PCLK_OFFSET)

#define PCLCK_I2C0_CLK_8    (3 << I2C0_PCLK_OFFSET)

//UART

#define IER_RDA_INT             (1 << 0)
#define IER_THRE_INT            (1 << 1)

#define LSR_RDA_INT         (1 << 0)
#define LSR_THRE_INT        (1 << 5)
#define LSR_RDR             (1 << 0)

#define IIR_RLS_INT         (3 << 0)
#define IIR_RDA_INT         (2 << 0)
#define IIR_CTI_INT         (6 << 0)
#define IIR_THRE_INT        (1 << 0)

//LCR

#define DLAB                (1 << 7)
#define EVEN_PARITY         (1 << 4)
#define PARITY_ENABLE       (1 << 3)
#define WLS_8BIT            (3 << 0)

//Trigger levels

#define TL_01BYTE            (0 << 6)
#define TL_04BYTE            (1 << 6)
#define TL_08BYTE            (2 << 6)
#define TL_14BYTE            (3 << 6)

//SSP

#define SCR12       (12 << 8)
#define WT8BIT      (7 << 0)
#define SSP_CPOL        (1 << 6)
#define SSP_CPHA        (1 << 7)
#define RXDMAE      (1 << 0)
#define TXDMAE      (1 << 1)

//I2C

#define I2EN        (1 << 6)
#define I2STA       (1 << 5)
#define I2STO       (1 << 4)
#define I2SI        (1 << 3)
#define I2AA        (1 << 2)



#define TX_FIFO_SIZE        16

#define RX_FIFO_SIZE        14

//-------------����������� ��� �������� ������� LPC1768------------
#define PCONP_PCTIM0    0x00000002
#define PCONP_PCTIM1    0x00000004
#define PCONP_PCUART0   0x00000008
#define PCONP_PCUART1   0x00000010
#define PCONP_PCPWM1    0x00000040
#define PCONP_PCI2C0    0x00000080
#define PCONP_PCSPI     0x00000100
#define PCONP_PCRTC     0x00000200
#define PCONP_PCSSP1    0x00000400
#define PCONP_PCAD      0x00001000
#define PCONP_PCCAN1    0x00002000
#define PCONP_PCCAN2    0x00004000
#define PCONP_PCGPIO    0x00008000
#define PCONP_PCRIT     0x00010000
#define PCONP_PCMCPWM   0x00020000
#define PCONP_PCQEI     0x00040000
#define PCONP_PCI2C1    0x00080000
#define PCONP_PCSSP0    0x00200000
#define PCONP_PCTIM2    0x00400000
#define PCONP_PCTIM3    0x00800000
#define PCONP_PCUART2   0x01000000
#define PCONP_PCUART3   0x02000000
#define PCONP_PCI2C2    0x04000000
#define PCONP_PCI2S     0x08000000
#define PCONP_PCGPDMA   0x20000000
#define PCONP_PCENET    0x40000000
#define PCONP_PCUSB     0x80000000
//--------------------------------------------------------------------

#endif /* __BOARD_H */
