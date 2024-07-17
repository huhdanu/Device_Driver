/* define by user-------------------------------------------------------------*/
#ifndef _S32K1xx_H_
#define _S32K1xx_H_
#include <stdint.h>

//typedef _IO volatile;


/****************************************************************/
/*  Types And Constants                                         */
/****************************************************************/


/************************ list struct for register ***********************/
/*<! define struct for GPIO register  */
typedef struct 
{
	uint32_t PDOR;						// Port Data Output Register
	uint32_t PSOR;						// Port Set Output Register	
	uint32_t PCOR;						// Port Clear Output Register
	uint32_t PTOR;						// Port Toggle Output Register
	uint32_t PDIR;						// Port Data Input Register
	uint32_t PDDR;						// Port Data Direstion Register
	uint32_t PIDR;						// Port Input Disable Register

}GPIO_Typedef;


/*<! define struct for PORT register */

// typedef struct for PCC
typedef struct 
{
	uint32_t RES[32];
	uint32_t mFTFC;
	uint32_t mDMAMUX;
	uint32_t RES0[2];
	uint32_t mFlexCAN0;
	uint32_t mFlexCAN1;
	uint32_t mFTM3;
	uint32_t mADC1;
	uint32_t RES1[3];			// dont use
	uint32_t mFlexCAN2;
	uint32_t mLPSPI0;
	uint32_t mLPSPI1;
	uint32_t mLPSPI2;
	uint32_t RES2[2];
	uint32_t mPDB1;
	uint32_t mCRC;
	uint32_t RES3[3];
	uint32_t mPDB0;
	uint32_t mLPIT;
	uint32_t mFTM0;
	uint32_t mFTM1;
	uint32_t mFTM2;
	uint32_t mADC0;
	uint32_t RES4[1];
	uint32_t mRTC;
	uint32_t RES5[2];
	uint32_t mLPTMR0;
	uint32_t RES6[8];
	uint32_t mPORTA;
	uint32_t mPORTB;
	uint32_t mPORTC;
	uint32_t mPORTD;
	uint32_t mPORTE;
	uint32_t RES7[6];
	uint32_t mSAI0;
	uint32_t mSAI1;
	uint32_t RES8[4];
	uint32_t mFlexIO;
	uint32_t RES9[6];
	uint32_t mEWM;
	uint32_t RES10[4];
	uint32_t mLPI2C0;
	uint32_t mLPI2C1;
	uint32_t RES11[2];
	uint32_t mLPUART0;
	uint32_t mLPUART1;
	uint32_t mLPUART2;
	uint32_t RES12[1];
	uint32_t mFTM4;
	uint32_t mFTM5;
	uint32_t mFTM6;
	uint32_t mFTM7;
	uint32_t RES13[1];
	uint32_t mCMP0;
	uint32_t RES14[2];
	uint32_t mQSPI;
	uint32_t RES15[1];
	uint32_t mENET;
}PCC_Typedef;


/* struct for SCG */
typedef struct 
{
	uint32_t VERID;
	uint32_t PARAM;
	uint32_t RES0[2];
	uint32_t CSR;
	uint32_t RCCR;
	uint32_t VCCR;
	uint32_t HCCR;
	uint32_t CLKOUTCNFG;
	uint32_t RES1[55];
	uint32_t SOSCCSR;
	uint32_t SOSCDIV;
	uint32_t SOSCCFG;
	uint32_t RES2[61];
	uint32_t SIRCCSR;
	uint32_t SIRCDIV;
	uint32_t SIRCCFG;
	uint32_t RES3[61];
	uint32_t FIRCCSR;
	uint32_t FIRCDIV;
	uint32_t FIRCCFG;
	uint32_t RES4[189];
	uint32_t SPLLCSR;
	uint32_t SPLLDIV;
	uint32_t SPLLCFG;
}SCG_Typedef;


/* typedef struct for SysTick Timer */
typedef struct 
{
	uint32_t CSR;						// Control and Status Register
	uint32_t RVR;						// Reload Value Register
	uint32_t CVR;						// Current Value 
	uint32_t CALIB;					// Calibration Value Register
}SYSTICK_Typedef;

/* typedef struct for LPUART */
typedef struct 
{
	uint32_t VERID;					// Verision ID Register
	uint32_t PARAM;					// Parameter Register
	uint32_t GLOBAL;					// LPUART Global Register
	uint32_t PINCFG;					// LPUART Pin Configuration Register
	uint32_t BAUD;						// LPUART Baud Rate Register
	uint32_t STAT;						// LPUART Status Register
	uint32_t CTRL;						// LPUART Control Register
	uint32_t DATA;						// LPUART Data Register
	uint32_t MATCH;					// LPUART Match Address Register
	uint32_t MODIR;					// LPUART Modem IrDA Register
	uint32_t FIFO;						// LPUART FIFO Register
	uint32_t WATER;					// LPUART Watermark Register
}LPUART_Typedef;

/* typedef for LOW POWER INTERRUPT TIMER */
typedef struct 
{
	uint32_t VERID;					// Version ID Register
	uint32_t PARAM;					// Paremeter Register (PARAM) 
	uint32_t MCR;						// Module Control Register
	uint32_t MSR;						// Module Status Register
	uint32_t MIER;						// Module Interrupt Enable Register
	uint32_t SETTEN;					// Set Timer Enable Register
	uint32_t CLRTEN;					// Clear Timer Enable Register
	uint32_t NULLVALUE0;
	uint32_t TVAL0;					// Timer Value Register
	uint32_t CVAL0;					// Current Timer Value
	uint32_t TCTRL0;					// Timer Control Register
	uint32_t NULLVALUE1;
	uint32_t TVAL1;					// Timer Value Register
	uint32_t CVAL1;					// Current Timer Value
	uint32_t TCTRL1;					// Timer Control Register
	uint32_t NULLVALUE2;
	uint32_t TVAL2;					// Timer Value Register
	uint32_t CVAL2;					// Current Timer Value
	uint32_t TCTRL2;					// Timer Control Register
	uint32_t NULLVALUE3;
	uint32_t TVAL3;					// Timer Value Register
	uint32_t CVAL3;					// Current Timer Value
	uint32_t TCTRL3;					// Timer Control Register
}LPIT_Typedef;


typedef struct 
{
	uint32_t VERID;
	uint32_t PARAM;
	uint32_t RES0[2];
	uint32_t CR;
	uint32_t SR;
	uint32_t IER;	
	uint32_t DER;
	uint32_t CFGR0;
	uint32_t CFGR1;
	uint32_t RES1[2];
	uint32_t DMR0;
	uint32_t DMR1;
	uint32_t RES2[2];
	uint32_t CCR;
	uint32_t RES3[5];
	uint32_t FCR;
	uint32_t FSR;
	uint32_t TCR;
	uint32_t TDR;
	uint32_t RES4[2];
	uint32_t RSR;
	uint32_t RDR;
}LPSPI_Typedef;

/* struct for ADC */
/* typedef struct 
{
	
}ADC_Typedef; */

/* clock */
#define CORE_CLOCK					48000000u		// 48Mhz
#define LPO128_CLK					128000u			// 128Khz

/* limit count bit */
#define LIMITCOUNT24BIT				16777215u

/* macro for PCC memory map */
#define PCC_BASE 						((uint32_t)0x40065000)
#define SCG_BASE						((uint32_t)0x40064000)	

/* map base address */
#define PORTA_BASE					((uint32_t)0x40049000)	
#define PORTB_BASE					((uint32_t)0x4004A000)	
#define PORTC_BASE					((uint32_t)0x4004B000)	
#define PORTD_BASE					((uint32_t)0x4004C000)	
#define PORTE_BASE					((uint32_t)0x4004D000)	
#define GPIOA_BASE					((uint32_t)0x400FF000)
#define GPIOB_BASE					((uint32_t)0x400FF040)
#define GPIOC_BASE					((uint32_t)0x400FF080)
#define GPIOD_BASE					((uint32_t)0x400FF0C0)
#define GPIOE_BASE					((uint32_t)0x400FF100)

#define SYSTICK_BASE					((uint32_t)0xE000E010)
#define NVIC_BASE						((uint32_t)0xE000E100)

#define ADC0_BASE						((uint32_t)0x4003B000)
#define ADC1_BASE						((uint32_t)0x40027000)

/* macro for some peripheral LP-low power */
#define LPUART0_BASE					((uint32_t)0x4006A000)
#define LPUART1_BASE					((uint32_t)0x4006B000)
#define LPUART2_BASE					((uint32_t)0x4006C000)
#define LPIT_BASE						((uint32_t)0x40037000)
#define LPSPI0_BASE					((uint32_t)0x4002C000)
#define LPSPI1_BASE					((uint32_t)0x4002D000)
#define LPSPI2_BASE					((uint32_t)0x4002E000)



///#define PCC_LPUART1						((uint32_t)0x40065000|0x1AC)


/*<! Nested Vectored Interrupt Controller - NVIC  */
#define NVICID_LPUART0				31
#define NVICID_LPUART1				33
#define NVICID_LPUART2				35
#define NVICID_LPSPI0				26
#define NVICID_LPSPI1				27
#define NVICID_LPSPI2				28
#define NVICID_LPIT0					48			// for channel 0
#define NVICID_LPIT1					49			// for channel 1
#define NVICID_LPIT2					50			// for channel 2
#define NVICID_LPIT3					51			// for channel 3
#define NVICID_PORTA					59
#define NVICID_PORTB					60
#define NVICID_PORTC					61
#define NVICID_PORTD					62
#define NVICID_PORTE					63
#define NVICID_ADC0					39					
#define NVICID_ADC1					40	


/*<! define for address of register  */
#define PCC								((PCC_Typedef*)PCC_BASE)
#define SCG								((SCG_Typedef*)SCG_BASE)
#define SYSTICK						((SYSTICK_Typedef*)SYSTICK_BASE)



/*<! peripheral  */
#define GPIOA							((GPIO_Typedef*)GPIOA_BASE)
#define GPIOB							((GPIO_Typedef*)GPIOB_BASE)
#define GPIOC							((GPIO_Typedef*)GPIOC_BASE)
#define GPIOD							((GPIO_Typedef*)GPIOD_BASE)
#define GPIOE							((GPIO_Typedef*)GPIOE_BASE)
#define LPUART0						((LPUART_Typedef*)LPUART0_BASE)
#define LPUART1						((LPUART_Typedef*)LPUART1_BASE)
#define LPUART2						((LPUART_Typedef*)LPUART2_BASE)
#define LPIT							((LPIT_Typedef*)LPIT_BASE)
#define LPSPI0                   ((LPSPI_Typedef*)LPSPI0_BASE)
#define LPSPI1                   ((LPSPI_Typedef*)LPSPI1_BASE)
#define LPSPI2                   ((LPSPI_Typedef*)LPSPI2_BASE)


/* macro offset address of each feature in GPIO */



/* macro for use MASK bit */
#define MASK_2BIT						0x3		//	0b11
#define MASK_3BIT						0x7		//	0b111
#define MASK_4BIT						0xF		//	0b1111
#define MASK_5BIT						0x1F		// 0b1_1111



/* */
#define HIGH                 		1
#define LOW                   	0
#define OUTPUT                	0x1
#define INPUT                 	0x0

#define PORTA                 	0x0
#define PORTB                 	0x1
#define PORTC                 	0x2
#define PORTD                 	0x3
#define PORTE                 	0x4

#define MODE_ADC0						0
#define MODE_ADC1 					1
#define ADC_8BIT						0x0				// 0b00
#define ADC_12BIT						0x1				// 0b01
#define ADC_10BIT						0x2				// 0b10

/* macro for mode interrupt */
#define ISF_0							0x8				// when logic 0 		- 0b1000
#define ISF_RE							0x9				// when rising edge	- 0b1001
#define ISF_FE							0xA				// when falling edge	- 0b1010
#define ISF_EE							0xB				// when either edge	- 0b1011
#define ISF_1							0xC				// when logic 1		- 0b1100


// define for some FLAG
#define TDRE_LPUART1					((LPUART1->STAT)&(1<<23))			// Transmit Data Register Empty Flag equal 1 be empty
#define RDRF_LPUART1					((LPUART1->STAT)&(1<<21))			// Receive Data Register Full Flag equal 1 be full
#define TCIE_LPUART1					((LPUART1->CTRL)&(1<<22))			// Tranmission Complete Interrupt Enable
#define RIE_LPUART1					((LPUART1->CTRL)&(1<<21))			// Receiver Interrupt Enable 
#define RIE_LPUART2					((LPUART2->CTRL)&(1<<21))			// Receiver Interrupt Enable

#define TDRE_LPUART2					((LPUART2->STAT)&(1<<23))			// Transmit Data Register Empty Flag equal 1 be empty


// list enum for number of pin ADC
enum{
	SE0 = 0,
	SE1,
	SE2,
	SE3,
	SE4,
	SE5,
	SE6,
	SE7,
	SE8,
	SE9,
	SE10,
	SE11,
	SE12
};


// list enum for number of pin port
enum{
	PIN0 = 0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15,
	PIN16,
};

#endif
