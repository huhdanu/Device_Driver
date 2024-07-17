/* comment detail */

#include		"func_s32k1xx.h"				// for use prototype
#include 	"s32k1xx.h"						// for use address define
#include 	<string.h>




/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				SET to set 1
				CLEAR to set 0
**********************************************************/
void f_SetClearBit(uint32_t* nAddress, uint8_t nPosition, uint8_t cSel){
	(cSel == 1)?(*nAddress = (*nAddress)|(1<<nPosition)) : (*nAddress = (*nAddress)&(~(1<<nPosition)));
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				SET to set 1
				CLEAR to set 0
**********************************************************/
void f_SetMultiBit(uint32_t* nAddress, uint8_t nPos, uint32_t nValue, uint32_t nMask){
		*nAddress &= (uint32_t)~(nMask<<nPos);					// clear 8 9 10 at MUX
		*nAddress |= nValue<<nPos;									// set value bit for 8 9 10 at MUX
}
/* --------------------------------------------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------------------------------------------- */
	/********************* STEP TO FOLLOW ********************************** 
					1. Address PDDR of GPIO port					
					2. Address PTOR of GPIO port
	***********************************************************************/
void f_TogglePin(uint32_t* GPIOx_PDDR, uint32_t* GPIOx_PTOR){
	// set GPIO as output
	f_SetClearBit(GPIOx_PDDR,15,HIGH);
	// set for toggle status in PIN of GPIO port
	f_SetClearBit(GPIOx_PTOR,15,HIGH);
}
/* --------------------------------------------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************************** 
 *          Select port have pin want to setup
 *          Select pin in port
 *          Choose setup of pin is INPUT or OUTPUT
************************************************************************/
void f_PinMode(uint8_t PORTx, uint8_t PinX, uint8_t setIO){
	if(PORTx == 0x0){	//portA	-	0b000
		/* address of PIN in PORT */
		uint32_t *add_PORT_PCRn = (uint32_t*)(PORTA_BASE|PinX*4);
		/* set clock for PORT A to use peripheral */
		f_SetClearBit(&PCC->mPORTA,30,HIGH);
		/* set mux to select GPIO for pin */
		f_SetMultiBit(add_PORT_PCRn,8u,0x1,MASK_3BIT);
		/* address of PDDR of GPIO */
		/* set as output or input dependent on status */
		f_SetClearBit(&GPIOA->PDDR,PinX,setIO);
	}
	else if(PORTx == 0x1){	//portB	-	0b001
		/* address PORT_PCR pin of PORTD */
		uint32_t *add_PORT_PCRn = (uint32_t*)(PORTB_BASE|PinX*4);
		/* set clock for PORT B to use peripheral */
		f_SetClearBit(&PCC->mPORTB,30,HIGH);
		/* set mux to select GPIO for pin */
		f_SetMultiBit(add_PORT_PCRn,8u,0x1,MASK_3BIT);
		/* address of PDDR of GPIO */
		/* set as output or input dependent on status */
		f_SetClearBit(&GPIOB->PDDR,PinX,setIO);
	}
	else if(PORTx == 0x2){	//portC	-	0b010
		/* address PCC_PORTD and PORT_PCR pin of PORTD */
		uint32_t *add_PORT_PCRn = (uint32_t*)(PORTC_BASE|PinX*4);
		/* set clock for PORT C to use peripheral */
		f_SetClearBit(&PCC->mPORTC,30,HIGH);
		/* set mux to select GPIO for pin */
		f_SetMultiBit(add_PORT_PCRn,8u,0x1,MASK_3BIT);
		/* set as output or input dependent on status */
		f_SetClearBit(&GPIOC->PDDR,PinX,setIO);
	}
	else if(PORTx == 0x3){	//portD	-	0b011
		/* address PORT_PCR pin of PORTD */
		uint32_t *add_PORT_PCRn = (uint32_t*)(PORTD_BASE|PinX*4);
		/* set clock for PORT D to use peripheral */
		f_SetClearBit(&PCC->mPORTD,30,HIGH);
		/* set mux to select GPIO for pin */
		f_SetMultiBit(add_PORT_PCRn,8u,0x1,MASK_3BIT*4);
		/* set as output or input dependent on status */
		f_SetClearBit(&GPIOD->PDDR,PinX,setIO);
	}
	else if(PORTx == 0x4){	//portE	-	0b100
		/* address PORT_PCR pin of PORTD */
		uint32_t *add_PORT_PCRn = (uint32_t*)(PORTE_BASE|PinX*4);
		/* set clock for PORT E to use peripheral */
		f_SetClearBit(&PCC->mPORTE,30,HIGH);
		/* set mux to select GPIO for pin */
		f_SetMultiBit(add_PORT_PCRn,8u,0x1,MASK_3BIT);
		/* set as output or input dependent on status */
		f_SetClearBit(&GPIOE->PDDR,PinX,setIO);

	}
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************************** 
 *          Select PORT have PIN want to set up
 *          Select PIN in port
 *          Choose status LOW or HIGH for PIN
************************************************************************/
void f_DigitalWrite(uint8_t PORTx, uint8_t PinX, uint8_t status){
	if(PORTx == 0x0){	//portA	-	0b000
		/* set status of pin output LOW or HIGH */
		f_SetClearBit(&GPIOA->PDOR,PinX,status);
	}
	else if(PORTx == 0x1){	//portB	-	0b001
		/* set status of pin output LOW or HIGH */
		f_SetClearBit(&GPIOB->PDOR,PinX,status);
	}
	else if(PORTx == 0x2){	//portC	-	0b010
		/* set status of pin output LOW or HIGH */
		f_SetClearBit(&GPIOC->PDOR,PinX ,status);
	}
	else if(PORTx == 0x3){	//portD	-	0b011
		/* set status of pin output LOW or HIGH */
		f_SetClearBit(&GPIOD->PDOR,PinX,status);
	}
	else if(PORTx == 0x4){	//portE	-	0b100
		/* set status of pin output LOW or HIGH */
		f_SetClearBit(&GPIOE->PDOR,PinX,status);
	}
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************************** 
 *          Select PORT have PIN want to set up
 *          Select PIN in port
 *          Choose status LOW or HIGH for PIN
************************************************************************/
/* uint8_t f_DigitalRead(uint8_t PORTx, uint8_t PinX){
	if(PORTx == 0x0){	//portA	-	0b000
		// set status of pin output LOW or HIGH
		return ((GPIOA->PDIR)&(1<<PinX));
	}
	else if(PORTx == 0x1){	//portB	-	0b001
		// set status of pin output LOW or HIGH
		return ((GPIOB->PDIR)&(1<<PinX));
	}
	else if(PORTx == 0x2){	//portC	-	0b010
		// set status of pin output LOW or HIGH
		return ((GPIOC->PDIR)&(1<<PinX));
	}
	else if(PORTx == 0x3){	//portD	-	0b011
		// set status of pin output LOW or HIGH
		return ((GPIOD->PDIR)&(1<<PinX));
	}
	else if(PORTx == 0x4){	//portE	-	0b100
		// set status of pin output LOW or HIGH
		return ((GPIOE->PDIR)&(1<<PinX));
	}
}  */
/* --------------------------------------------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------------------------------------------- */
/* use to set up interrupt in PIN of PORT */
void f_GPIOPinInterrupt(uint8_t PORTx, uint8_t PinX, uint8_t modeInterrupt){
	if(PORTx == 0x0){				// for port A
		uint32_t* add_PORT_PCRn = (uint32_t*)(PORTA_BASE|PinX*4);
		// set ISF of PIN
		f_SetClearBit(add_PORT_PCRn,24,HIGH);
		// select mode for interrupt
		f_SetMultiBit(add_PORT_PCRn,16u,modeInterrupt,MASK_4BIT);
	}
	else if(PORTx == 0x1){			// for port B
		uint32_t* add_PORT_PCRn = (uint32_t*)(PORTB_BASE|PinX*4);
		// set ISF of PIN
		f_SetClearBit(add_PORT_PCRn,24,HIGH);
		// select mode for interrupt
		f_SetMultiBit(add_PORT_PCRn,16u,modeInterrupt,MASK_4BIT);
	}
	else if(PORTx == 0x2){			// for port C
		uint32_t* add_PORT_PCRn = (uint32_t*)(PORTC_BASE|PinX*4);
		// set ISF of PIN
		f_SetClearBit(add_PORT_PCRn,24,HIGH);
		// select mode for interrupt
		f_SetMultiBit(add_PORT_PCRn,16u,modeInterrupt,MASK_4BIT);
	}
	else if(PORTx == 0x3){			// for port D
		uint32_t* add_PORT_PCRn = (uint32_t*)(PORTD_BASE|PinX*4);
		// set ISF of PIN
		f_SetClearBit(add_PORT_PCRn,24,HIGH);
		// select mode for interrupt
		f_SetMultiBit(add_PORT_PCRn,16u,modeInterrupt,MASK_4BIT);
	}
	else if(PORTx == 0x4){			// for port E
		uint32_t* add_PORT_PCRn = (uint32_t*)(PORTE_BASE|PinX*4);
		// set ISF of PIN
		f_SetClearBit(add_PORT_PCRn,24,HIGH);
		// select mode for interrupt
		f_SetMultiBit(add_PORT_PCRn,16u,modeInterrupt,MASK_4BIT);
	}
}
/* --------------------------------------------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter number of Nested vestor
**********************************************************/
void f_SetNVIC(uint8_t NVICID){
	uint8_t SelRegister = (NVICID/32);					/* to select ISER Interrupt Select Enable Register */
	uint8_t SelLocationBit = (NVICID%32);				/* select location of bit in above register */
	uint32_t* NVICset = ((uint32_t*)(NVIC_BASE|SelRegister*4));			/* get value NVIC at ISER */
	*NVICset |=  1<<SelLocationBit;
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
// LPUART0:(rx-tx) PORTA pin2-3, PORTB pin0-1, PORTC pin2-3
// LPUART1:(rx-tx) PORTC pin6-7, PORTC pin8-9 --> use PTC6 and PTC7 to use USB support
// LPUART2:(rx-tx)
/********************* STEP TO FOLLOW ********************* 
				enter number of baudrate
**********************************************************/
void f_InitLPUART1(uint32_t baudrate){
	uint32_t *add_PORT_PCR6 = (uint32_t*)(PORTC_BASE|6*4);  
	uint32_t *add_PORT_PCR7 = (uint32_t*)(PORTC_BASE|7*4);
	uint32_t SBRValue = (uint32_t)(CORE_CLOCK/(baudrate*16));			// calculate value to input SBR
	
	// config clock for UART 48Mhz
	f_SetMultiBit(&SCG->FIRCDIV,8u,0x1,MASK_3BIT);
	
	/* step 1: setting Tx/Rx pin */
	// enable clock for PORT C
	f_SetClearBit(&PCC->mPORTC,30,HIGH);
	// set pin 6 PORTC as UART RX
	
	f_SetMultiBit(add_PORT_PCR6,8u,0x2,MASK_3BIT);
	// set pin 7 PORTC as UART TX
	
	f_SetMultiBit(add_PORT_PCR7,8u,0x2,MASK_3BIT);

	/* step 2: Select source LPUART */
	// select clock for UART use FIRCDIV2 48Mhz
	f_SetMultiBit(&PCC->mLPUART1,24,0x3,MASK_3BIT);		
	// select source LPUART
	f_SetClearBit(&PCC->mLPUART1,30,HIGH);

	/* step 3: setting baud rate */
	// set modulo divide rate
	
	f_SetMultiBit(&LPUART1->BAUD,0u,SBRValue,0x3FF);							// 0x3FF to mask 12 bit 
	// set Oversampling default div 15+1, maybe not set

	/* step 4: setting frame */
	// set one bit stop
	f_SetClearBit(&PCC->mLPUART1,13,LOW);
	// set 8 BIT for data number
	f_SetClearBit(&LPUART1->CTRL,4,LOW);		
	// disable PARITY bit for frame data
	f_SetClearBit(&LPUART1->CTRL,0,LOW);

	/* step 5: enable Tx or Rx */
	// Tx enable
	f_SetClearBit(&LPUART1->CTRL,19,HIGH);
	// Rx enable
	f_SetClearBit(&LPUART1->CTRL,18,HIGH);
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter number baurate
**********************************************************/
void f_InterruptLPUART1(uint32_t baudrate){
	uint32_t *add_PORT_PCR6 = (uint32_t*)(PORTC_BASE|6*4);
	uint32_t *add_PORT_PCR7 = (uint32_t*)(PORTC_BASE|7*4);
	uint32_t SBRValue = (uint32_t)(CORE_CLOCK/(baudrate*16));			// calculate value to input SBR
	
	// config clock for UART 48Mhz
	f_SetMultiBit(&SCG->FIRCDIV,8u,0x1,MASK_3BIT);
	
	/* step 1: setting Tx/Rx pin */
	// enable clock for PORT C
	f_SetClearBit(&PCC->mPORTC,30,HIGH);
	// set pin 6 PORTC as UART RX
	
	f_SetMultiBit(add_PORT_PCR6,8u,0x2,MASK_3BIT);
	// set pin 7 PORTC as UART TX
	
	f_SetMultiBit(add_PORT_PCR7,8u,0x2,MASK_3BIT);

	/* step 2: Select source LPUART */
	// select clock for UART use FIRCDIV2 48Mhz
	f_SetMultiBit(&PCC->mLPUART1,24,0x3,MASK_3BIT);		
	// select source LPUART
	f_SetClearBit(&PCC->mLPUART1,30,HIGH);

	/* step 3: setting baud rate */
	// set modulo divide rate
	
	f_SetMultiBit(&LPUART1->BAUD,0u,SBRValue,0x3FF);							// 0x3FF to mask 10 bit 
	// set Oversampling default div 15+1, maybe not set

	/* step 4: setting frame */
	// set one bit stop
	f_SetClearBit(&PCC->mLPUART1,13,LOW);
	// set 8 BIT for data number
	f_SetClearBit(&LPUART1->CTRL,4,LOW);		
	// disable PARITY bit for frame data
	f_SetClearBit(&LPUART1->CTRL,0,LOW);

	/* config for interrupt */
	//f_SetClearBit(&LPUART1->CTRL,23u,HIGH);						// set flag for Tx
	f_SetClearBit(&LPUART1->CTRL,21u,HIGH);						// set flag for Rx

	/* step 5: enable Tx or Rx */
	// Tx enable
	f_SetClearBit(&LPUART1->CTRL,19,HIGH);
	// Rx enable
	f_SetClearBit(&LPUART1->CTRL,18,HIGH);
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter number baudrate
**********************************************************/
void f_InterruptLPUART2(uint32_t baudrate){
	
	uint32_t *add_PORT_PCR6 = (uint32_t*)(PORTD_BASE|6*4);				// use pin D6 as RX
	uint32_t *add_PORT_PCR7 = (uint32_t*)(PORTD_BASE|7*4);				// use pin D7 as TX
	uint32_t SBRValue = (uint32_t)(CORE_CLOCK/(baudrate*16));			// calculate value to input SBR
	
	// config clock for UART 48Mhz
	f_SetMultiBit(&SCG->FIRCDIV,8u,0x1,MASK_3BIT);
	
	/* step 1: setting Tx/Rx pin */
	// enable clock for PORT B
	f_SetClearBit(&PCC->mPORTD,30,HIGH);
	
	// set pin 6 PORTC as UART RX
	f_SetMultiBit(add_PORT_PCR6,8u,0x2,MASK_3BIT);
	// set pin 7 PORTC as UART TX
	f_SetMultiBit(add_PORT_PCR7,8u,0x2,MASK_3BIT);

	/* step 2: Select source LPUART */
	// select clock for UART use FIRCDIV2 48Mhz
	f_SetMultiBit(&PCC->mLPUART2,24,0x3,MASK_3BIT);		
	// select source LPUART
	f_SetClearBit(&PCC->mLPUART2,30,HIGH);

	/* step 3: setting baud rate */
	// set modulo divide rate
	
	f_SetMultiBit(&LPUART2->BAUD,0u,SBRValue,0x3FF);						// 0x3FF to mask 10 bit 
	// set Oversampling default div 15+1, maybe not set

	/* step 4: setting frame */
	// set one bit stop
	f_SetClearBit(&PCC->mLPUART2,13,LOW);
	// set 8 BIT for data number
	f_SetClearBit(&LPUART2->CTRL,4,LOW);		
	// disable PARITY bit for frame data
	f_SetClearBit(&LPUART2->CTRL,0,LOW);

	/* config for interrupt */
	//f_SetClearBit(&LPUART1->CTRL,23u,HIGH);									// set flag for Tx
	f_SetClearBit(&LPUART2->CTRL,21u,HIGH);									// set flag for Rx

	/* step 5: enable Tx or Rx */
	// Tx enable
	f_SetClearBit(&LPUART2->CTRL,19,HIGH);
	// Rx enable
	f_SetClearBit(&LPUART2->CTRL,18,HIGH);
}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************************** 
 *          	Insert time interrupt to process handler(Tick)
************************************************************************/
void f_SysTickInit(uint32_t ValueInterrupt){
	/* calculate value to set for Reload Value Register */
	uint32_t ValueSet = (CORE_CLOCK/ValueInterrupt - 1);
	uint32_t ValueSet_save = ValueSet;
	(ValueSet>=LIMITCOUNT24BIT)?(ValueSet=LIMITCOUNT24BIT):(ValueSet = ValueSet_save);
	// disable Clock
	f_SetClearBit(&SYSTICK->CSR,0,LOW);
	// set reload max
	f_SetMultiBit(&SYSTICK->RVR,0,ValueSet,0xFFFFFFFF);
	// enable clock
	f_SetMultiBit(&SYSTICK->CSR,0,0x5,MASK_3BIT);					// 0x5 for disable interrupt
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				use to delay each 1ms
**********************************************************/
void f_delayMillis(void){
	f_SetMultiBit(&SYSTICK->RVR,0,(CORE_CLOCK/1000)-1,0xFFFFFFFF);		// 1ms
	while((SYSTICK->CSR & (1<<16)) == 0);			// wait count flag turn to 1
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter number delay ms
**********************************************************/
void f_delayMs(uint32_t t){
	for(;t>0;t--){
		f_delayMillis();
	}
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter symbol to sent by UART
**********************************************************/
void f_SentSymbol(char symbol){
	while(TDRE_LPUART2 == 0);							// wait to TDRE empty
	LPUART2->DATA = symbol;
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter string to sent by UART
**********************************************************/
void f_SentStringUART(char* string){
	while(*string != '\0'){
		f_SentSymbol(*string);
		string++;
	}
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter symbol to sent by SPI
**********************************************************/
void f_SentSymbolSPI(char symbol){
	while(((LPSPI0->SR)&1) == 0);
	LPSPI0->TDR = symbol;
}


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter symbol sent to MAX7219
**********************************************************/
void f_SentMAX7219(short symbol){
	while(((LPSPI0->SR)&1) == 0);
	LPSPI0->TDR = (uint32_t)symbol;
}
/* --------------------------------------------------------------------------------------------------------------- */


//extern char g_RCF;
///* --------------------------------------------------------------------------------------------------------------- */
///********************* STEP TO FOLLOW ********************* 
//				enter string to sent by SPI to slave
//**********************************************************/
//void f_SentStringSPI(char* string){
//	while(*string != '\0'){
//		f_SentSymbolSPI(*string);
//		g_RCF = 1;								// enable Receive complete flag
//		while(g_RCF);							// receive complete
//		string++;
//	}
//	f_SentSymbolSPI(*string);
//}
///* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter 2 string to compare
**********************************************************/
uint8_t f_CompareString(char* string1, char* string2){
	if(strlen(string1) != strlen(string2)) return 0;				// compare length
	else{
		while((*string1!='\0') && (*string2!='\0')){				// check each components in string
			if(*string1 == *string2);
			else return 0;
			string1++;
			string2++;
		}
		return 1;
	}
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				init LPIT
**********************************************************/
void f_InitLPIT(void){
//	// config clock for timer select SIRCDIV2
//	f_SetMultiBit((uint32_t*)(SCG_BASE|0x204),8u,0b100,MASK_3BIT);			// set SIRDCDIV2_CLK div8 equal 1Mhz
	
	/* step 1: setting cloking */
	f_SetMultiBit((uint32_t*)(PCC_BASE|0xDC),24u,0x7,MASK_3BIT);			// use LPO_CLK		
	f_SetClearBit((uint32_t*)(PCC_BASE|0xDC),30u,HIGH);						// enable clock

	/* step 2: LPIT initialization */
	f_SetClearBit(&LPIT->MCR,0u, HIGH);												// enable clock for TIMER
	f_SetClearBit(&LPIT->MCR,3u, LOW);												// stop timer channel in Debug mode

	/* step 3: Config channel */
	f_SetMultiBit(&LPIT->TCTRL0,2u,0x0,MASK_2BIT);								// select 32 bit, auto reload when overflow

	/* set value */
	f_SetMultiBit(&LPIT->TVAL0,0u,128000u,0xffffffff);							// 1s count down

	/* step 4: Timer channel */
	f_SetClearBit(&LPIT->TCTRL0,0u, HIGH);											// enable TIMER channel
	
	/* step 5: Get timer Counter value */

}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				enter number ms want to be interrupt
**********************************************************/
void f_InterruptLPIT(uint16_t millisecond){
	
	uint32_t valueSet = (uint32_t)((millisecond/1000)*LPO128_CLK);
	
	/* step 1: setting cloking */
	f_SetMultiBit(&PCC->mLPIT,24u,0x7,MASK_3BIT);									// use LPO_CLK	
	f_SetClearBit(&PCC->mLPIT,30u,HIGH);												// enable clock

	/* step 2: LPIT initialization */
	f_SetClearBit(&LPIT->MCR,0u, HIGH);													// enable clock for TIMER
	f_SetClearBit(&LPIT->MCR,3u, LOW);													// stop timer channel in Debug mode

	/* enable channel interrupt */
	f_SetClearBit(&LPIT->MIER,0u,HIGH);													// enable interrupt channel

	/* step 3: Config channel */
	f_SetMultiBit(&LPIT->TCTRL0,2u,0x0,MASK_2BIT);									// select 32 bit, auto reload when overflow

	/* set value */
	
	f_SetMultiBit(&LPIT->TVAL0,0u,valueSet-1,0xffffffff);							// 1s count down

	/* step 4: Timer channel */
	f_SetClearBit(&LPIT->TCTRL0,0u, HIGH);												// enable TIMER channel
	
	/* step 5: Get timer Counter value */

}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
				select channel use ADC
				select number channel in that ADC
				select resolution
**********************************************************/
void f_AnalogSetup(uint8_t channel_ADC, uint8_t channel_num,uint8_t resolution){
	if (channel_ADC == 0){														// ADC0
		// config clock for timer select SIRCDIV2
		f_SetMultiBit((uint32_t*)(SCG_BASE|0x204),8u,0x1,MASK_3BIT);			// set SIRDCDIV2_CLK div1 equal 8Mhz

		/* step 1: setting PCC */
		f_SetMultiBit(&PCC->mADC0,24u,0x2,MASK_3BIT);								// use SIRCDIV2_CLK
		f_SetClearBit(&PCC->mADC0,30u,HIGH);											// enable clock
		/* step 2: config ADC clock */		// in CFG1 register
		f_SetMultiBit((uint32_t*)(ADC0_BASE|0x40),0u,0x0,MASK_2BIT);			// input ALTCLK_1
		f_SetMultiBit((uint32_t*)(ADC0_BASE|0x40),5u,0x0,MASK_2BIT);			// CLK divide select
		
		/* step 3: config scale */			
		f_SetClearBit((uint32_t*)(ADC0_BASE|0),6u,HIGH);							// use interrupt
		
		f_SetMultiBit((uint32_t*)(ADC0_BASE|0x40),2u,resolution,MASK_2BIT);	// set resolution
		f_SetMultiBit((uint32_t*)(ADC0_BASE|0x44),0u,13u-1u,MASK_3BIT);		// set sample time in CFG2
		/* step 4: config mode */		
		f_SetClearBit((uint32_t*)(ADC0_BASE|0x90),6u,LOW);							// select SW 	// in SC2 register
		f_SetClearBit((uint32_t*)(ADC0_BASE|0x94),3u,HIGH);						// in SC3 register
		/* step 5: config channel and start */
		f_SetMultiBit((uint32_t*)(ADC0_BASE|0),0u,channel_num,0x3F);			// select SC1 channel num to input Analog
	}																			
	else{																							// ADC1
		// config clock for timer select SIRCDIV2
		f_SetMultiBit((uint32_t*)(SCG_BASE|0x204),8u,0x1,MASK_3BIT);			// set SIRDCDIV2_CLK div1 equal 8Mhz

		/* step 1: setting PCC */
		f_SetMultiBit((uint32_t*)(PCC_BASE|0x9C),24u,0x2,MASK_3BIT);			// use SIRCDIV2_CLK
		f_SetClearBit((uint32_t*)(PCC_BASE|0x9C),30u,HIGH);						// enable clock
		/* step 2: config ADC clock */		// in CFG1 register
		f_SetMultiBit((uint32_t*)(ADC1_BASE|0x40),0u,0x0,MASK_2BIT);			// input ALTCLK_1
		f_SetMultiBit((uint32_t*)(ADC1_BASE|0x40),5u,0x0,MASK_2BIT);			// CLK divide select
		/* step 3: config scale */			
		f_SetMultiBit((uint32_t*)(ADC1_BASE|0x40),2u,resolution,MASK_2BIT);	// set resolution
		f_SetMultiBit((uint32_t*)(ADC1_BASE|0x44),0u,13u-1u,MASK_3BIT);		// set sample time in CFG2
		/* step 4: config mode */		
		f_SetClearBit((uint32_t*)(ADC1_BASE|0x90),6u,LOW);							// select SW 	// in SC2 register
		f_SetClearBit((uint32_t*)(ADC1_BASE|0x94),3u,LOW);							// in SC3 register
		/* step 5: config channel and start */
		f_SetMultiBit((uint32_t*)(ADC1_BASE|0),0u,channel_num,0x3F);			// select channel num to input Analog
	}
}

/* note code in main with ADC peripheral*/
//		if(!g_state) f_OffRedLed();
//		else			 f_OnRedLed(); 
//		if(g_ADCvalue>4095/2) f_OnRedLed();
//		else 						 f_OffRedLed();
//		uint32_t volatile flag_COCO = ((*(uint32_t*)(ADC0_BASE|0x00))&(1<<7));			// flag COCO in SC1
//		while(!flag_COCO);											// wait to flag COCO completed	
//		g_ADCvalue = (*((uint32_t*)(ADC0_BASE|0x48))); 		// read value in SE12
		
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
			function set up SPI0
**********************************************************/
void f_InitSPI0(void){		// use as master
	uint32_t Reg_TCR = 0;										// TCR
	
	uint32_t* PTB2 = (uint32_t*)(PORTB_BASE|2*4);		// SCK
	uint32_t* PTB3 = (uint32_t*)(PORTB_BASE|3*4);		// SIN
	uint32_t* PTB1 = (uint32_t*)(PORTB_BASE|1*4);		// SOUT
	uint32_t* PTB0 = (uint32_t*)(PORTB_BASE|0*4);		// PCS0
	/* set select PIN - port */
	f_SetClearBit(&PCC->mPORTB,30u,HIGH);					// enable clock port
	
	f_SetMultiBit(PTB2,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB3,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB1,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB0,8u,0x3, MASK_3BIT);
	
	/* set PCC for SPI */
	f_SetMultiBit(&PCC->mLPSPI0,24u,0x3,MASK_3BIT);	   // select FIRCDIV2 48Mhz
	f_SetClearBit(&PCC->mLPSPI0,30u,HIGH); 

	
	/* setting clock */
	Reg_TCR |= 0x3<<27;											// prescale div 8
	f_SetMultiBit(&LPSPI0->CCR,0u,0x4,0xf);				// SCK div 4 + 2
//	Reg_TCR |= 0x1<<27;											// prescale div 2
//	f_SetMultiBit(&LPSPI0->CCR,0u,0x3,0xf);				// SCK div 3
	
	/* config pol and phase */
	Reg_TCR |= 0<<31;												// cphase
	Reg_TCR |= 0<<30;												// cpol
	
	/* setting frame data */
	Reg_TCR |= 1<<23;												// LSB first
	Reg_TCR |= 0x7;												// FRAME 8bit

	/* config chip select */
	Reg_TCR |= 0x0<<24;											// PCS0
	
	LPSPI0->TCR = Reg_TCR;										// assign value for TCR 
	
	/* setting Tx/Rx FIFO */
	f_SetMultiBit(&LPSPI0->FCR,0u,0u,MASK_2BIT);			// 1 byte TX
	f_SetMultiBit(&LPSPI0->FCR,16u,0u,MASK_2BIT);		// 0 byte RX
	f_SetClearBit(&LPSPI0->CFGR1,3u,HIGH);					// NOSTALL

	/* config LPSPI mode */
	f_SetClearBit(&LPSPI0->CFGR1,0u,HIGH);					// as master
	f_SetClearBit(&LPSPI0->CR,3u,HIGH);						// enable debugging

	/* enable module SPI */
	f_SetClearBit(&LPSPI0->CR,0u,HIGH);			

}
/* --------------------------------------------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
			function set up SPI1
**********************************************************/
void f_InitSPI1(void){		// use as slave
	uint32_t Reg_TCR1 = 0;										// TCR register
	
	uint32_t* PTB14= (uint32_t*)(PORTB_BASE|14*4);		// SCK	
	uint32_t* PTB15 = (uint32_t*)(PORTB_BASE|15*4);		// SIN
	uint32_t* PTB16 = (uint32_t*)(PORTB_BASE|16*4);		// SOUT
	uint32_t* PTB17 = (uint32_t*)(PORTB_BASE|17*4);		// PCS3
	/* set select PIN - port */
	f_SetClearBit(&PCC->mPORTD,30u,HIGH);
	
	f_SetMultiBit(PTB14,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB15,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB16,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB17,8u,0x3, MASK_3BIT);
	
	/* set PCC for SPI */
	f_SetMultiBit(&PCC->mLPSPI1,24u,0x3,MASK_3BIT);	// select FIRCDIV2 48Mhz
	f_SetClearBit(&PCC->mLPSPI1,30u,HIGH);

	
	/* setting clock */
	Reg_TCR1 |= 0x3<<27;											// prescale div 8
	f_SetMultiBit(&LPSPI1->CCR,0u,0x4,0xf);				// SCK div 4 + 2
	
	/* config pol and phase */
	Reg_TCR1 |= 0<<31;				// cphase
	Reg_TCR1 |= 0<<30;				// cpol
	
	/* setting frame data */
	Reg_TCR1 |= 1<<23;
	Reg_TCR1 |= 0x7;

	/* config chip select */
	Reg_TCR1 |= 0x3<<24;										// PCS0
	
	LPSPI1->TCR = Reg_TCR1;

	/* enable receive interrupt */
	f_SetClearBit(&LPSPI1->IER,1u,HIGH);

	/* setting Tx/Rx FIFO */
	f_SetMultiBit(&LPSPI1->FCR,0u,0u,MASK_2BIT);			// 0 byte TX
	f_SetMultiBit(&LPSPI1->FCR,16u,0u,MASK_2BIT);		// 0 byte RX
	f_SetClearBit(&LPSPI1->CFGR1,3u,HIGH);					// NOTSTALL

	/* config LPSPI mode */
	f_SetClearBit(&LPSPI1->CFGR1,0u,LOW);					// as slave
	f_SetClearBit(&LPSPI1->CR,3u,HIGH);						// enable debugging

	/* enable module SPI */
	f_SetClearBit(&LPSPI1->CR,0u,HIGH);			
}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
			function set up SPI for IC MAX7219
**********************************************************/
void f_InitSPI_max7219(void){		// use as master
	uint32_t Reg_TCR = 0;										// TCR register
	
	uint32_t* PTB2 = (uint32_t*)(PORTB_BASE|2*4);		// SCK
	uint32_t* PTB3 = (uint32_t*)(PORTB_BASE|3*4);		// SIN
	uint32_t* PTB1 = (uint32_t*)(PORTB_BASE|1*4);		// SOUT
	uint32_t* PTB0 = (uint32_t*)(PORTB_BASE|0*4);		// PCS0
	/* set select PIN - port */
	f_SetClearBit(&PCC->mPORTB,30u,HIGH);					// enable clock port
	f_SetMultiBit(PTB2,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB3,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB1,8u,0x3, MASK_3BIT);
	f_SetMultiBit(PTB0,8u,0x3, MASK_3BIT);
	
	/* set PCC for SPI */
	f_SetMultiBit(&PCC->mLPSPI0,24u,0x3,MASK_3BIT);		// select FIRCDIV2 48Mhz
	f_SetClearBit(&PCC->mLPSPI0,30u,HIGH);

	
	/* setting clock */
	Reg_TCR |= 0x1<<27;											// prescale div 2
	f_SetMultiBit(&LPSPI0->CCR,0u,1u,0xf);					// SCK div 1 + 2
	
	/* config pol and phase */
	Reg_TCR |= 0<<31;												// cphase
	Reg_TCR |= 0<<30;												// cpol
	
	/* setting frame data */
	Reg_TCR |= 0<<23;												// MSB first
	Reg_TCR |= 0xE;												// FRAME 16bit

	/* config chip select */
	Reg_TCR |= 0x0<<24;											// PCS0
	
	LPSPI0->TCR = Reg_TCR;										// assign value for TCR 
	
	/* setting Tx/Rx FIFO */
	f_SetMultiBit(&LPSPI0->FCR,0u,0u,MASK_2BIT);			// 0 byte TX
	f_SetMultiBit(&LPSPI0->FCR,16u,0u,MASK_2BIT);		// 0 byte RX
	f_SetClearBit(&LPSPI0->CFGR1,3u,HIGH);					// NOSTALL

	/* config LPSPI mode */
	f_SetClearBit(&LPSPI0->CFGR1,0u,HIGH);					// as masterW
	f_SetClearBit(&LPSPI0->CR,3u,HIGH);						// enable debugging

	/* enable module SPI */
	f_SetClearBit(&LPSPI0->CR,0u,HIGH);			

}
/* --------------------------------------------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
			start up for IC MAX7219
**********************************************************/
void f_SetupMAX7219(void){								// use SPI0 
	// decoder MODE all led
	f_DigitalWrite(PORTB,PIN0,LOW);					// chip select
	f_SentMAX7219(0x09FF);								// address
	f_DigitalWrite(PORTB,PIN0,HIGH);

	//brightness setting
	f_DigitalWrite(PORTB,PIN0,LOW);
	f_SentMAX7219(0x0A04);							// 9/32	
	f_DigitalWrite(PORTB,PIN0,HIGH);

	// scan limit
	f_DigitalWrite(PORTB,PIN0,LOW);
	f_SentMAX7219(0x0B07);							// all led	
	f_DigitalWrite(PORTB,PIN0,HIGH);

	// display test
	f_DigitalWrite(PORTB,PIN0,LOW);
	f_SentMAX7219(0x0F00);							// normal
	f_DigitalWrite(PORTB,PIN0,HIGH);
	
	// shutdown MODE
	f_DigitalWrite(PORTB,PIN0,LOW);
	f_SentMAX7219(0x0C01);							// on led
	f_DigitalWrite(PORTB,PIN0,HIGH);
	
}
/* --------------------------------------------------------------------------------------------------------------- */










/* --------------------------------------------------------------------------------------------------------------- */
/********************* STEP TO FOLLOW ********************* 
			string to save value
			enter ADC value
			enter value max of Vref
**********************************************************/
//void f_ProcessADC(char* g_String,uint32_t ADCValue, uint8_t Vref){
//	uint32_t VolValue = ((float)ADCValue/0xFFF)*100*Vref;					// 5V Vref
//	
//	g_String[0] = 48 + (char)(VolValue/100);					
//	g_String[1] = '.';
//	g_String[2] = 48 + (char)(VolValue%100/10);				
//	g_String[3] = 48 + (char)(VolValue%10);
//	g_String[4] = 0x20; 		// space
//	g_String[5] = 'V';
//	g_String[6] = '\n';
//	g_String[7] = '\0';		// end string
//}
/* --------------------------------------------------------------------------------------------------------------- */




	
	/* if(!g_type){									// type 24h
		g_Time[0] = '0' + (g_hour/10);
		g_Time[1] = '0' + (g_hour%10);
		g_Time[2] = ':';
		g_Time[3] = '0' + (g_minute/10);
		g_Time[4] = '0' + (g_minute%10);
		g_Time[5] = ':';
		g_Time[6] = '0' + (g_second/10);
		g_Time[7] = '0' + (g_second%10);
		g_Time[8] = '\n';
		g_Time[9] = '\0';							// end string 
	}	
	else{ 											// type 12h
		if(g_hour>=12){
			g_Time[9] = 'P';						// set 'P' for "PM"
			if(g_hour>12){
				g_Time[0] = '0' + ((g_hour%12)/10);
				g_Time[1] = '0' + ((g_hour%12)%10);
			}
			else{
				g_Time[0] = '0' + ((g_hour)/10);
				g_Time[1] = '0' + ((g_hour)%10);
			}
		}
		else{	
			g_Time[9] = 'A';						// set 'A' for "AM"
			g_Time[0] = '0' + ((g_hour%12)/10);
			g_Time[1] = '0' + ((g_hour%12)%10);
		}	
		g_Time[2] = ':';
		g_Time[3] = '0' + (g_minute/10);
		g_Time[4] = '0' + (g_minute%10);
		g_Time[5] = ':';
		g_Time[6] = '0' + (g_second/10);
		g_Time[7] = '0' + (g_second%10);
		g_Time[8]= 0x20;							// space symbol
		g_Time[10] = 'M';
		g_Time[11]= '\n';
	} */
//}

