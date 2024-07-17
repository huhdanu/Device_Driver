/****************************************************************/
/**
 *  file        main.c
 *  containts   Run application code
 */
/****************************************************************/

/****************************************************************/
/*  Includes section                                            */
/****************************************************************/
#include 	"../Lib/s32k1xx.h"				// to use macro address
#include 	"../Lib/func_s32k1xx.h"			// to use function
#include 	<string.h>


/****************************************************************/
/*  Types And Constants                                         */
/****************************************************************/
#define LEDRED						15					// red led for PTD15
#define LEDBLUE					0					// blue led for PTD0
#define LEDGREEN					16					// green led for PTD16
#define SW3							13					// button for PTC13
#define SW2							12					// button for PTC12


// macro for read data
#define PORTC_PCR_PIN12   		(*(volatile unsigned int*)(0x4004B000u + 0x30))
#define PORTC_PCR_PIN13			(*(volatile unsigned int*)(0x4004B000u + 13*4))

#define FORWARDSTATE				((g_state==0) && (g_convert==0))				// forward is: green->red->blue
#define REWARDSTATE				((g_state==0) && (g_convert==1))				// reward is: 	blue->red->green
#define STOP						(g_state == 1)
#define EMPTYFLAG					((LPUART1->STAT)&(1<<23))


/****************************************************************/
/*  extern function for NVIC                                    */
/****************************************************************/
extern void PORTC_IRQHandler(void);
extern void LPUART1_RxTx_IRQHandler(void);
extern void LPUART2_RxTx_IRQHandler(void);
extern void LPIT0_Ch0_IRQHandler(void);
extern void LPSPI1_IRQHandler(void);
extern void ADC0_IRQHandler(void);

/****************************************************************/
/*  Internal function prototype                                 */
/****************************************************************/
//static void f_OnRedLed(void);										// to on RED LED
//static void f_OffRedLed(void);
//static void f_OnBlueLed(void);										// to on BLUE LED
//static void f_OnGreenLed(void);									// to on GREEN LED
//static void f_OffGreenLed(void);									// to off GREEN LED

static void f_ADC4MAX7219(uint32_t ADCValue, uint8_t Vref);


/****************************************************************/
/*  Global variable			                                     */
/****************************************************************/
extern volatile char g_Time[12];
extern volatile char g_Date[12];
extern char g_type;
extern char g_second;

//static uint8_t g_state = 0;
		
static volatile uint8_t index = 0;										// for count members in data receive UART
//static uint32_t g_ADCvalue;												// use read data in ADC
//static char g_String[8];													// use for ADC
static char g_PowerMAX7219 = 1;

char g_RCF = 0;																// Receive Complete Flag in Slave
static char g_flagDateUART = 0;											// default display time initial
static char g_flagReceive = 0;				
static char g_flagUpdate = 0;												// flag for update time and date

/* main function -------------------------------------------------------------------------------------------------*/
int main(void){
	
	// setup pin for led	PTD15 and PTD0 and PTD16
	f_PinMode(PORTD,LEDRED,OUTPUT);										// config red led
	f_PinMode(PORTB,0u,OUTPUT);
	
	// setup pin for button PTC13
	f_PinMode(PORTC, SW3,INPUT);
	f_PinMode(PORTC, SW2,INPUT);
	
	/* turn off for led connect with PTD15 and PTD0 */
	f_DigitalWrite(PORTD,LEDRED,HIGH);			
	
	// set for interrupt PTC12 and PTC13 use rising edge to interrupt
	f_GPIOPinInterrupt(PORTC,SW2,ISF_RE);						// pin 12
	f_GPIOPinInterrupt(PORTC,SW3,ISF_RE);						// pin 13
	
	// set NVIC in CORE
	f_SetNVIC(NVICID_PORTC);										// for port C			
	f_SetNVIC(NVICID_LPUART1);										// for UART 1
	f_SetNVIC(NVICID_LPUART2);										// for UART 0
	f_SetNVIC(NVICID_LPIT0);										// for Timer 0
	f_SetNVIC(NVICID_LPSPI1);										// for SPI 1
	//f_SetNVIC(NVICID_ADC0);											// for ADC0

	
	f_SysTickInit(1000);												// each 1000us ~ 1ms will interrupt	
	f_InterruptLPUART1(19200);										// enable Tx Rx use interrupt
	f_InterruptLPUART2(19200);
	//f_AnalogSetup(MODE_ADC0,SE12,ADC_12BIT);
	
	//f_InitLPIT();
	f_SentStringUART("Clock timer:\n\n");
	f_InterruptLPIT(1000);											// LPIT interrupt initial with each 1000ms will be interrupted
	
//	f_InitSPI0();			// master
//	f_InitSPI1();			// slave

	f_UpdateTime();
	f_InitSPI_max7219();
	f_SetupMAX7219();
	
	/* loop */
	while(1){
		if(g_flagUpdate){					// update time and date
			g_flagUpdate = 0;				// off flag
			f_UpdateTime();
			f_SentStringUART(g_Time);
		}
		else{
			if((!g_flagDateUART)){
				f_SentClockMAX7219(g_Time);
			}
			else if((g_flagDateUART)){		
				f_SentDateMAX7219(g_Date);	
			}
		}
	}
}
/* ---------------------------------------------------------------------------------------------------------------*/

/* interrupt handler ---------------------------------------------------------------------------------------------*/
void LPUART2_RxTx_IRQHandler(void){									// CP2101
	if(index==0){
		g_flagReceive = LPUART2->DATA;
		if(g_flagReceive == 'd')   memset(g_Date,0,12);
		if(g_flagReceive == 't')   memset(g_Time,0,12);
		index = 1;
	} 	
	
	char tempReceive;
	if(RIE_LPUART1){														// Rx interrupt 
		if(g_flagReceive == 'd'){
			tempReceive = LPUART2->DATA;
			if(tempReceive != '\\'){									// '\' is end symbol
				g_Date[index-2] = tempReceive;
				index++;
			}	
			else index = 0;												// reset value
		}
		else {
			tempReceive = LPUART2->DATA;
			if(tempReceive != '\\'){									// '\' is end symbol
				g_Time[index-2] = tempReceive;
				index++;
			}	
			else index = 0;												// reset value
		}
	}
}

void LPUART1_RxTx_IRQHandler(void){									// Open SDA 
	char tempReceive;
	if(index==0){
		g_flagReceive = LPUART1->DATA;
		if(g_flagReceive == 'd') memset(g_Date,0,12);
		if(g_flagReceive == 't') memset(g_Time,0,12);
		index = 1;
	} 	
	if(RIE_LPUART1){														// Rx interrupt 
		if(g_flagReceive == 'd'){
			tempReceive = LPUART1->DATA;
			if(tempReceive != '\\'){									// '\' is end symbol
				g_Date[index-2] = tempReceive;
				index++;
			}	
			else index = 0;												// reset value
		}
		else {
			tempReceive = LPUART1->DATA;
			if(tempReceive != '\\'){									// '\' is end symbol
				g_Time[index-2] = tempReceive;
				index++;
			}	
			else index = 0;												// reset value
		}
	}
}


// interrupt handler for TIMER overflow
void LPIT0_Ch0_IRQHandler(void){
	f_SetClearBit(&LPIT->MSR,0,HIGH);							// clear interrupt TIMER flag
	g_second++;
	g_flagUpdate = 1;													// turn on flag
}


void PORTC_IRQHandler(void){
	if(PORTC_PCR_PIN12 & (1<<24)){								// flag interrupt Pin12 ON
		PORTC_PCR_PIN12 |= (1<<24);								// set ISF PTC12 ON
		if((g_flagDateUART)) 			g_flagDateUART = 0;
		else if((!g_flagDateUART))	   g_flagDateUART = 1;
	}
	else if(PORTC_PCR_PIN13 & (1<<24)){							// flag interrupt Pin12 ON
		PORTC_PCR_PIN13 |= (1<<24);								// set ISF PTC12 ON
		if(g_PowerMAX7219){											// on
			g_PowerMAX7219 = 0;
			f_DigitalWrite(PORTB,PIN0,LOW);
			f_SentMAX7219(0x0C00);									// off led
			f_DigitalWrite(PORTB,PIN0,HIGH);
		}
		else{
			g_PowerMAX7219 = 1;
			f_DigitalWrite(PORTB,PIN0,LOW);
			f_SentMAX7219(0x0C01);									// on led
			f_DigitalWrite(PORTB,PIN0,HIGH);
		}
	}

}

//void ADC0_IRQHandler(void){
//	uint32_t ADCvalue = (*((uint32_t*)(ADC0_BASE|0x48)));
//	f_ADC4MAX7219(ADCvalue,15);
//} 

void f_ADC4MAX7219(uint32_t ADCValue, uint8_t Vref){
	uint8_t VolValue = ((uint32_t)ADCValue/0xFFF)*(100*Vref);		
	
	f_DigitalWrite(PORTB,PIN0,LOW);
	f_SentMAX7219(0x0A<<8|(VolValue));									// on led
	f_DigitalWrite(PORTB,PIN0,HIGH);
}

// interrupt for SPI receive data in slave
//void LPSPI1_IRQHandler(void){	
//	char getSymbol;
//	getSymbol = LPSPI1->RDR;
//	if(g_count == 0) 		memset(g_DataSPI,0,15);					// reset string receive
//	if(getSymbol != '\0'){
//		g_DataSPI[g_count] = getSymbol;
//		g_count++;
//	}
//	else g_count = 0;
//	g_RCF = 0;															// disable flag when receive complete	
//}
/* ---------------------------------------------------------------------------------------------------------------*/






/* on red led func */
//void f_OnRedLed(void){
//	f_DigitalWrite(PORTD,LEDRED, LOW);							// on red led
//	f_DigitalWrite(PORTD,LEDBLUE, HIGH);						// off blue led
//	f_DigitalWrite(PORTD,LEDGREEN, HIGH);						// off green led
//}

///* off red led func */
//void f_OffRedLed(void){
//	f_DigitalWrite(PORTD,LEDRED, HIGH);							// off red led
//	f_DigitalWrite(PORTD,LEDBLUE, HIGH);						// off blue led
//	f_DigitalWrite(PORTD,LEDGREEN, HIGH);						// off green led
//}


///* on green led func */
//void f_OnGreenLed(void){
//	f_DigitalWrite(PORTD,LEDRED, HIGH);							// off red led
//	f_DigitalWrite(PORTD,LEDBLUE, HIGH);						// off blue led
//	f_DigitalWrite(PORTD,LEDGREEN, LOW);						// on green led
//}

///* off green led func */
//void f_OffGreenLed(void){
//	f_DigitalWrite(PORTD,LEDRED, HIGH);							// off red led
//	f_DigitalWrite(PORTD,LEDBLUE, HIGH);						// off blue led
//	f_DigitalWrite(PORTD,LEDGREEN, HIGH);						// off green led
//}

