#ifndef 	_FUNC_S32K1xx_H_
#define		_FUNC_S32K1xx_H_
#include <stdint.h>					// for use data type


/* Private function prototypes ------------------------------------------------------------------*/

void f_SetClearBit(uint32_t* nAddress, uint8_t nPosition, uint8_t cSel);


void f_SetMultiBit(uint32_t* nAddress, uint8_t nPos, uint32_t nValue,  uint32_t nMask);


void f_TogglePin(uint32_t* GPIOx_PDDR, uint32_t* GPIOx_PTOR);


void f_PinMode(uint8_t PORTx, uint8_t PinX, uint8_t setIO);


void f_DigitalWrite(uint8_t PORTx, uint8_t PinX, uint8_t status);


void f_GPIOPinInterrupt(uint8_t PORTx, uint8_t PinX, uint8_t modeInterrupt);


void f_SysTickInit(uint32_t ValueSet);


void f_SetNVIC(uint8_t NVICID);


//uint8_t f_DigitalRead(uint8_t PORTx, uint8_t PinX);


void f_InitLPUART1(uint32_t baudrate);
void f_InterruptLPUART1(uint32_t baudrate);

//void f_InitLPUART(uint32_t baudrate);
void f_InterruptLPUART2(uint32_t baudrate);

void f_SentSymbol(char symbol);
void f_SentStringUART(char* string);


uint8_t f_CompareString(char* receiveData, char* valueString);


void f_delayms(uint32_t ms);
void f_delayMillis(void);
void f_delayMs(uint32_t time);


void f_InitLPIT(void);
void f_InterruptLPIT(uint16_t millisecond);




void f_AnalogSetup(uint8_t channel_ADC, uint8_t channel_num,uint8_t resolution);
void f_ProcessADC(char* string, uint32_t ADCValue, uint8_t Vref);	


void f_InitSPI0(void);
void f_InitSPI1(void);
void f_InitSPI_max7219(void);

void f_SetupMAX7219(void);

void f_SentSymbolSPI(char symbol);
void f_SentMAX7219(short symbol);
void f_SentStringSPI(char* string);






/* Extern function of Start file interrupt ------------------------------------------------------------------*/
extern void SysTick_Handler(void);
#endif
