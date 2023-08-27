/*********************************************************************
 *      Example User Program to be used with resident Bootloader
 *********************************************************************
 * FileName:        main.c
 * Dependencies:    
 * Processor:       PIC18
 * Compiler:        C18 
 * Company:         
 * Autor:			
 *					
 *					Plantilla Generica para hacer un Programa en C18
 *					utilizando el Bootloader con el  PIC18F4550
 *********************************************************************/

#include <p18cxxx.h>
#include <math.h>

/****************  Bits de Configuracion *************************************/
//CONFIG1H 
#pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
#pragma config FOSC     = HSPLL_HS
//#pragma config CPUDIV   = OSC1_PLL2
#pragma config CPUDIV   = OSC1_PLL2	
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
//#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON		//USB Voltage Regulator
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = ON //Timer1 configured for low-power operation
#pragma config PBADEN   = OFF
#pragma config CCP2MX   = ON //CCP2 outpu RC1
//#pragma config CCP2MX = OFF  //CCP2 outpu RB3
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CP2      = OFF
#pragma config CP3      = OFF
#pragma config CPB      = OFF
#pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRT2     = OFF
#pragma config WRT3     = OFF
#pragma config WRTB     = ON       // Boot Block Write Protection
#pragma config WRTC     = OFF
#pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTR2    = OFF
#pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF

//*Este programa prueba el hardware de la tarjeta con el bootloader 
//#define PI 		3.14159265
//#define FREC	100.0
//#define FS		4000.0
//#define FS		50000.0
#define MAX_SAMPLES	512

#define L1              LATAbits.LATA4
#define L2              LATAbits.LATA5
#define L3              LATCbits.LATC0
#define L4              LATCbits.LATC1

/** ************  Entradas Digitales *****************************************************/
#define mInitAllSwitches()  TRISBbits.TRISB4=1;TRISBbits.TRISB5=1;
/*#define mInitSwitch3()      TRISBbits.TRISB4=1;
#define mInitSwitch2()      TRISBbits.TRISB5=1;
#define sw2                 PORTBbits.RB5
#define sw3                 PORTBbits.RB4
*/
//****************************************************************************************
/** V A R I A B L E S ********************************************************/
#pragma udata gpr0
	volatile unsigned short nMax= 0;
	volatile unsigned short N= 0;
	double FS= 300000.0;
	//volatile unsigned short div= 1;
	//volatile unsigned short divMax= 1;

	double f= 10000.0;
	typedef enum{
		F300KHZ, F50KHZ, F10KHZ, F2KHZ, F400HZ
	} FREQ;
//#pragma romdata

#pragma udata stk //gpr1
volatile far unsigned char func[MAX_SAMPLES];

//****************** DEFINICON DE ALGUNAS CONSTANTES *************************************

//****************************************************************************************

/*************  DECLARACION DE LOS PROTOTIPOS *******************************/
void YourHighPriorityISRCode(void);
void YourLowPriorityISRCode(void);
//void Mi_delay(void);
void EEPROM_Write(unsigned char address,unsigned char databyte);
void calFunc(double freq, float amp);
void setSampleFreq(FREQ f);
//****************************************************************************************


/******************** VECTOR  REMAPPING *******************************************/
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
/*********************************************************************************/
		extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
/*********************************************************************************/
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
/*********************************************************************************/
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
/*********************************************************************************/
/*********************************************************************************/


#pragma code
volatile unsigned short n=0;
///////////// MITAD DE MUESTRAS //////
//volatile unsigned char senCos[80]= {219,    223,    226,    228,    227,    225,    222,    217,    211,    204,    197,    190,    182,    175,    168,    162,    156,    152,    148,    146,    146,    146,    148,    152,    156,    162,    168,    175,    182,    190,    197,    204,    211,    217,    222,    225,    227,    228,    226,    223,    219,    212,    204,    194,    182,    169,    155,    141,    125,    110,     94,     79,     64,     51,     38,     27,     17,     10,      4,      1,      0,      1,      4,     10,     17,     27,     38,     51,     64,     79,     94,    110,    125,    141,    155,    169,    182,    194,    204,    212};
//********************  VECTOR DE ALTA PRIORIDAD   ************************************************
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		LATC ^= 0x02;
		//LATD  = (sin(2*pi*(f)*t)+1)/res;//senCos[n];
		//LATD= func[n];//((sin(2.0*3.14159*(100.0)*t)+1.0)/res);	
		LATD= func[n];
		
		/* divide arregro de 512 en dos de 256 
		if(n < 256)
			LATD= func[n];//div/divMax
		else
			LATD= func2[n-256]; 	 
		*/
		//div++;
		//if(div > divMax){
			n++;
			if(n > N) n= 0;
		//}
		
		PIR2bits.CCP2IF = 0;//Limpio bandera de interrupccion de CCP1
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
//****************************************************************************************


//********************  VECTOR DE INTERRUPCION BAJA PRORIDAD  ************************************************
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//PIR2bits.CCP2IF = 0;//Limpio bandera de interrupccion de CCP1
		//AGREGAR CODIGO
		return;
	}	
//****************************************************************************************

/************************  PROGRAMA PRINCIPAL *************************************/
/*********************************************************************************/
void main (void)
{
	ADCON1 |= 0x0F;                 // Default all pins to digital
	PORTC=0x000;
	TRISC=0B00000000;
	PORTD=0x000;
   	TRISD = 0x00; 	 
    mInitAllSwitches();
	
	PIR2bits.CCP2IF = 0;//Limpio bandera de interrupccion de CCP1
	PIE2bits.CCP2IE= 1;	//Interrupciones habilitadas para CCP1
	IPR2bits.CCP2IP= 1;	//Interrupcion de alta prioriada en CCP1
	
	INTCONbits.TMR0IF =	0;	//Apaga bandera de overflow del timer0
	INTCONbits.GIE=1; //Habilita interrupciones globales
	INTCONbits.PEIE=1; //Habilita interrupciones Perifericos
		//PIR2bits.CCP2IF = 0;//Limpio bandera de interrupccion de CCP1

		
	T3CON = 0x40; //0xC0; //(16 bits reg., T3 clock source for CCP1 and CCP2, Fosc/4, Timer OF)
	TMR3H = 0x00;
	TMR3L = 0x00;
	
	CCP2CON= 0x0B;//0000 1011  //0x02; // 0000 0010
	
	//CCP1 --> RC2 ,  CCP2 --> RB3 or RC1	
	//Configure I/O on PortC
				/*CCP2CON CCP2M3: CCPXM0
		0010 = Compare mode, toggle output on match
		1000 = Compare mode, initialize CCP1 pin low, set output on compare match (set CCP1IF)
		1001 = Compare mode, initialize CCP1 pin high, clear output on compare match (set CCP1IF)
		1010 = Compare mode, generate software interrupt only, CCP1 pin reverts to I/O state
	(*)	1011 = Compare mode, trigger special event (CCP1 resets TMR1 or TMR3, sets CCP1IF bit)
		*/

	/// Value for FS= 4 KHz
	//CCPR2H=	0x0B;
	//CCPR2L= 0xC4 ;

	/// Values for FS = 5 KHz with compensed
	//CCPR2H 	= 0x09;
	//CCPR2L	= 0x6A;

	/// FOR FS 200kHz -> F 8 Khz
	//CCPR2H 	= 0x00;
	//CCPR2L	= 0x3C;

	/// FOR FS 2 MHz 300 KHz (Debido a ciclo de instruccion)
	//CCPR2H 	= 0x00;
	//CCPR2L	= 0x01;
	setSampleFreq(F300KHZ);
	if( f < 600 && f >= 100)
		setSampleFreq(F50KHZ);
	else if(f < 100 && f >= 20)
		setSampleFreq(F10KHZ);
	else if(f < 20 && f >= 4 )
		setSampleFreq(F2KHZ);
	else if( f < 4)
		setSampleFreq(F400HZ);
	nMax =  ( FS/f);
	//if(nMax > 256){
		//divMax	=	nMax / 256;
		//nMax 	=	256;
	//}
	//nMax =  200;//(nMax> 256.0? 256: nMax);
	calFunc(f, 1.0);
	N= nMax-1;
	
	T3CONbits.TMR3ON= 1; //Enciende Timer3		
   while (1)
   {
		//_asm nop _endasm;
   }    	
}

void setSampleFreq(FREQ f){
	switch(f){
		case F300KHZ:
			/// FOR FS 2 MHz 300 KHz (Debido a ciclo de instruccion)
			CCPR2H 	= 0x00;
			CCPR2L	= 0x01;
			FS = 300000.0;

		break;
		case F50KHZ:
			CCPR2H 	= 0x00;
			CCPR2L	= 0xF1;
			FS = 50000.0;
		break;
		case F10KHZ:
			CCPR2H 	= 0x04;
			CCPR2L	= 0xB5;
			FS = 10000.0;
		break;
		case F2KHZ:
			CCPR2H 	= 0x17;
			CCPR2L	= 0x88;
			FS = 2000.0;
		break;
		case F400HZ:
			CCPR2H 	= 0x75;
			CCPR2L	= 0xA8;
			FS = 400.0;
		break;
		default: break;
	}
}


void calFunc( double freq, float amp)
{
	float res = ( (8.0)/255.0);
	//res = (amp/5.0)*res;	 
	double dt= (1.0/(nMax*freq));
	double t= 0.0;
	unsigned short i= 0;	

	for(i= 0; i < nMax; i++){
		func[i]= ((sin(2.0*3.1459*(freq*t))+1.0)/res);
		t= t+dt;
	}
}

/*********************************************************************************/

void EEPROM_Write(unsigned char address,unsigned char databyte) //Funci�n para escribir un databyte
{                                                               //en la localidad address de la EPROM
 EECON1bits.EEPGD=0;
 EECON1bits.CFGS=0;
 
 EEDATA=databyte;
 EEADR=address;
 EECON1bits.WREN=1;

 EECON2=0x55;
 EECON2=0xAA;
 EECON1bits.WR=1;

 while(EECON1bits.WR==1);

 EECON1bits.WREN=0;
}