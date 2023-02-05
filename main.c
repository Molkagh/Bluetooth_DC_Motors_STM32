#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
char BUFFER[50];

int ADC_VALUE[3];
int i ;
int j;
float k;

void Systemclock()
{
	// HSE_ON
  RCC->CR |= 0x00010000; //17ème bit dans le registre CR du périphérique RCC à 1,

  // wait until HSE READY
  while (!(RCC->CR & 0x00020000));

  //Select HSE as system clock
  	RCC->CFGR = 0x00000001;

}

void config_ADC ()
{
	//ENABLE ADC 8éme bit
		RCC -> APB2ENR=1<<8;

		//  prescaler=8 (bit 16 & 17)
		ADC->CCR |=3<<16;

		//ADC mode multi_channels 8éme bit
		ADC1->CR1|=1<<8;

		//ADC mode DISCONT_ENABLE (bit 11)
		ADC1->CR1|=1<<11;

		//DISCONT_num=3(3 channels)(bits 13 14 15)
		ADC1->CR1|=0<<13;

		//CH1->CH3->CH4(l'ordre de conversion)
		ADC1->SQR3=0x1061;

		//Le groupe (3 CHANNEL)
		ADC1->SQR1=2<<20;


		//config de flag (bit 10) ACTIVER L'INTERRUPTION ADC
		ADC1->CR2|=1<<10;


		//EOCIE=1 AUTORISATION interrupt(bit 5)
		ADC1-> CR1 |=1<<5;

		//Enable request  ACTIVER L'INTERRUPTION ADC DANS LE NVIC
		NVIC_EnableIRQ(ADC_IRQn);

		//activation et config GPIOA(liaison l'adc GPIOA) en entrée ADC

		//activation GPIOA(bit 0)
	 	RCC->AHB1ENR|=1;

	 	//config PA1,PA3 et PA4 EN MODE Analogique
	 	GPIOA->MODER|=0X3CC;


	 	//ACTIVATION CLOCK TIM2
	     RCC->APB1ENR |=0x1;

	   	//TIM2 PRESCALER
	     TIM2->PSC = 7999;

	   	 // Set timer reload value, update event
	   	 TIM2->ARR = 999;

	   	 // Select update event as trigger output (TRGO)
	   	 TIM2->CR2 |= 0x20;

	   	// External trigger enable for injected channels
	   	 ADC1->CR2 |= 1<<28;

	   	// Select Timer 2 TRGO event as external event for injected group
	   	 ADC1->CR2 |= 0x6<<24;

	   	//ADON(ACTIVATIN ADC1)
	   	 ADC1->CR2|=0X1;

}
void config_EXTI()
{
	// syst config=1 bit 14 ce qui active l'horloge du module de configuration système (SYSCFG)
	RCC->APB2ENR |=1<<14;

	//1er entrée dans le tableau EXTICR du SYSCFG=l'entrée EXTI0 est connectée à la source GPIOA
	SYSCFG->EXTICR [0] =0x0000;

	// activer la détection de front montant (bit0=1 == front montant)
	EXTI->RTSR=0X1;

	// autoriser les interruptions sur EXTI0
	EXTI-> IMR =0x1;

	// Activer l'interruption
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void config_usart()
{
	// Enable clock for USART3
	  RCC->APB1ENR|=0x00040000;

  //USART3 enable(bit0->5| bit8->9|bit14->15)Activer TX et Rx pour synch d'horloge liaison serie
	    USART3->CR1|=0x0000203C;

	 // BaudRate:USART_DIV=8MHZ/9600=833 (Taux de Baud: la vitesse de transmission de données )
	    USART3->BRR=833;

	 // Enable clock for GPIOB
	    RCC->AHB1ENR |= 0x00000002;

	 // configuring the USART3 ALTERNATE function  to PB10 and PB11
	    GPIOB->MODER|=0x2AA00000;

	 //enable USART3_TX to PB10 and USART3_RX to PB11 (liaison usart3 et GPIOB)
	      GPIOB->AFR[1]=0x00007700;

}

void config_TIM3()
{

	 // Enable TIM3 clock (RCC_APB1ENR[1]=1)
	 RCC->APB1ENR |= 0x1<<1;//le bit [0] du registre APB1ENR

	 // Set prescaler to 7999
	 TIM3->PSC = 7; // prescaler=8000 sachant syst clk=HSE  => counter clk= 1000Hz

	 // Set ARR to 999
	 TIM3->ARR = 999; // le compteur compte de 0 à 999

	 //ENABLE counter CCR=<ARR , de CH1,CH2,CH3
	 TIM3->CCER|=0x111;

	 // CH1,CH2 output mode 1
	 TIM3->CCMR1|=0x6060;

	 // CH3 output mode 1
	 TIM3->CCMR2|=0x60;

	 k=0.243;//(k=ARR/4095)
		 	TIM3->CCR1=0;
		 	TIM3->CCR2=0;
		 	TIM3->CCR3=0;

		 	//activation GPIOA(bit 0)
		 	RCC->AHB1ENR|=1;

		 	//PA6 et PA7 en mode alternate function
		 	 GPIOA->MODER|=0xA000;

	 //ACTIVATION CLOCK GPIOB <-------------------------------------UP DATE
	 RCC->AHB1ENR |= 1<<1;

	 //CONFIG PIN0 GPIOB EN MODE ALTERNATE FUNCTION <------------------------UPDATE
	 	 GPIOB->MODER|=0x2;


	 // lier TIM3 CH1(AF2) à PA6 et CH2(AF2) à PA7 connectée
	 GPIOA->AFR[0]|=0x22000000;


	 // lier TIM3 CH3 (AF2) à PB0
	 GPIOB->AFR[0]|=0x2;


	 // ENABLE counter CNT=ARR bit tim2[0]= CEN=1
	TIM3->CR1|=1;

}
void SendChar(char Tx)
{
	while((USART3->SR&0x80)==0);  // On attend à ce que TXBUFF soit vide (libere)ou(FLAG TXNE=1) ou Le caractere est envoyé
	USART3->DR=Tx;
}
void Sendstring(char *Adr)
{
  while(*Adr)
  {
    SendChar(*Adr);
    Adr++;
  }
}
int main(void)
{
	  Systemclock();
	  config_ADC();
	  config_EXTI();
	  config_usart();
	  config_TIM3();
	  Sendstring("MOLKA");


    while(1);
}

void EXTI0_IRQHandler()
 {
	if ((EXTI->PR &(0x1))!=0)
	{
		j++;
			 if(j==1)
			 {
				 TIM2->CR1 =1; //EN_TIMER=1 bit0
			 }
			 if(j==2)
			 {
				 j=0;
				 TIM2-> CR1 =0;  //EN_TIMER=0
			 }
			 //clear flag
			 EXTI->PR|= 0x1;
	}

 }
void ADC_IRQHandler(void)
{
	if((ADC1->SR&0x2)!=0) //TEST EOC
		 {

	  i++;
	  if(i==1)
	  {

		  ADC_VALUE[0]=ADC1->DR; // EOC is cleared when DR is read
	  }
	  if(i==2)
	  	  {
	  		  ADC_VALUE[1]=ADC1->DR;
	  	  }
	  if(i==3)
	  	  {
		      i=0;
	  		  ADC_VALUE[2]=ADC1->DR;

	  		TIM3->CCR1= 0.243*ADC_VALUE[0];
	  		TIM3->CCR2= 0.243*ADC_VALUE[1];
	  		TIM3->CCR3= 0.243*ADC_VALUE[2];


	  		  sprintf(BUFFER,"CH1=%d,CH2=%d,CH3=%d",ADC_VALUE[0],ADC_VALUE[1],ADC_VALUE[2]);
	  	      Sendstring(&BUFFER);
	  	  }

		 }
}



