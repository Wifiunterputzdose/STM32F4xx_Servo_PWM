/*
 * Blinking LED2 (green) with SysTick Timer!
 */
#include <stdint.h>
#include "stm32f4xx.h"

#define GREENLED_PORT GPIOA
#define GREENLED_PIN 5
#define SERVO_0   58
#define SERVO_90 110
#define SERVO_180 0 //tbd

/*
 *  System CLock predefines
 *  HSE = 8 MHz - external oscillator used as clock source
 *  PLL:
 *  8/PLL_M*PLL_N/PLL_P = 8MHz/1*18/2 = 72MHz Sysclock (Systemclock)
 *  AHB Prescaler = 1
 *  HCLK = 72MHz
 *  APB1 prescaler = 2
 *  PCLK1, APB1 = 72MHz / 2 = 36MHz
 *
 *  AHB/APB1 - TIM2,3,4,5,6,7,12,13,14 72MHz
 *  AHB/APB2 - TIM1,8,9,10,11
 *
 */
#define PLL_M 	1
#define PLL_N 	18
#define PLL_P 	0  // PLLP = 2


volatile uint32_t  ticks = 0; // must be volatile to prevent compiler optimisations

void  SysTick_Handler(void)
{
    ticks++;
}

void delay_ms(int ms)
{
    uint32_t started = ticks;
    while((ticks-started)<=ms); // rollover-safe (within limits)
}

// delay loop for the default 8 MHz CPU clock with optimizer enabled
/*void delay(uint32_t msec)
{
    for (uint32_t j=0; j < msec * 2000; j++)
    {
        __NOP();
    }
}*/

void servo_set_pos(uint8_t pos) {
  uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
  TIM3->CCR1 = SERVO_0 + tmp * pos;
}

int main(void)
{
	SystemInit();
	//RCC->CFGR |= (0x1UL << RCC_CFGR_SW_Pos); //HSE oscillator selected as system clock
	//System Clock Configuration
	//////////////////////////////////////////////////////////////////////
	RCC->CR |= (0x1UL << RCC_CR_HSEON_Pos); //Set HSE osci on
	while (!(RCC->CR & (1<<RCC_CR_HSERDY_Pos))); //wait until HSE is ready
	RCC->APB1ENR |= (1<<RCC_APB1ENR_PWREN_Pos); //Enable Power interface clock
	PWR->CR |= (0x3UL << PWR_CR_VOS_Pos); //Scale 1 mode <= 100MHz
	FLASH->ACR |= (1<<8) | (1<<9)| (1<<FLASH_ACR_DCEN_Pos) | (5<<FLASH_ACR_LATENCY_Pos);
	//RCC->CFGR |= (0x2UL << RCC_CFGR_SW_Pos) //PLL as system clock
	RCC->CFGR &= ~(1<<RCC_CFGR_HPRE_Pos); //AHB prescaler = 1 (system clock not divided
	RCC->CFGR |= (0x4UL << RCC_CFGR_PPRE1_Pos); //APB1 lowspeed prescaler = 2
	//RCC->CFGR &= ~(0x0UL << RCC_CFGR_PPRE2_Pos); //APB2 lowspeed prescaler = 1
	RCC->PLLCFGR |= (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16);
	RCC->PLLCFGR |= (1<<RCC_PLLCFGR_PLLSRC_Pos);
	RCC->CR |= (1<<RCC_CR_PLLON_Pos); //enable the PLL
	while (!(RCC->CR & (1<<RCC_CR_PLLRDY_Pos)));
	RCC->CFGR |= (2<<RCC_CFGR_SW_Pos); //PLL selected as system clock source
	while (!(RCC->CFGR & (2<<RCC_CFGR_SWS_Pos)));
	//////////////////////////////////////////////////////////////////////

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // set tick to every 1ms

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//RCC->AHB1ENR |= RCC_APB1ENR_TIM3EN; //RCC on for Timer 3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC on for GPIO A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //RCC on for GPIO B

	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER5,  0b01 << GPIO_MODER_MODER5_Pos); //PA5 LED2 (green)
	//MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7,  0b01 << GPIO_MODER_MODER7_Pos); //PA7 Servo Test Pin
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER4,  0b10 << GPIO_MODER_MODER4_Pos); //PB4 (D5) AF TIM3_CH1 PWM Output
	//GPIOB->MODER |= (0x2UL << GPIO_MODER_MODER4_Pos); //High speed
	//MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED4,  0b10 << GPIO_OSPEEDR_OSPEED4_Pos); //High speed
	GPIOB->OSPEEDR |= (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos);
	GPIOB->AFR[0] |= ( 0x2UL << GPIO_AFRL_AFSEL4_Pos );	//map TIM3_CH1 to PB4 : AF2 for PIN4 :pins 0 to 7 AFR[0] pins 8 to 15 AFR[1]

	//SysTick_Config(SystemCoreClock/1000); // set tick to every 1ms

	TIM3->PSC = 960-1; //960-1; 48MHz /PSC = //2880-1; 144MHz / PSC =  //720-1; //36MHz / PSC = 50kHz = 1/50kHz //1440-1; //72MHz/PSC = 50kHz = 1/50kHz = 20us
	TIM3->ARR = 1000;	//Period //SystemCoreClock/50;
	TIM3->CCR1 = 58; //PSC/fcclock = 960 / 48MHz = 20us -> 20us*75 = 1.5ms

	//1ms - 58 dec - 0°
	//1.5ms - 110 dec - 90°
	//2ms - 100

	/*
	 * CC1S = 0 - CC1 channel is configured as output
	 * OC1M = 0b111 PWM2 mode - in upcounting channel 1 is active as long TIMx_CNT < TIMx_CCR1
	 */
	TIM3->CCMR1 &= ~(0x3UL << TIM_CCMR1_CC1S_Pos); //clear all bits so that CC1S is output
	TIM3->CCMR1 |= (0x6UL << TIM_CCMR1_OC1M_Pos); //pwm mode 1 //output compare mode OCxM = 0b11, OCxPE=0b00, CCxP = 0b00, CCxE = 1
	TIM3->CCMR1 |= (0x1UL << TIM_CCMR1_OC1PE_Pos); //enable preload of CCR1 write CCR1 anytime
	//TIM3->CCMR1 &= ~(0x1UL << TIM_CCMR1_OC1PE_Pos); //enable preload of CCR1 write CCR1 anytime

	TIM3->CR1 |= (0x1UL << TIM_CR1_ARPE_Pos); //enable Auto-reload preload of
	/*
	 * CC1P = 0 - OC1 active high
	 */
	//TIM3->CCER &= ~(0x1UL << TIM_CCER_CC1P_Pos); //output polarity CCxP
	TIM3->CCER |= (0x1UL << TIM_CCER_CC1E_Pos);//enable capture compare
	//Edge aligned PWM 0 - max. value then restart by 0 again to max.

	//TIM3->CR1 = TIM_CR1_DIR//upcounting p.336 //CMS=0 Edge aligned mode, DIR=0 upcounter
	TIM3->CNT = 0; //reset cnt
	TIM3->CR1 |= (0x1UL << TIM_CR1_CEN_Pos);//TIM_CR1_CEN; //enable Counter

	//start at 0°
	TIM3->CCR1 = 58; //1.5ms Position "0°"
	int cnt=58;

	while(1)
	{

		cnt++;
		WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_5);
		//servo_set_pos(180);
		if (cnt > 999 || cnt == 0)
		{
			cnt=58;
		}


		TIM3->CCR1 = cnt; //1.5ms Position "0°"
		delay_ms(10);

		WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BR_5);

		//servo_set_pos(0);
		//TIM3->CCR1 = 200; //2ms Position "+90°"
		//delay_ms(1000);

		//TIM3->CCR1 = 100; //1.5ms Position "0°"
		//delay_ms(1000);


	}

}

