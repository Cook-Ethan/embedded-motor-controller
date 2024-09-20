#include "stm32l4xx.h" /* Microcontroller information */

/* Define constant variables */
#define FILTER_SIZE 15

/* Define global variables */

// Counter global variables
unsigned static char counter_state;
unsigned static char counter_up;
unsigned static char count;

// PID controller global variables
static unsigned int speed_desired;
static unsigned int speed_actual;
volatile static unsigned int adc_in;
static float k_p = 0.9f;
static float k_i = 0.05f;
static float k_d = 0.005f;
static float integral;
static float last_error;
static unsigned int adc_filter_arr[FILTER_SIZE] = {0};
static int filter_i;

static double output;

struct {	// Setting up the structure of the keypad interface
	int row, col;
	unsigned char event;
	const unsigned char row1[4];
	const unsigned char row2[4];
	const unsigned char row3[4];
	const unsigned char row4[4];
	const unsigned char* map[];
} typedef keypad;

static keypad kp;

/* Function Prototypes */
void delay(void);
void shortDelay(void);
void PinSetupForRow(void);
void PinSetupForCol(void);
void PinSetup(void);
void InterruptSetup(void);
void Timer6Setup(void);
void Timer7Setup(void);
void PWMSetup(void);
void ADCSetup(void);
int readRow(void);
int readCol(void);
void displayLeds(unsigned char value);
void handleKeypad(void);
void EXTI0_IRQHandler(void);
void feedback_control(void);
unsigned int filterADC(unsigned int value);
void TIM6_IRQHandler(void);
void TIM7_IRQHandler(void);

static keypad kp = {	// Defining the characteristics of the keypad interface
	.row = ~0,
	.col = ~0,
	.event = 0,
	.row1 = {0x1, 0x2, 0x3, 0xA},
	.row2 = {0x4, 0x5, 0x6, 0xB},
	.row3 = {0x7, 0x8, 0x9, 0xC},
	.row4 = {0xE, 0x0, 0xF, 0xD},
	.map = {kp.row1, kp.row2, kp.row3, kp.row4},
};

/*----------------------------------------------------------*/
/* Delay function - do nothing for about 1 second */
/*----------------------------------------------------------*/
void delay (void) {
	int volatile i,j,n;
	for (i=0; i<20; i++) { //outer loop
		for (j=0; j<12000; j++) { //inner loop
			n = j; //dummy operation for single-step test
		} //do nothing
	}
}

/*----------------------------------------------------------*/
/* ShortDelay - do nothing for a short bit */
/*----------------------------------------------------------*/
void shortDelay (void) {
	
	int volatile i,j;
	for (i=0; i<20; i++) {
		j = i;	//dummy operation
	}
}

void PinSetupForRow (void) {
	/* Configure PA[5:2] as inputs and PA[11:8] as outputs */
	RCC->AHB2ENR |= 0x01;
	GPIOA->MODER &= ~(0x00FF0FF0);
	GPIOA->MODER |= (0x00550000);
	GPIOA->ODR = 0x0000;
	
	/* Configure PA[5:2] with pull-up resistors */
	GPIOA->PUPDR &= ~(0x00000FF0);
	GPIOA->PUPDR |= (0x00000550);
}

void PinSetupForCol (void) {
	/* Configure PA[5:2] as outputs and PA[11:8] as inputs */
	RCC->AHB2ENR |= 0x01;
	GPIOA->MODER &= ~(0x00FF0FF0);
	GPIOA->MODER |= (0x00000550);
	GPIOA->ODR = 0x0000;
	
	/* Configure PA[11:8] with pull-up resistors */
	GPIOA->PUPDR &= ~(0x00FF0000);
	GPIOA->PUPDR |= (0x00550000);
}

void PinSetup (void) {
	
	/* Configure PB0 as input */
	RCC->AHB2ENR |= 0x02;
	GPIOB->MODER &= ~(0x00000003);
	
	/* Configure PA0 as PWM output */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(0x00000003);
	GPIOA->MODER |= (0x00000002);
	GPIOA->AFR[0] &= ~(0x0000000F);	// Configure GPIOA pin 0 mode for AF
	GPIOA->AFR[0] |= (0x00000001);
	
	/* Configure PA1 as ADC input */
	GPIOA->MODER &= ~(0x0000000C);
	GPIOA->MODER |= (0x0000000C);
	
	PinSetupForRow();
	
	/* Configure PB[6:3] as output pins displaying counter value */
	GPIOB->MODER &= ~(0x00003FC0);
	GPIOB->MODER |= (0x00001540);
	
}

void InterruptSetup (void) {

	/* enable SYSCFG clock - only necessary for change of SYSCFG */
	RCC->APB2ENR |= 0x01; // Set bit 0 of APB2ENR to turn on clock for SYSCFG
	
	/* select PB0 as EXTI0 */
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;	// Clear EXTI0 bit in config reg
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;	// Select PB0 as interrupt source
	
	/* configure and enable EXTI0 as falling-edge triggered */
	EXTI->FTSR1 |= EXTI_FTSR1_FT0;	// EXTI0 = rising-edge triggered
	EXTI->PR1 = EXTI_PR1_PIF0;	// Clear EXTI0 pending bit
	EXTI->IMR1 |= EXTI_IMR1_IM0;	// Enable EXTI0
	
	/* Program NVIC to clear pending bit and enable EXTI0 -- use CMSIS functions */
	NVIC_ClearPendingIRQ(EXTI0_IRQn);	// Clear NVIC pending bit
	NVIC_EnableIRQ(EXTI0_IRQn);	// Enable IRQ
	
}

void Timer6Setup (void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
	
	// T_INT = (PSC + 1) * (ARR + 1)/F_CK_PSC
	TIM6->PSC = 3088;
	TIM6->ARR = 999;
	
	TIM6->CR1 |= 0x0001;
	
	counter_state = 0;
	counter_up = 1;
	count = 0;
	
	TIM6->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_IRQn);
}

void Timer7Setup (void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	
	// T_INT = (PSC + 1) * (ARR + 1)/F_CK_PSC
	TIM7->PSC = 48;
	TIM7->ARR = 999;
	
	TIM7->CR1 |= 0x0001;
	
	TIM7->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_IRQn);
	
}

void PWMSetup (void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;	// Enable timer 2 module
	
	TIM2->CR1 |= TIM_CR1_CEN;	// Enable timer 2 counter
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S);	// Enable timer 2 channel 1 output mode
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M);	// Enable output mode for PWM mode 1
	TIM2->CCMR1 |= (0x00000060);
	TIM2->CCER &= ~(TIM_CCER_CC1E);	// Enable timer 2 channel 1 output
	TIM2->CCER |= (TIM_CCER_CC1E);
	
	// T_INT = (PSC + 1) * (ARR + 1)/F_CK_PSC
	TIM2->PSC = 0;
	TIM2->ARR = 982;
	TIM2->CCR1 = 0;
}

void ADCSetup(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;	// Enable ADC clock
	
	RCC->CCIPR &= ~(RCC_CCIPR_ADCSEL);
	RCC->CCIPR |= (RCC_CCIPR_ADCSEL_0);
	RCC->CCIPR |= (RCC_CCIPR_ADCSEL_1); // Set ADC clock to system clock
	
	ADC1->CR &= ~ADC_CR_DEEPPWD;	// Wake up ADC
	
	ADC1->CR |= ADC_CR_ADVREGEN;	// Turn on ADC voltage regulator
	shortDelay();
	
	ADC1->CR |= ADC_CR_ADCAL;	// Start ADC calibration
	while (ADC1->CR & ADC_CR_ADCAL); // Check if calibration complete
	
	ADC1->CFGR |= ADC_CFGR_CONT;
	ADC1->CFGR |= ADC_CFGR_OVRMOD;	// Select continuous mode
	
	ADC1->SQR1 &= ~(ADC_SQR1_L); // Define sequence length to 1
	
	ADC1->SQR1 &= ~(ADC_SQR1_SQ1);
	ADC1->SQR1 |= (0x00000180);	// Assign channel number 6
	
	ADC1->CR |= ADC_CR_ADEN;	// Set ADC enable bit
	while (!(ADC1->ISR & 0x00000001));	// Check ADC ready bit
	ADC1->CR |= ADC_CR_ADSTART;	// Set ADC start bit
	
	integral = 0;
	last_error = 0;
	filter_i = 0;
	
}

/*----------------------------------------------------------*/
/* readRow function - reads which row keypad key is pressed */
/*----------------------------------------------------------*/
int readRow(void) {
	int row;
	
	PinSetupForRow();
	shortDelay();
	
	row = (GPIOA->IDR & 0x003C) >> 2;
	
	switch (row) {
		case 0xE:
			return 0;
		case 0xD:
			return 1;
		case 0xB:
			return 2;
		case 0x7:
			return 3;
		default:
			return -1;
	}
}

/*----------------------------------------------------------*/
/* readCol function - reads which column keypad key is pressed */
/*----------------------------------------------------------*/
int readCol(void) {
	int col;
	
	PinSetupForCol();
	shortDelay();
	
	col = (GPIOA->IDR & 0x0F00) >> 8;
	
	switch (col) {
		case 0xE:
			return 0;
		case 0xD:
			return 1;
		case 0xB:
			return 2;
		case 0x7:
			return 3;
		default:
			return -1;
	}
}

/*----------------------------------------------------------*/
/* Display LED's function - displays value on bits PB[6:3] */
/*----------------------------------------------------------*/
void displayLeds(unsigned char value) {
	
	GPIOB->BSRR |= (unsigned) (~value & 0x000F) << 19;	// reset !value to output
	GPIOB->BSRR |= (unsigned) (value & 0x000F) << 3;	// set value to output
}

/*----------------------------------------------------------*/
/* handleKeypad function - sets states of stopwatch */
/*----------------------------------------------------------*/
void handleKeypad(void) {
	kp.row = readRow();
	kp.col = readCol();
	
	if (kp.row != -1 && kp.col != -1) {
		if (kp.map[kp.row][kp.col] <= 0xA) {
			speed_desired = 0;
			TIM2->CCR1 = kp.map[kp.row][kp.col]*100;
		} else if (kp.map[kp.row][kp.col] == 0xE) {
			counter_state = (counter_state) ? 0 : 1;
			TIM6->CNT = 0;
		} else if (kp.map[kp.row][kp.col] == 0xF) {
			counter_up = (counter_up) ? 0 : 1;
			TIM6->CNT = 0;
		} else if (kp.map[kp.row][kp.col] == 0xD) {
			speed_desired = (speed_desired >= 0xC00) ? 0 : speed_desired + 0x300;
			last_error = 0;
			integral = 0;
		}
	}
}

/*----------------------------------------------------------*/
/* Interrupt Service Routine - handles interrupt */
/*----------------------------------------------------------*/
void EXTI0_IRQHandler(void) {
	
	handleKeypad();
	
	PinSetupForRow();
	
	delay();
	
	NVIC_ClearPendingIRQ(EXTI0_IRQn);	// Clear pending flag in NVIC
	EXTI->PR1 = EXTI_PR1_PIF0;	// Clear pending interrupt
}

/*----------------------------------------------------------*/
/* Feedback Control function - controls pwm to get fixed speed */
/*----------------------------------------------------------*/
void feedback_control(void) {
	float error, derivative;
	unsigned int output_mapped;
	
	speed_actual = adc_in;
	error = (float) speed_desired - (float) speed_actual;
	integral += error;
	derivative = error - last_error;
	last_error = error;
	
	output = k_p*error + k_i*integral + k_d*derivative;
	output = (output<0) ? 0 : (output>0xE00) ? 0xE00 : output;
	
	output_mapped = (unsigned int)((output * TIM2->ARR) / 0xE00);
	TIM2->CCR1 = output_mapped;
}

/*----------------------------------------------------------*/
/* Timer Interrupt Service Routine - handles timer interrupt */
/*----------------------------------------------------------*/
unsigned int filterADC(unsigned int value) {
	unsigned int min, i;
	
	adc_filter_arr[filter_i] = value;
	filter_i = (filter_i+1) % FILTER_SIZE;
	
	min = adc_filter_arr[0];
	for (i = 1; i < FILTER_SIZE; i++) {
		if (adc_filter_arr[i] < min)
				min = adc_filter_arr[i];
	}
	return min;
}

/*----------------------------------------------------------*/
/* Timer Interrupt Service Routine - handles timer interrupt */
/*----------------------------------------------------------*/
void TIM6_IRQHandler(void) {
	
	if (counter_state) {
		if (counter_up) {
			count = (count >= 9) ? 0 : count + 1;
		} else {
			count = (count <= 0) ? 9 : count - 1;
		}
	}
		
	displayLeds(count);
			
	TIM6->SR &= ~(TIM_SR_UIF);	// Clear pending timer interrupt
	NVIC_ClearPendingIRQ(TIM6_IRQn);	// Clear pending flag in NVIC
}

/*----------------------------------------------------------*/
/* Timer Interrupt Service Routine - handles timer interrupt */
/*----------------------------------------------------------*/
void TIM7_IRQHandler(void) {
	
	if (ADC1->ISR & ADC_ISR_EOC) {
		adc_in = ADC1->DR;
		adc_in = filterADC(adc_in);
		if (speed_desired) {
			feedback_control();
		}
	}
	
			
	TIM7->SR &= ~(TIM_SR_UIF);	// Clear pending timer interrupt
	NVIC_ClearPendingIRQ(TIM7_IRQn);	// Clear pending flag in NVIC
}

/*------------------------------------------------*/
/* Main program */
/*------------------------------------------------*/
int main(void) {
	
	PinSetup(); // Configure GPIO pins
	InterruptSetup(); // Configure Interrupt Service Routine (keypad interrupt)
	Timer6Setup(); // Configure Timer 6 Service Routine (counter interrupt)
	Timer7Setup(); // Configure Timer 7 Service Routine (ADC interrupt)
	PWMSetup(); // Configure Pulse-Width Modulation
	ADCSetup(); // Configure Analog-To-Digital input
	
	// TimerSetup(); //Configure Timer Interrupt
	
	__enable_irq();
	
	
	/* Endless loop */
	while (1) {
	} /* repeat forever */
}
