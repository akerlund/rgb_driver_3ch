#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "gamma.h"

#define RMAX 4096
#define GMAX 3500
#define BMAX 2500

#define led_2 GPIO12	//
#define led_5 GPIO15
#define led_6 GPIO3

#define led_1 GPIO11
#define led_3 GPIO13
#define led_4 GPIO14

#define adc_pot1 GPIO0
#define adc_pot2 GPIO4
#define adc_pot3 GPIO5

#define btn_1 GPIO1
#define btn_2 GPIO1


const uint16_t gamma_lut[4096] =  {GAMMA_LUT};
volatile uint16_t adc_samples[16];
static volatile int ms_time_delay;	//WTF!? Why do I have to use static!???
volatile uint8_t usart_rx_buffer[64];
volatile bool usart_recieved = false;


void sys_tick_handler(void){
	if (ms_time_delay) {
		ms_time_delay--;
	}
}


void sleep_ms(int t){
	ms_time_delay = t;
	while (ms_time_delay);
}


void setup_clock( ){

	rcc_clock_setup_in_hsi_out_24mhz( );

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);

	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_ADC1);

	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	
	rcc_periph_clock_enable(RCC_DMA1);
}

void setup_timers( ){

	// Tim2 ch2, ch3, ch4
	// Tim3 ch1, ch2, ch3, ch4
	// Tim4 ch2, ch3

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
					GPIO1 | GPIO2 | GPIO3 | GPIO6 | GPIO7);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
					GPIO0 | GPIO1 | GPIO7 | GPIO8);

	// Control register 1 for timer 2
	// Clock division ratio and center aligned mode selection  
	TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	// Auto reload register
	TIM2_ARR = 4096-1;

	/* Prescaler */
	TIM2_PSC = 0;

	// event generation register 0 update generation
	TIM2_EGR = TIM_EGR_UG;

	/* Output compare 3 mode and preload */
	// capture/compare mode register, output compare 1 mode, preload enable
	//TIM2_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	//TIM2_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
	TIM2_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	TIM2_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
	/* Polarity and state */
	TIM2_CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	/* ARR reload enable */
	TIM2_CR1 |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM2_CR1 |= TIM_CR1_CEN;


	TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM3_ARR = 4096-1;
	/* Prescaler */
	TIM3_PSC = 0;
	TIM3_EGR = TIM_EGR_UG;

	/* Output compare 3 mode and preload */
	//TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
	TIM3_CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;

	/* Polarity and state */
	//TIM3_CCER |= TIM_CCER_CC3E;

	/* ARR reload enable */
	TIM3_CR1 |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM3_CR1 |= TIM_CR1_CEN;

	TIM4_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM4_ARR = 4096-1;
	/* Prescaler */
	TIM4_PSC = 0;
	TIM4_EGR = TIM_EGR_UG;

	/* Output compare 3 mode and preload */
	TIM4_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;

	/* Polarity and state */
	TIM4_CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E;

	/* ARR reload enable */
	TIM4_CR1 |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM4_CR1 |= TIM_CR1_CEN;
}




void setup_usart(void){

	// Enable the USART1 interrupt.
	nvic_enable_irq(NVIC_USART3_IRQ);

	// GPIO port B.
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	// UART parameters.
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	// Enable USART1 Receive interrupt.
	usart_enable_rx_interrupt(USART3);
	usart_enable_tx_interrupt(USART3);

	// Enable the USART.
	usart_enable(USART3);
}

void usart3_isr(void){

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {


		/* Retrieve the data from the peripheral. */
		usart_rx_buffer[0] = usart_recv(USART3);
		usart_recieved = true;

		/* Enable transmit interrupt so it sends back the data. */
		
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		//usart_send(USART3, &usart_rx_buffer);
		gpio_toggle(GPIOA, led_2);
		/* Disable the TXE interrupt, it's no longer needed. */
		USART_CR1(USART3) &= ~USART_CR1_TXEIE;

	}

}


void dma1_channel1_isr(void) {

	DMA1_IFCR |= DMA_IFCR_CTCIF1;

	adc_start_conversion_regular(ADC1);
} 


void setup_LEDs( ){

	// For using PA13-15 and PB3.
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,0);

	// Output pushpulls.
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_2);

	// JTAG GPIOs remapped.
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_5);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_6);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_3);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_4);
}

/*
	
*/
inline void colorHexagon(int hue, int *R, int *G, int *B){

	int frac = hue >> 12;
	int ci = hue & 0xFFF;
	int cd = 4095 - ci;
	int cs = 4095;
	switch (frac) {
		case 0:	*R = cs;	*G = ci;	*B = 0; break;	//R1	G+	B0
		case 1:	*R = cd;	*G = cs;	*B = 0; break;	//R-	G1	B0
		case 2:	*R = 0;		*G = cs;	*B = ci; break;	//R0	G1	B+
		case 3:	*R = 0;		*G = cd;	*B = cs; break;	//R0	G-	B1
		case 4:	*R = ci;	*G = 0;		*B = cs; break;	//R+	G0	B1
		case 5:	*R = cs;	*G = 0;		*B = cd; break;	//R1	G0	B-
	}
}

void set_pwm(int R, int G, int B) {

	int cR = gamma_lut[ (R*RMAX) >> 12 ];
	int cG = gamma_lut[ (G*GMAX) >> 12 ];
	int cB = gamma_lut[ (B*BMAX) >> 12 ];

	TIM2_CCR2 = cR;
	TIM2_CCR3 = cG;
	TIM2_CCR4 = cB;

	TIM3_CCR1 = cR;
	TIM3_CCR2 = cG;
	TIM3_CCR3 = cB;

	TIM3_CCR4 = cR;
	TIM4_CCR2 = cG;
	TIM4_CCR3 = cB;
}


void USART_putn(uint32_t USARTx, volatile char *s, int size){
    
    if (!size) return;
    while(size--){
        usart_send_blocking(USARTx, *s);
        *s++;
    }
}
void USART_puts(uint32_t USARTx, volatile char *s){

    while(*s){
        usart_send_blocking(USARTx, *s);
        *s++;
    }
}

void setup_ADC( ){

	adc_off(ADC1);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO4 | GPIO5);

	adc_enable_scan_mode(ADC1);	
	adc_set_single_conversion_mode(ADC1);

	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	
	adc_power_on(ADC1);

	sleep_ms(2);

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

	uint8_t channel_array[3];
	channel_array[0] = ADC_CHANNEL0;
	channel_array[1] = ADC_CHANNEL4;
	channel_array[2] = ADC_CHANNEL5;

	adc_set_regular_sequence(ADC1, 3, channel_array);


	dma_channel_reset(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL1,(uint32_t)adc_samples);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, 3);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);

	dma_enable_channel(DMA1, DMA_CHANNEL1);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	adc_start_conversion_regular(ADC1);
	adc_enable_dma(ADC1);
}


int main( ){

	setup_clock( );

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(12000-1);  // 1 kHz
	systick_interrupt_enable( );
	systick_counter_enable( );

	setup_LEDs( );

	gpio_clear(GPIOA, led_2 | led_1);
	gpio_clear(GPIOA, led_3 | led_4 | led_5);
	gpio_clear(GPIOB, led_6);

	setup_timers( );
	setup_usart( );
	setup_ADC( );


	int R, G, B;
	int hue = 0;
	int counter = 0;

	int blinks = 0;
	volatile uint32_t adc_samples2[16];
	adc_samples2[0] = 1024;
	adc_samples2[1] = 2;
	adc_samples2[2] = 3;
	adc_samples2[3] = 4;
	while(1) {

		if (blinks == 500){
			blinks = 0;
			gpio_toggle(GPIOA, led_1);
			//gpio_toggle(GPIOA, led_2);
			//gpio_toggle(GPIOA, led_3);
			//gpio_toggle(GPIOA, led_4);
			//gpio_toggle(GPIOA, led_5);
			//gpio_toggle(GPIOB, led_6);
			/*
			USART_putn(USART3, &adc_samples2[0], 4);
			USART_putn(USART3, &adc_samples2[1], 4);
			USART_putn(USART3, &adc_samples2[2], 4);+
			*/
			/*
	        usart_send_blocking(USART3, 1);
	        usart_send_blocking(USART3, 2);
	        usart_send_blocking(USART3, 3);
	        usart_send_blocking(USART3, 4);
			*/

			USART_putn(USART3, &counter, 4);
			counter += 1;

		} else { blinks += 1; }


		//	usart_send(USART3, 63);
		//usart_rx_buffer[0] = usart_recv(USART3);

		if (usart_recieved == true){

			usart_recieved = false;
			gpio_toggle(GPIOA, led_2);
		}
		

		hue = (hue + 1) % (4096*6);		
		//colorHexagon(adc_samples[0], &R, &G, &B);
		colorHexagon(hue, &R, &G, &B);
		set_pwm(R, G, B);


			//adc_start_conversion_regular(ADC1);


		sleep_ms(1);

	}
	return 0;
}


