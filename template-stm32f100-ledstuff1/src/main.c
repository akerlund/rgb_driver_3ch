#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "gamma.h"

const uint16_t gamma_lut[4096] =  {GAMMA_LUT};

/*
	TIM2_CCR2 = B;
	TIM2_CCR3 = G;
	TIM3_CCR3 = R;	
*/

#define RMAX 4096
#define GMAX 3500
#define BMAX 2500

#define led_set1 GPIO12	//
#define led_set2 GPIO14
#define led_set3 GPIO3

#define led_seg1 GPIO11
#define led_seg2 GPIO13
#define led_seg3 GPIO15

#define adc_pot1 GPIO0
#define adc_pot2 GPIO4
#define adc_pot3 GPIO5

#define btn_1 GPIO1
#define btn_2 GPIO1

void SetPWM(int R, int G, int B) {

	int cR = gamma_lut[ (R*RMAX) >> 12 ];
	int cG = gamma_lut[ (G*GMAX) >> 12 ];
	int cB = gamma_lut[ (B*BMAX) >> 12 ];
	/*
	TIM3_CCR3 = cR;	
	TIM2_CCR3 = cG;
	TIM2_CCR2 = cB;
	*/
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

void SetupPWM() {

	// tim2 ch2
	// tim2 ch3
	// tim2 ch4

	// tim3 ch1
	// tim3 ch2
	// tim3 ch3
	// tim3 ch4

	// tim4 ch2
	// tim4 ch3
	SetPWM(0,0,0);

	// control register 1 for timer 2
	// clock division ratio and center aligned mode selection  
	TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	// auto reload register
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


static volatile int ms_time_delay;	//WTF!? Why do I have to use static!???

void sys_tick_handler(void) {
	if (ms_time_delay) {
		ms_time_delay--;
	}
}

void sleep_ms(int t) {
	ms_time_delay = t;	while (ms_time_delay);
}

inline void colorHexagon(int hue, int *R, int *G, int *B) {
	int frac = hue >> 12;
	int ci = hue & 0xFFF;
	int cd = 4095 - ci;
	int cs = 4095;
	switch (frac) {
		case 0:	*R = cs;	*G = ci;	*B = 0; break;		//R1	G+	B0
		case 1:	*R = cd;	*G = cs;	*B = 0; break;		//R-	G1	B0
		case 2:	*R = 0;	*G = cs;	*B = ci; break;	//R0	G1	B+
		case 3:	*R = 0;	*G = cd;	*B = cs; break;	//R0	G-	B1
		case 4:	*R = ci;	*G = 0;	*B = cs; break;	//R+	G0	B1
		case 5:	*R = cs;	*G = 0;	*B = cd; break;	//R1	G0	B-
	}
}
/*
static unsigned long xorshf96(void) {    // A George Marsaglia generator, period 2^96-1
	static unsigned long x=123456789, y=362436069, z=521288629;
	unsigned long t;

	x ^= x << 16;
	x ^= x >> 5;
	x ^= x << 1;

	t = x;
	x = y;
	y = z;

	z = t ^ x ^ y;
	return z;
}
*/
static volatile done=0;


void SetupADC() {

	adc_off(ADC1);

	adc_enable_scan_mode(ADC1);	
	adc_set_single_conversion_mode(ADC1);

	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	
	adc_power_on(ADC1);


	sleep_ms(2);	//Sleeping 1 ms may be less than one ms if systick is about to happen, 
	// we actually just need to wait 3µS according to http://libopencm3.github.io/docs/latest/stm32f1/html/group__adc__file.html#ga51f01f6dedbcfc4231e0fc1d8943d956

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

	uint8_t channel_array[3];
	channel_array[0] = ADC_CHANNEL0;
	channel_array[1] = ADC_CHANNEL4;
	channel_array[2] = ADC_CHANNEL5;
	adc_set_regular_sequence(ADC1, 1, channel_array);

}

volatile uint16_t adc_samples[16];

int main() {


	rcc_clock_setup_in_hsi_out_24mhz( );
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	// Leds for status
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_set1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_set2);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_set3);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_seg1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_seg2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_seg3);

	//gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, maps);

	// ledstrips
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO1 | GPIO2 | GPIO3 | GPIO6 | GPIO7);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1 | GPIO7 | GPIO8);

	
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_ADC1);
	


	//PA11, PA4
	//gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);


	SetupPWM();
	ms_time_delay=0;
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(12000-1);  // 1 kHz
	systick_interrupt_enable();
	systick_counter_enable();


	// ADC
	
	SetupADC();

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO4 | GPIO5);

	//Start sampling
	dma_channel_reset(DMA1, DMA_CHANNEL1);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL1,(uint32_t)adc_samples);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);
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




	//int q=0;


	 int R, G, B;
	 int hue=0;

	int blinks = 0;
	while(1) {

		if (blinks == 50){
			blinks = 0;
			gpio_toggle(GPIOA, led_set1 | led_seg1);
		}else{
			blinks += 1;
		}

		/*
		1.7	34 seg2 A13 jtms-swdio
		0.0	37 set2 A14 JTCK-SWCLK
		1.7	38 seg3 A15 JTDI
		0.0	39 set3 B3  JTDO
		*/
		
		hue = (hue + 1) % (4096*6);
		
		//colorHexagon(adc_samples[0], &R, &G, &B);
		colorHexagon(hue, &R, &G, &B);
		SetPWM(R, G, B);


		sleep_ms(1);
		//adc_start_conversion_regular(ADC1);
	}
	return 0;
}



void dma1_channel1_isr(void) {
	DMA1_IFCR |= DMA_IFCR_CTCIF1;

	int R, G, B;
	//colorHexagon(adc_samples[0]*6, &R, &G, &B);

	//SetPWM(R, G, B);

	// int z;
	// if (adc_samples[1] & 2048) {
	// 	z = adc_samples[1] - 2048;
	// 	int zi = 2047 - z;
	// 	SetPWM(((R * zi)>>11)+z, ((G * zi)>>11)+z, ((B * zi)>>11)+z);
	// } else {
	// 	z = adc_samples[1];
	// 	SetPWM((R * z)>>11, (G * z)>>11, (B * z)>>11);
	// }



	adc_start_conversion_regular(ADC1);

}
