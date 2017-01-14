#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "gamma.h"
#include <stdio.h>

/*#define RMAX 4096
#define GMAX 4096
#define BMAX 4096*/

#define RMAX 4096
#define GMAX 3500
#define BMAX 2500

#define led_1 GPIO13
#define led_2 GPIO14
#define led_3 GPIO15
#define led_4 GPIO3

#define adc_pot1 GPIO4
#define adc_pot2 GPIO5
#define adc_pot3 GPIO6

#define btn_1 GPIO11
#define btn_2 GPIO12

// WS2812
#define NUMBER_LEDS 16
#define BIT_0 10
#define BIT_1 29
#define FREQ 400000


const uint16_t gamma_lut[4096] =  {GAMMA_LUT};
volatile uint16_t adc_samples[16];
volatile int ms_time_delay;	//WTF!? Why do I have to use static!???
volatile uint8_t usart_rx_buffer[64];
volatile bool usart_recieved = false;
volatile uint32_t idle_state_LED_blinker_cnt = 0;

// A struct for keeping track of an average ADC value.
struct value_handler {
	uint32_t size;
	uint16_t *list;
	uint16_t index;
};

// A struct for saving data of LED strips.
struct led_strip{
	int H;
	int S;
	int L;
};

// For deciding which LEDs to light up, and to what strip
// the inputs are directed to.
enum states {
	IDLE,
	AUTO,
	STRIP_1,
	STRIP_2,
	STRIP_3
};

// States for a button, for dealing with debouncing.
enum button_states {
	NOT_PUSHED,
	BOUNCE_KEEP,
	PUSHED,
	SUSTAINED
};

// WS2812
struct color {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};
volatile uint8_t led_output_buf[NUMBER_LEDS*24+1];	//42 x 24
volatile struct color led_input_buf[NUMBER_LEDS];



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

	//rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_ADC1);

	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	
	rcc_periph_clock_enable(RCC_DMA1);
}

void setup_timers( ){

	void setup_timer1( ){

		// Clock division ratio and center aligned mode selection.
		TIM1_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;

		// Auto reload register.
		TIM1_ARR = 4096-1;

		// Prescaler.
		TIM1_PSC = 0;

		// Event generation register 0 update generation.
		TIM1_EGR = TIM_EGR_UG;

		TIM1_BDTR |= TIM_BDTR_MOE;

		// Output compare 3 mode and preload.
		// Capture/compare mode register, output compare 1 mode, 
		// preload enable.
		TIM1_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
		TIM1_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
		TIM1_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
		TIM1_CCMR2 |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
		
		// Polarity and state.
		TIM1_CCER |= TIM_CCER_CC1E;

		// ARR reload enable.
		TIM1_CR1 |= TIM_CR1_ARPE;

		// Counter enable.
		TIM1_CR1 |= TIM_CR1_CEN;
	}

	void setup_timer2( ){

		// Clock division ratio and center aligned mode selection.
		TIM2_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;

		// Auto reload register.
		TIM2_ARR = 4096-1;

		// Prescaler.
		TIM2_PSC = 0;

		// event generation register 0 update generation.
		TIM2_EGR = TIM_EGR_UG;

		// Output compare 3 mode and preload.
		// Capture/compare mode register, output compare 1 mode, 
		// preload enable.
		TIM2_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
		TIM2_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;

		// Polarity and state.
		TIM2_CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		// ARR reload enable.
		TIM2_CR1 |= TIM_CR1_ARPE;

		// Counter enable.
		TIM2_CR1 |= TIM_CR1_CEN;
	}

	void setup_timer3( ){

		// Clock division ratio and center aligned mode selection.
		TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;

		// Period
		TIM3_ARR = 4096-1;

		// Prescaler
		TIM3_PSC = 0;
		TIM3_EGR = TIM_EGR_UG;

		// Output compare 3 mode and preload.
		TIM3_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
		TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;

		// Polarity and state.
		TIM3_CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		// ARR reload enable.
		TIM3_CR1 |= TIM_CR1_ARPE;

		// Counter enable.
		TIM3_CR1 |= TIM_CR1_CEN;		
	}

	void setup_timer4( ){

		// Clock division ratio and center aligned mode selection.
		TIM4_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;

		// Period.
		TIM4_ARR = 4096-1;

		// Prescaler.
		TIM4_PSC = 0;
		TIM4_EGR = TIM_EGR_UG;

		// Output compare 3 mode and preload.
		TIM4_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
		TIM4_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
		
		// Polarity and state.
		TIM4_CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		// ARR reload enable.
		TIM4_CR1 |= TIM_CR1_ARPE;
		
		// Counter enable.
		TIM4_CR1 |= TIM_CR1_CEN;	
	}

	/*
		tim1_ch1 A8
		tim3_ch4 B1
		tim3_ch3 B0

		tim3_ch2 A7
		tim2_ch2 A1
		tim4_ch4 B9

		tim4_ch3 B8
		tim4_ch2 B7
		tim4_ch1 B6
	*/

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
					GPIO1 | GPIO7 | GPIO8);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
					GPIO0 | GPIO1 | GPIO6 | GPIO7 | GPIO8 | GPIO9);

	setup_timer1( );
	setup_timer2( );
	setup_timer3( );
	setup_timer4( );
}


void setup_timer_for_ws2812( ){

	//LED data out
	gpio_set(GPIOB, GPIO0);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);


	TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM3_ARR = (24000000 / FREQ)-1;
	/* Prescaler */
	TIM3_PSC = 0;
	TIM3_EGR = TIM_EGR_UG;

	/* Output compare 3 mode and preload */
	TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM2 | TIM_CCMR2_OC3PE;

	/* Polarity and state */
	TIM3_CCER |= TIM_CCER_CC3E;

	/* ARR reload enable */
	TIM3_CR1 |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM3_CR1 |= TIM_CR1_CEN;


	TIM3_CCR3 = 0;
}

void set_pwm_strip_1(int R, int G, int B) {

	int cR = gamma_lut[ (R*RMAX) >> 12 ];
	int cG = gamma_lut[ (G*GMAX) >> 12 ];
	int cB = gamma_lut[ (B*BMAX) >> 12 ];

	
	TIM1_CCR1 = cR;
	TIM3_CCR4 = cG;
	TIM3_CCR3 = cB;
}

void set_pwm_strip_2(int R, int G, int B) {

	int cR = gamma_lut[ (R*RMAX) >> 12 ];
	int cG = gamma_lut[ (G*GMAX) >> 12 ];
	int cB = gamma_lut[ (B*BMAX) >> 12 ];

	TIM3_CCR2 = cR;
	TIM2_CCR2 = cG;
	TIM4_CCR4 = cB;
}

void set_pwm_strip_3(int R, int G, int B) {

	int cR = gamma_lut[ (R*RMAX) >> 12 ];
	int cG = gamma_lut[ (G*GMAX) >> 12 ];
	int cB = gamma_lut[ (B*BMAX) >> 12 ];

	TIM4_CCR3 = cR;
	TIM4_CCR2 = cG;
	TIM4_CCR1 = cB;
}

void setup_usart_1( ) {

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);


	usart_set_baudrate(USART1, 921600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);
}

void setup_usart_2(void){

	// Enable the USART1 interrupt.
	nvic_enable_irq(NVIC_USART2_IRQ);

	// GPIO port A.
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	// UART parameters.
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// Enable USART1 Receive interrupt.
	usart_enable_rx_interrupt(USART2);
	usart_enable_tx_interrupt(USART2);

	// Enable the USART.
	usart_enable(USART2);
}

void usart2_isr(void){

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {


		/* Retrieve the data from the peripheral. */
		usart_rx_buffer[0] = usart_recv(USART2);
		usart_recieved = true;

		/* Enable transmit interrupt so it sends back the data. */
		
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		//usart_send(USART2, &usart_rx_buffer);
		/* Disable the TXE interrupt, it's no longer needed. */
		USART_CR1(USART2) &= ~USART_CR1_TXEIE;

	}

}

void setup_LEDs( ){

	// For using PA13-15 and PB3.
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,0);

	// JTAG GPIOs remapped.
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_3);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, led_4);

	gpio_clear(GPIOA, led_1);
	gpio_clear(GPIOA, led_2);
	gpio_clear(GPIOA, led_3);
	gpio_clear(GPIOB, led_4);
}

void setup_buttons( ){

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
					GPIO_CNF_INPUT_FLOAT, GPIO11);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
					GPIO_CNF_INPUT_FLOAT, GPIO12);
}

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

inline void colorHSL(int hue, int sat, int light,int *R, int *G, int *B) {

	int tR,tG,tB;
	int frac = hue >> 12;

	// Chroma.
	int C = ((4095-abs((light<<1)-4095))*sat)>>12;
	int X= (C*(4095-abs((hue % 8192) - 4095)))>>12;

	// Hue.
	switch (frac) {
		case 0:	tR = C;	tG = X;	tB = 0; break;	//R1	G+	B0
		case 1:	tR = X;	tG = C;	tB = 0; break;	//R-	G1	B0
		case 2:	tR = 0;	tG = C;	tB = X; break;	//R0	G1	B+
		case 3:	tR = 0;	tG = X;	tB = C; break;	//R0	G-	B1
		case 4:	tR = X;	tG = 0;	tB = C; break;	//R+	G0	B1
		case 5:	tR = C;	tG = 0;	tB = X; break;	//R1	G0	B-
	}

	// Lightness.	
	int m = light - (C>>1);
	tR+=m; tG+=m; tB+=m;
	*R = tR; *G = tG; *B = tB;
}

inline void colorHCY(int hue, int chroma, int luma,int *R, int *G, int *B) {

	int tR,tG,tB;
	int frac = hue >> 12;

	// Chroma.
	int C = chroma;
	int X= (C*(4095-abs((hue % 8192) - 4095)))>>12;

	// Hue.
	switch (frac) {
		case 0:	tR = C;	tG = X;	tB = 0; break;	//R1	G+	B0
		case 1:	tR = X;	tG = C;	tB = 0; break;	//R-	G1	B0
		case 2:	tR = 0;	tG = C;	tB = X; break;	//R0	G1	B+
		case 3:	tR = 0;	tG = X;	tB = C; break;	//R0	G-	B1
		case 4:	tR = X;	tG = 0;	tB = C; break;	//R+	G0	B1
		case 5:	tR = C;	tG = 0;	tB = X; break;	//R1	G0	B-
	}

	// Luma.
	int m = luma - ((tR*1229 + tG*2417 + tB*451) >> 12);
	tR+=m; tG+=m; tB+=m;

	#define Crowbar(x, min, max)	if (x>max) x=max; if (x<min) x=min

	Crowbar(tR, 0, 4095);
	Crowbar(tG, 0, 4095);
	Crowbar(tB, 0, 4095);
	
	*R = tR; *G = tG; *B = tB;
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
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4 | GPIO5 | GPIO6);

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
	channel_array[0] = ADC_CHANNEL4;
	channel_array[1] = ADC_CHANNEL5;
	channel_array[2] = ADC_CHANNEL6;

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

void dma1_channel1_isr(void) {

	DMA1_IFCR |= DMA_IFCR_CTCIF1;
	adc_start_conversion_regular(ADC1);
} 

uint16_t average_of_value_handler(struct value_handler *vh){

	uint16_t aver = 0;
	for(uint32_t i = 0; i < vh->size; i++){
		aver += vh->list[i];
	}
	aver = aver / vh->size;
	return aver;
}

void add_to_value_handler(struct value_handler *vh, uint16_t value){

	vh->list[vh->index] = value;
	vh->index += 1;
	if (vh->index == vh->size){
		vh->index = 0;
	}
}

void intro_blink( ){

	uint32_t counter = 0;
	uint32_t led_state = 1;

	while(led_state < 11) {

		if (counter == 500){

			counter = 0;

			if (led_state == 1){
				led_state = 2;
				gpio_set(GPIOA,led_1);
				gpio_clear(GPIOA,led_2);
				gpio_clear(GPIOA,led_3);
				gpio_clear(GPIOB,led_4);
			} else {
				if (led_state == 2){
					led_state = 3;
					gpio_clear(GPIOA,led_1);
					gpio_set(GPIOA,led_2);
					gpio_clear(GPIOA,led_3);
					gpio_clear(GPIOB,led_4);

				} else {
					if (led_state == 3){
						led_state = 4;
						gpio_clear(GPIOA,led_1);
						gpio_clear(GPIOA,led_2);
						gpio_set(GPIOA,led_3);
						gpio_clear(GPIOB,led_4);
					} else {							
						if (led_state == 4){
							led_state = 5;
							gpio_clear(GPIOA,led_1);
							gpio_clear(GPIOA,led_2);
							gpio_clear(GPIOA,led_3);
							gpio_set(GPIOB,led_4);
						} else {
							if (led_state == 5){
								led_state = 6;
								gpio_set(GPIOA,led_1);
								gpio_set(GPIOA,led_2);
								gpio_set(GPIOA,led_3);
								gpio_set(GPIOB,led_4);
							} else {
								if (led_state >= 5){
									led_state += 1;
									gpio_toggle(GPIOA,led_1);
									gpio_toggle(GPIOA,led_2);
									gpio_toggle(GPIOA,led_3);
									gpio_toggle(GPIOB,led_4);
								}	
							}
						}
					}
				}
			}
		} else { 
			counter += 1;
		}
		sleep_ms(1);
	}

	gpio_clear(GPIOA,led_1);
	gpio_clear(GPIOA,led_2);
	gpio_clear(GPIOA,led_3);
	gpio_clear(GPIOB,led_4);
}

void idle_state_LED_blinker( ){

	idle_state_LED_blinker_cnt++;

	if (idle_state_LED_blinker_cnt == 4){
		idle_state_LED_blinker_cnt = 0;
	}

	switch (idle_state_LED_blinker_cnt){

		case 0:
			gpio_set(GPIOA,led_1);
			gpio_clear(GPIOA,led_2);
			gpio_clear(GPIOA,led_3);
			gpio_clear(GPIOB,led_4);

			break;
			
		case 1:
			gpio_clear(GPIOA,led_1);
			gpio_set(GPIOA,led_2);
			gpio_clear(GPIOA,led_3);
			gpio_clear(GPIOB,led_4);

			break;
			
		case 2:
			gpio_clear(GPIOA,led_1);
			gpio_clear(GPIOA,led_2);
			gpio_set(GPIOA,led_3);
			gpio_clear(GPIOB,led_4);

			break;
			
		case 3:
			gpio_clear(GPIOA,led_1);
			gpio_clear(GPIOA,led_2);
			gpio_clear(GPIOA,led_3);
			gpio_set(GPIOB,led_4);

			break;
	}
}

// WS2812
void dma1_transmit_8_32(uint32_t src, uint32_t dst, uint32_t length, uint32_t channel) {

	dma_disable_channel(DMA1, channel);

	dma_channel_reset(DMA1, channel);

	dma_set_peripheral_address(DMA1, channel, dst);
	dma_set_memory_address(DMA1, channel, src);
	dma_set_number_of_data(DMA1, channel, length);
	dma_set_read_from_memory(DMA1, channel);
	dma_enable_memory_increment_mode(DMA1, channel);
	dma_set_peripheral_size(DMA1, channel, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(DMA1, channel, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, channel, DMA_CCR_PL_VERY_HIGH);

	dma_enable_channel(DMA1, channel);

}

void dma1_recieve(uint32_t src, uint32_t dst, uint32_t length, uint32_t channel) {
	
	dma_disable_channel(DMA1, channel);
	dma_channel_reset(DMA1, channel);

	dma_set_peripheral_address(DMA1, channel, src);
	dma_set_memory_address(DMA1, channel, dst);
	dma_set_number_of_data(DMA1, channel, length);
	dma_set_read_from_peripheral(DMA1, channel);
	dma_enable_memory_increment_mode(DMA1, channel);
	dma_set_peripheral_size(DMA1, channel, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, channel, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, channel, DMA_CCR_PL_VERY_HIGH);

	dma_enable_channel(DMA1, channel);

}

void populate_test_buffer() {
	// start populate buffer
	for (int led=0;led<NUMBER_LEDS;led++) {
		led_input_buf[led].r = 192;
		led_input_buf[led].g = 0;
		led_input_buf[led].b = 0;
	}
}

void process_buffer() {
	volatile uint8_t* ptr=led_output_buf;
	for (int led=0;led<NUMBER_LEDS;led++) {

		for (int bit=7;bit>=0;bit--) { *ptr++ = led_input_buf[led].g & (1 << bit) ? BIT_1 : BIT_0; }
		for (int bit=7;bit>=0;bit--) { *ptr++ = led_input_buf[led].r & (1 << bit) ? BIT_1 : BIT_0; }
		for (int bit=7;bit>=0;bit--) { *ptr++ = led_input_buf[led].b & (1 << bit) ? BIT_1 : BIT_0; }
	}
}

void send_led_buffer( ) {
	// Send led data
	// dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);

	dma1_transmit_8_32((uint32_t) led_output_buf, (uint32_t) &TIM3_CCR3, 24*NUMBER_LEDS+1, DMA_CHANNEL2 );
	TIM3_DIER |= TIM_DIER_CC3DE;	//Enable DMA transfer for CC1
}

void send_led_buffer_blocking( ) {
	//dma_clear_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_TCIF);
	send_led_buffer();
	while (!dma_get_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_TCIF));

	TIM3_DIER &= ~TIM_DIER_CC3DE;	//Disable DMA transfer for CC1
}

void recieve_led_data_blocking( ) {


	//dma_clear_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_TCIF);
	dma1_recieve((uint32_t)&USART1_DR, (uint32_t)led_input_buf, NUMBER_LEDS * sizeof(struct color), DMA_CHANNEL5);

	usart_enable_rx_dma(USART1);


	while (!dma_get_interrupt_flag(DMA1, DMA_CHANNEL5, DMA_TCIF));

	usart_disable_rx_dma(USART1);


}
int main( ){

	setup_clock( );

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(12000-1);  // 1 kHz
	systick_interrupt_enable( );
	systick_counter_enable( );


	setup_LEDs( );
	setup_buttons( );

	setup_timers( );
	//setup_usart_2( );
	setup_ADC( );
	setup_usart_1( );


/*
//WS2812

	setup_LEDs( );
	setup_timer_for_ws2812( );

	
	populate_test_buffer();
	process_buffer();
	send_led_buffer_blocking( );

	while(1) {
		//recieve_led_data_blocking();
		sleep_ms(10);
		program_counter++;
		if(program_counter % 40 == 0){
			idle_state_LED_blinker( );
		}
		process_buffer( );
		send_led_buffer_blocking( );
	}
*/
uint32_t program_counter = 0;

	// ADC average.
	struct value_handler adc_values1;
	adc_values1.size = 16;
	uint16_t adc1_list[adc_values1.size];
	adc_values1.list = &adc1_list;
	adc_values1.index = 0;
	uint16_t average = 0;

	// Declaring variables for the auto roll mode.
	//uint32_t program_counter = 0;
	uint32_t auto_roll_speed = 40;
	uint32_t auto_roll_is_paused = 0;
	uint32_t auto_roll_hue_strip_1 = 1333;
	uint32_t auto_roll_hue_strip_2 = 2666;
	uint32_t auto_roll_hue_strip_3 = 3999;

	// Declaring the state variables.
	enum states current_state = AUTO;
	enum button_states button_1_state = NOT_PUSHED;
	enum button_states button_2_state = NOT_PUSHED;


	// Declaring the LED strip structs.
	struct led_strip led_strip_1 = {1333,4095,2000};
	struct led_strip led_strip_2 = {2666,4095,2000};
	struct led_strip led_strip_3 = {3999,4095,2000};

	// Variables for PWM data, derived from hue variable.
	int R, G, B = 0;



	// Startup blinking function, for added coolness.
	//intro_blink( );

	while(1)
	{
		sleep_ms(1);
		program_counter++;

		// UART data.
		if (usart_recieved == true){
			usart_recieved = false;
		}

/*			add_to_value_handler(&adc_values1, adc_samples[2]);
			average = average_of_value_handler(&adc_values1);
*/

		// Checking the buttons and changing state.
		if(program_counter % 40 == 0)
		{
			// Checking button 1.
			if (gpio_get(GPIOA, btn_1) != 0) {
				if (button_1_state == NOT_PUSHED){
					button_1_state = PUSHED;
				} else {
					if (button_1_state == PUSHED){
						button_1_state = SUSTAINED;
					}
				}
			} else {
				button_1_state = NOT_PUSHED;
			}
	
			// Checking button 2.
			if (gpio_get(GPIOA, btn_2) != 0) {
				if (button_2_state == NOT_PUSHED){
					button_2_state = PUSHED;
				} else {
					if (button_2_state == PUSHED){
						button_2_state = SUSTAINED;
					}
				}
			} else {
				button_2_state = NOT_PUSHED;
			}

			// Changing state.
			if (button_1_state == PUSHED){
				if (current_state == STRIP_3){
					current_state = IDLE;
				} else {
					current_state++;
				}
			}

			// Pausing autoroll.
			if (button_2_state == PUSHED){
				if (current_state == AUTO){
					if (auto_roll_is_paused == 0){
						auto_roll_is_paused = 1;
					} else {
						auto_roll_is_paused = 0;
					}
				}
			}
		}

		// State depending actions, i.e., setting the PWMs, and saving the
		// inputs. If in a state of any strip, it will display the current
		// inputs of the ADC, otherwise it will use its saved data.
		// The four LEDs will indicate which state is the current.
		switch (current_state){
			case IDLE:	

				if (program_counter % 500 == 0){
					idle_state_LED_blinker( );
				}

				colorHSL(led_strip_1.H*6, led_strip_1.S, led_strip_1.L, &R, &G, &B);
				set_pwm_strip_1(R, G, B);
				colorHSL(led_strip_2.H*6, led_strip_2.S, led_strip_2.L, &R, &G, &B);
				set_pwm_strip_2(R, G, B);
				colorHSL(led_strip_3.H*6, led_strip_3.S, led_strip_3.L, &R, &G, &B);
				set_pwm_strip_3(R, G, B);

				break;

			case AUTO:

				gpio_set(GPIOA,led_1);
				gpio_clear(GPIOA,led_2);
				gpio_clear(GPIOA,led_3);
				gpio_clear(GPIOB,led_4);

				if (auto_roll_is_paused == 0){
					auto_roll_speed = adc_samples[2]/400+1;
					if(program_counter % auto_roll_speed == 0){
						
						auto_roll_hue_strip_1++;
						auto_roll_hue_strip_2++;
						auto_roll_hue_strip_3++;
					}
				}
				colorHSL((auto_roll_hue_strip_1%4095)*6, 4095, adc_samples[0], &R, &G, &B);
				set_pwm_strip_1(R, G, B);
				colorHSL((auto_roll_hue_strip_2%4095)*6, 4095, adc_samples[0], &R, &G, &B);
				set_pwm_strip_2(R, G, B);
				colorHSL((auto_roll_hue_strip_3%4095)*6, 4095, adc_samples[0], &R, &G, &B);
				set_pwm_strip_3(R, G, B);

				break;

			case STRIP_1:

				gpio_clear(GPIOA,led_1);
				gpio_set(GPIOA,led_2);

				if (button_2_state == PUSHED){
					auto_roll_hue_strip_1 = adc_samples[2];
					led_strip_1.H = adc_samples[2];
					led_strip_1.S = adc_samples[1];
					led_strip_1.L = adc_samples[0];
				}
				colorHSL(adc_samples[2]*6, adc_samples[1], adc_samples[0], &R, &G, &B);
				set_pwm_strip_1(R, G, B);
				colorHSL(led_strip_2.H*6, led_strip_2.S, led_strip_2.L, &R, &G, &B);
				set_pwm_strip_2(R, G, B);
				colorHSL(led_strip_3.H*6, led_strip_3.S, led_strip_3.L, &R, &G, &B);
				set_pwm_strip_3(R, G, B);

				break;

			case STRIP_2:

				gpio_clear(GPIOA,led_2);
				gpio_set(GPIOA,led_3);

				if (button_2_state == PUSHED){
					auto_roll_hue_strip_2 = adc_samples[2];
					led_strip_2.H = adc_samples[2];
					led_strip_2.S = adc_samples[1];
					led_strip_2.L = adc_samples[0];
				}

				colorHSL(led_strip_1.H*6, led_strip_1.S, led_strip_1.L, &R, &G, &B);
				set_pwm_strip_1(R, G, B);
				colorHSL(adc_samples[2]*6, adc_samples[1], adc_samples[0], &R, &G, &B);
				set_pwm_strip_2(R, G, B);
				colorHSL(led_strip_3.H*6, led_strip_3.S, led_strip_3.L, &R, &G, &B);
				set_pwm_strip_3(R, G, B);

				break;

			case STRIP_3:

				gpio_clear(GPIOA,led_3);
				gpio_set(GPIOB,led_4);

				if (button_2_state == PUSHED){
					auto_roll_hue_strip_3 = adc_samples[2];
					led_strip_3.H = adc_samples[2];
					led_strip_3.S = adc_samples[1];
					led_strip_3.L = adc_samples[0];
				}

				colorHSL(led_strip_1.H*6, led_strip_1.S, led_strip_1.L, &R, &G, &B);
				set_pwm_strip_1(R, G, B);
				colorHSL(led_strip_2.H*6, led_strip_2.S, led_strip_2.L, &R, &G, &B);
				set_pwm_strip_2(R, G, B);
				colorHSL(adc_samples[2]*6, adc_samples[1], adc_samples[0], &R, &G, &B);
				set_pwm_strip_3(R, G, B);

				break;
		}
	}
	return 0;
}