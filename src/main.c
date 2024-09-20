/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 24 2024
  * @brief   ECE 362 Lab 4 Student template
  ******************************************************************************
*/

/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "mann132";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

void nano_wait(int);
void autotest();

//=============================================================================
// Part 1: 7-segment display update with DMA
//=============================================================================

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);


//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER &= ~0x3FFFFF;
    GPIOB->MODER |= 0x155555;
    
    GPIOC->MODER &= ~0xFF00;
    GPIOC->MODER |= 0x5500;
    GPIOC->OTYPER &= ~0xF0;
    GPIOC->OTYPER |= 0xF0;

    GPIOC->MODER &= ~0xFF;
    GPIOC->PUPDR &= ~0xFF;
    GPIOC->PUPDR |= 0x55;
}

//============================================================================
// setup_dma() + enable_dma()
//============================================================================
void setup_dma(void) {
    //enable the RCC clock to the DMA controller
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    //turn off the enable bit for the channel
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; //A.5.1 in manual
    //set CMAR to the address of the msg array
    DMA1_Channel1->CMAR = (uint32_t)msg;
    //set CPAR to the address of the GPIOB_ODR register
    DMA1_Channel1->CPAR = (uint32_t)(&GPIOB->ODR);
    //set CNDTR to 8 (the amount of LEDs)
    DMA1_Channel1->CNDTR = 8;
    //set the direction for copying from memory to peripheral (1 = read from memory)
    DMA1_Channel1->CCR |= DMA_CCR_DIR;
    //set the MINC to incrememt the CMAR for every transfer
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    //set the memory datum size to 16-bit (01)
    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE_0;
    //set the peripheral datum size to 16-bit (01)
    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE_0;
    //set the channel for circular bit operation
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;
}   

void enable_dma(void) {
//enable the channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 83; //(sysclk 84MHZ / 84) timer clock of 1 MHz
    TIM15->ARR = 999; //1khz update frequency
    TIM15->DIER |= TIM_DIER_UDE;
    TIM15->DIER &= ~TIM_DIER_UIE;
    TIM15->CR1 |= TIM_CR1_CEN;
}

//=============================================================================
// Part 2: Debounced keypad scanning.
//=============================================================================

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
// Write the Timer 7 ISR here.  Be sure to give it the right name.
void TIM7_IRQHandler(void){
    TIM7->SR = ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 8399;
    TIM7->ARR = 999;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);

}

//=============================================================================
// Part 3: Analog-to-digital conversion for a volume level.
//=============================================================================
uint32_t volume = 2048;

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0xC; //PA1
    GPIOA->MODER |= 0xC;
    //enable the clock to the ADC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    //turn on the "high speed interval" 14 MHz clock (HSI14)
    RCC->CR2 |= RCC_CR2_HSI14ON;
    //wait for the 14 MHz clock to be ready
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    //enable the ADC by setting the ADEN bit in the CR register
    ADC1->CR |= ADC_CR_ADEN;
    //wait for the ADC to be ready
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    //select the corresponding channel for ADC_IN1 in the CHSLR
    while(1){
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1 << 1;//CHANNEL NUMBER
        //wait for the ADC to be ready
        while(!(ADC1->ISR & ADC_ISR_ADRDY));  
    }
}

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.

void TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    bcsum -= boxcar[bcn];
    bcsum+= boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if(bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE;
}

//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 4799;
    TIM2->ARR = 9999;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Part 4: Create an analog sine wave of a specified frequency
//===========================================================================
void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void) {
    //enable the clock to GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    //set the configuration for analog operation only for the pin associated w/ DAC_OUT1
    GPIOA->MODER |= (0x3 << (4 * 2));
    //enable the RCC clock for the DAC
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    //select a TIM6 (TRGO) trigger for the DAC with the (TSEL) field of the CR register
    DAC->CR |= DAC_CR_TSEL1_1;
    //enable the trigger for the DAC
    DAC->CR |= DAC_CR_TEN1;
    //enable the DAC
    DAC->CR |= DAC_CR_EN1;
}

//============================================================================
// Timer 6 ISR
//============================================================================
// Write the Timer 6 ISR here.  Be sure to give it the right name.

void TIM6_DAC_IRQHandler(void){
    TIM6->SR &= ~TIM_SR_UIF;
    offset0 += step0;
    offset1 += step1;
    if(offset0 >= (N >> 16)) {
        offset0 -= (N << 16);
    }
    if(offset1 >= (N << 16)){
        offset1 -+ (N << 16);
    }
    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16];
    samp = (samp * volume) >> 17;
    samp += 2048;
    DAC->DHR12R1 = samp;
}

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 0;
    TIM6->ARR = (48000000 / RATE) - 1;
    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= (0x1 << TIM_CR2_MMS_Pos);
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {
    internal_clock();
    // Initialize the display to something interesting to get started.
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    // Uncomment when you are ready to produce a confirmation code.
    // autotest();

    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();

    // Comment this for-loop before you demo part 1!
    // Uncomment this loop to see if "ECE 362" is displayed on LEDs.
    for (;;) {
        asm("wfi");
    }
    // End of for loop

    // Demonstrate part 1
// #define SCROLL_DISPLAY
#ifdef SCROLL_DISPLAY
    for(;;)
        for(int i=0; i<8; i++) {
            print(&"Hello...Hello..."[i]);
            nano_wait(250000000);
        }
#endif

    init_tim7();

    // Demonstrate part 2
// #define SHOW_KEY_EVENTS
#ifdef SHOW_KEY_EVENTS
    show_keys();
#endif

    setup_adc();
    init_tim2();

    // Demonstrate part 3
// #define SHOW_VOLTAGE
#ifdef SHOW_VOLTAGE
    for(;;) {
        printfloat(2.95 * volume / 4096);
    }
#endif

    init_wavetable();
    setup_dac();
    init_tim6();

// #define ONE_TONE
#ifdef ONE_TONE
    for(;;) {
        float f = getfloat();
        set_freq(0,f);
    }
#endif

    // demonstrate part 4
// #define MIX_TONES
#ifdef MIX_TONES
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
    }
#endif

    // Have fun.
    dialer();
}
