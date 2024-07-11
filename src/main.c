#include "stm32f0xx.h"

#define MAIN_APP_START_ADDRESS 0x08004000U

#define RAM_START_ADDRESS 0x20000000U
#define ISR_VECTOR_TABLE_SIZE 192 // bytes


typedef void (*FuncPtr_TypeDef)(void);

int main(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODER8_0; // pin 8 mode out
    for (uint8_t blinks=0; blinks<6; blinks++) {
        if ( blinks%2 ) {
            GPIOC->BSRR = GPIO_BSRR_BS_8; // orange led on
        } else {
            GPIOC->BRR = GPIO_BRR_BR_8; // orange led off
        }
        for (uint32_t nops=0; nops<500000; nops++) {} // sleep
    }


    //SysTick DeInit
    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    SysTick->LOAD = 0;

    __disable_irq();

    NVIC->ICER[0U] = 0xFFFFFFFF; // Interrupt Clear Enable Register
    NVIC->ICPR[0U] = 0xFFFFFFFF; //Interrupt Clear Pending Register

    // Disabling perepherials
    RCC->AHBENR &= ~(RCC_AHBENR_DMA1EN | RCC_AHBENR_CRCEN| RCC_AHBENR_GPIOAEN| RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN| RCC_AHBENR_TSCEN);
    RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN 
                        | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN | RCC_APB2ENR_SYSCFGCOMPEN);
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM14EN | RCC_APB1ENR_WWDGEN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_USART2EN
                        | RCC_APB1ENR_USART3EN | RCC_APB1ENR_USART4EN | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN | RCC_APB1ENR_USBEN | RCC_APB1ENR_CANEN | RCC_APB1ENR_DACEN | RCC_APB1ENR_CECEN );


    // Copy '.isr_vector' table (size 'ISR_VECTOR_TABLE_SIZE' bytes) section from main application located on FLASH
    // to section in RAM '.isr_vector_ram' (48 words / 192 bytes) 
    // See the RAM offset in the linker script file (Like: "RAM_IVT (xrw) : ORIGIN = 0x20000000, LENGTH = 192", "RAM (xrw) : ORIGIN = 0x200000C0, LENGTH = 16192") of main application
    for (uint32_t addr_shift=0; addr_shift<ISR_VECTOR_TABLE_SIZE; addr_shift++) {
        *(volatile uint8_t*)(RAM_START_ADDRESS+addr_shift) = *(volatile uint8_t*)(MAIN_APP_START_ADDRESS+addr_shift);
    }

    __enable_irq();
    
  
    // MEM_MODE[1:0]: Memory mapping selection bits. RM0091 page 169
    //SYSCFG->CFGR1 |= (0b11UL); // Set loading the top of stack and IRQ vectors from RAM after reset (instead of FLASH by default)
    SYSCFG->CFGR1 |= (SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0); // Set loading the top of stack and IRQ vectors from RAM after reset (instead of FLASH by default) SYSCFG->CFGR1[1:0] = 0b11

    FuncPtr_TypeDef jump_to_app; // Create a call of Reset_Handler() from RAM
    jump_to_app = (FuncPtr_TypeDef)( *(volatile uint32_t*)(MAIN_APP_START_ADDRESS+4U) );

    // Change the main and local stack pointer.
    __set_MSP( *(volatile uint32_t*)MAIN_APP_START_ADDRESS );
    jump_to_app();

    return 1;
}

