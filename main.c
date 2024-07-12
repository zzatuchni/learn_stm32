#include <inttypes.h>
#include <stdbool.h>

// Pin stuff

#define LED_PIN_BANK 'A'
#define LED_PIN_NUMBER 5

typedef struct {
    char bank;
    uint8_t number;
} STM32_Pin;

// GPIO stuff

#define GPIOA_START_ADDRESS 0x40020000
#define GPIO_BANK_BOUNDARY_SIZE 0x400

typedef struct {
    volatile uint32_t 
        MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH;
} GPIO_Bank;
#define GET_GPIO_BANK(bank) (GPIO_Bank *) ((GPIOA_START_ADDRESS) + (GPIO_BANK_BOUNDARY_SIZE) * ((bank) - 'A'))

typedef enum { 
    GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG 
} GPIO_Mode;

static inline void setGPIOMode(GPIO_Mode mode, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->MODER &= ~(3U << (n * 2)); //reset mode
    gpio->MODER |= ((mode & 3U) << (n * 2)); //set to new mode
}

static inline void writeToGPIO(bool value, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->BSRR = (1U << n) << (value ? 0 : 16);
}

// RCC stuff

#define RCC_START_ADDRESS 0x40023800

typedef struct {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
} RCC_Bank;

// Code

static inline void spin(uint32_t count) {
  while (count--) { asm("nop"); }
}

int main(void) {
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};

    RCC_Bank *rcc = (RCC_Bank *)RCC_START_ADDRESS;

    rcc->AHB1ENR |= (1UL << (led_pin.bank - 'A')); 

    setGPIOMode(GPIO_MODE_OUTPUT, led_pin);

    for (;;) {
        writeToGPIO(true, led_pin);
        spin(1638400);
        writeToGPIO(false, led_pin);
        spin(1638400);
    }
    
    return 0;
}


////////////////////////////////////////////////////////////
// Startup code - try to understand later
////////////////////////////////////////////////////////////

__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
    _estack, _reset};