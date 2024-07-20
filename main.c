#include <inttypes.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////
//  Pin
////////////////////////////////////////////////////////////

#define LED_PIN_BANK 'A'
#define LED_PIN_NUMBER 5

typedef struct {
    char bank;
    uint8_t number;
} STM32_Pin;

////////////////////////////////////////////////////////////
//  GPIO
////////////////////////////////////////////////////////////

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

typedef uint8_t GPIO_AF;

static inline void gpio_set_mode(GPIO_Mode mode, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->MODER &= ~(3U << (n * 2)); //reset mode
    gpio->MODER |= ((mode & 3U) << (n * 2)); //set to new mode
}


//static inline void gpio_set_af(GPIO_AF af, STM32_Pin pin) {
//
//}

static inline void gpio_write(bool value, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->BSRR = (1U << n) << (value ? 0 : 16);
}

////////////////////////////////////////////////////////////
//  RCC
////////////////////////////////////////////////////////////

#define RCC_START_ADDRESS 0x40023800

typedef struct {
    volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
        RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
        RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
        AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
        RESERVED6[2], SSCGR, PLLI2SCFGR;
} RCC_Regs;

////////////////////////////////////////////////////////////
//  Systick
////////////////////////////////////////////////////////////

#define SYSTICK_START_ADDRESS 0xe000e010

typedef struct {
    volatile uint32_t CSR, RVR, CVR, CALIB;
} Systick_Regs;

static inline void systick_init(uint32_t ticks) {
    Systick_Regs *systick = (Systick_Regs *)SYSTICK_START_ADDRESS;
    if ((ticks - 1) > 0xffffff) return;
    systick->RVR = ticks - 1;
    systick->CVR = 0;
    systick->CSR |= 7UL;
}

__attribute__((used)) static volatile uint32_t num_ticks;
void _on_systick_interrupt(void) {
    num_ticks++;
}

////////////////////////////////////////////////////////////
//  UART
////////////////////////////////////////////////////////////

/*
#define UART1_START_ADDRESS 0x40011000
#define UART2_START_ADDRESS 0x40004400
#define UART3_START_ADDRESS 0x40004800

typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} UART_REGS;

static inline void uart_init(UART_REGS *uart, uint8_t af_num, uint8_t mode) {

}

static inline void uart_read_byte() {

}

static inline void uart_read_buf() {
    
}

static inline void uart_write_byte() {

}

static inline void uart_write_buf() {
    
}
*/

static inline void spin(uint32_t count) {
    while (count--) asm("nop");
}

////////////////////////////////////////////////////////////
//  Main code
////////////////////////////////////////////////////////////

int main(void) {
    RCC_Regs *rcc = (RCC_Regs *)RCC_START_ADDRESS;

    //  INITIALIZE SYSTICK
    systick_init(1600);
    rcc->AHB2ENR |= (1UL << 14);

    //  INITIALIZE LED
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    rcc->AHB1ENR |= (1UL << (led_pin.bank - 'A')); 
    gpio_set_mode(GPIO_MODE_OUTPUT, led_pin);

    //for (;;) {}

    bool on = true;
    gpio_write(true, led_pin);
    for (;;) {
        if (on && (num_ticks % 4000) > 2000) {
            on = !on;
            gpio_write(on, led_pin);
        }
        else if (!on && (num_ticks % 4000) < 2000) {
            on = !on;
            gpio_write(on, led_pin);
        }

    }

    return 0;
}


////////////////////////////////////////////////////////////
//  Startup code 
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
__attribute__((section(".vectors"))) void (*const volatile tab[16 + 91])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, _on_systick_interrupt};