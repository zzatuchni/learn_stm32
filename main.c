#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////
//  Misc
////////////////////////////////////////////////////////////

#define FREQ 16000000 
#define BIT(x) (1UL << x)

static inline void spin(uint32_t count) {
    while (count--) asm("nop");
}

////////////////////////////////////////////////////////////
//  Pin
////////////////////////////////////////////////////////////

typedef struct {
    char bank;
    uint8_t number;
} STM32_Pin;


////////////////////////////////////////////////////////////
//  Syscfg
////////////////////////////////////////////////////////////

#define SYSCFG_START_ADDRESS 0x40013800

typedef struct {
    volatile uint32_t MEMRMP, PCM, EXTICR[4], CMPCR, CFGR;
} Syscfg_Regs;

////////////////////////////////////////////////////////////
//  Interrupts
////////////////////////////////////////////////////////////

#define NVIC_ISER(x) (0xE000E100 + (0x04 * (x)))

static inline void enable_interrupt(uint8_t interrupt_num) {
    uint32_t *nvic_iser1 = (uint32_t *)NVIC_ISER(interrupt_num / 32);
    (*nvic_iser1) |= (1UL << (interrupt_num % 32));
}

////////////////////////////////////////////////////////////
//  EXTI
////////////////////////////////////////////////////////////

#define EXTI_START_ADDRESS 0x40013C00

#define EXTI0_INTERRUPT_NUM 6
#define EXTI1_INTERRUPT_NUM 7
#define EXTI2_INTERRUPT_NUM 8
#define EXTI3_INTERRUPT_NUM 9
#define EXTI4_INTERRUPT_NUM 10
#define EXTI9_5_INTERRUPT_NUM 23
#define EXTI15_10_INTERRUPT_NUM 40

typedef struct {
    volatile uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR;
} EXTI_Regs;

typedef enum { 
    EXTI_NO_TRIGGER, EXTI_TRIGGER_ON_RISE, EXTI_TRIGGER_ON_FALL, EXTI_TRIGGER_ON_BOTH 
} EXTI_Mode;

static inline uint8_t exti_get_interrupt_num(uint8_t line_number) {
    if (line_number <= 4) return line_number + EXTI0_INTERRUPT_NUM;
    else if (line_number <= 9) return EXTI9_5_INTERRUPT_NUM;
    else return EXTI15_10_INTERRUPT_NUM;
}

static inline void exti_enable(STM32_Pin pin, EXTI_Mode trigger_mode) {
    EXTI_Regs *exti = (EXTI_Regs *)EXTI_START_ADDRESS;
    Syscfg_Regs *syscfg = (Syscfg_Regs *)SYSCFG_START_ADDRESS;

    syscfg->EXTICR[pin.number / 4] &= ~(0b1111UL << ((uint32_t)(pin.number % 4) * 4));
    syscfg->EXTICR[pin.number / 4] |= ((uint32_t)(pin.bank - 'A') << ((uint32_t)(pin.number % 4) * 4));

    exti->IMR |= (1UL << pin.number);

    exti->RTSR &= ~(1UL << pin.number);
    exti->RTSR |= ((uint32_t)(trigger_mode & 0b01) << pin.number);
    exti->FTSR &= ~(1UL << pin.number);
    exti->FTSR |= ((uint32_t)((trigger_mode & 0b10) / 2) << pin.number);

    uint8_t interrupt_num = exti_get_interrupt_num(pin.number);
    enable_interrupt(interrupt_num);

    //exti->SWIER |= (1UL << line_number);
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
#define RCC ((RCC_Regs *) RCC_START_ADDRESS)

////////////////////////////////////////////////////////////
//  GPIO
////////////////////////////////////////////////////////////

#define GPIOA_START_ADDRESS 0x40020000
#define GPIO_BANK_BOUNDARY_SIZE 0x400

typedef struct {
    volatile uint32_t 
        MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_Bank;
#define GET_GPIO_BANK(bank) (GPIO_Bank *) ((GPIOA_START_ADDRESS) + (GPIO_BANK_BOUNDARY_SIZE) * ((bank) - 'A'))

typedef enum { 
    GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG 
} GPIO_Mode;

typedef uint8_t GPIO_AF;

static inline void gpio_set_mode(GPIO_Mode mode, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    RCC->AHB1ENR |= (1UL << (pin.bank - 'A'));
    gpio->MODER &= ~(3U << (n * 2));            //reset mode
    gpio->MODER |= ((mode & 3U) << (n * 2));    //set to new mode
}

static inline void gpio_set_af(GPIO_AF af, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));              //reset af
    gpio->AFR[n >> 3] |= (((uint32_t) af) << ((n & 7) * 4));    //set to new mode
}

static inline void gpio_write(bool value, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->BSRR = (1U << n) << (value ? 0 : 16);
}

static inline bool gpio_read(STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    return (gpio->IDR & (1U << n));
}


////////////////////////////////////////////////////////////
//  LED
////////////////////////////////////////////////////////////

#define LED_PIN_BANK 'A'
#define LED_PIN_NUMBER 5

static inline void led_init(STM32_Pin led_pin) { 
    gpio_set_mode(GPIO_MODE_OUTPUT, led_pin);
}

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
    RCC->APB2ENR |= BIT(14);
}

__attribute__((used)) static volatile uint32_t num_ticks;
void _on_systick_interrupt(void) {
    num_ticks++;
}

////////////////////////////////////////////////////////////
//  UART
////////////////////////////////////////////////////////////

#define UART2_START_ADDRESS 0x40004400

#define UART2_PIN_BANK 'A'
#define UART2_PIN_NUMBER_TX 2
#define UART2_PIN_NUMBER_RX 3
#define UART2_AF_NUMBER 7
#define UART2_BAUD_RATE 115200

#define UART2_INTERRUPT_NUM 38

typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} UART_REGS;
#define UART2 ((UART_REGS *) UART2_START_ADDRESS)

static inline void uart_init() {
    RCC->APB1ENR |= BIT(17);

    STM32_Pin uart2_tx_pin = {UART2_PIN_BANK, UART2_PIN_NUMBER_TX};
    STM32_Pin uart2_rx_pin = {UART2_PIN_BANK, UART2_PIN_NUMBER_RX};

    gpio_set_mode(GPIO_MODE_AF, uart2_tx_pin);
    gpio_set_mode(GPIO_MODE_AF, uart2_rx_pin);

    gpio_set_af(UART2_AF_NUMBER, uart2_tx_pin);
    gpio_set_af(UART2_AF_NUMBER, uart2_rx_pin);

    UART2->CR1 = 0;                           // Disable this UART
    UART2->BRR = FREQ / UART2_BAUD_RATE;      // FREQ is a UART bus frequency
    UART2->CR1 |= BIT(13) | BIT(2) | BIT(3) | BIT(5);  // Set UE, RE, TE, RXNEIE

    enable_interrupt(UART2_INTERRUPT_NUM);
}

static inline uint8_t uart_read_byte() {
    return (uint8_t) (UART2->DR & 255);
}

static inline int uart_read_ready() {
    return (UART2->SR & BIT(5));
}

static inline void uart_write_byte(uint8_t byte) {
    UART2->DR = byte;
    while ((UART2->SR & BIT(7)) == 0) spin(1);
}

static inline void uart_write_buf(char *buf, size_t len) {
    while (len-- > 0) uart_write_byte(*(uint8_t *) buf++);
}

////////////////////////////////////////////////////////////
//  Command processing
////////////////////////////////////////////////////////////

#define COMMAND_MAX_LENGTH 64
__attribute__((used)) static volatile uint8_t command_buffer[COMMAND_MAX_LENGTH];
__attribute__((used)) static volatile uint8_t current_command_length = 0;
__attribute__((used)) static volatile bool command_finished = false;

void _on_uart2_interrupt(void) {
    uint8_t byte = uart_read_byte();
    if (byte == 0x08 || byte == 0x7F) {
        if (current_command_length > 0) {
            current_command_length--;
            uart_write_byte(byte);
        }
        return;
    }
    if (current_command_length < COMMAND_MAX_LENGTH) {
        command_buffer[current_command_length] = byte;
        current_command_length++;
        uart_write_byte(byte);
    }
    if (byte == '\r') {
        command_finished = true;
    }
}

////////////////////////////////////////////////////////////
//  Button
////////////////////////////////////////////////////////////

#define BUTTON_PIN_BANK 'C'
#define BUTTON_PIN_NUMBER 13

static inline void button_init(STM32_Pin button_pin) { 
    gpio_set_mode(GPIO_MODE_INPUT, button_pin);

    exti_enable(button_pin, EXTI_TRIGGER_ON_FALL);
}

__attribute__((used)) static volatile bool led_on = false;

void _on_button_press(void) {
    EXTI_Regs *exti = (EXTI_Regs *)EXTI_START_ADDRESS;
    exti->PR |= (1UL << BUTTON_PIN_NUMBER);
    
    led_on = !led_on;
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    gpio_write(led_on, led_pin);
}

////////////////////////////////////////////////////////////
//  Main code
////////////////////////////////////////////////////////////

void handle_command(char *command, uint8_t length) {
    uart_write_buf(command, length);
}

int main(void) {

    //  INITIALIZE SYSTICK
    systick_init(FREQ / 1000);

    //  INITIALIZE UART
    uart_init();

    //  INITIALIZE LED
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    led_init(led_pin);

    //  INITIALIZE BUTTON
    STM32_Pin button_pin = {BUTTON_PIN_BANK, BUTTON_PIN_NUMBER};
    button_init(button_pin);

    // START LOOP

    uart_write_buf("\r\n>> ", 5);
    for (;;) {
        if (command_finished) {
            uart_write_buf("\r\n", 2);
            handle_command((char *)command_buffer, (uint8_t)current_command_length);
            uart_write_buf("\r\n>> ", 5);
            current_command_length = 0;
            command_finished = false;
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
__attribute__((section(".vectors"))) void (*const volatile tab[16 + 99])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, _on_systick_interrupt,  //ARM core interrupts
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0 - 15
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16 - 31
    0, 0, 0, 0, 0, 0, _on_uart2_interrupt, 0, _on_button_press, 0, 0, 0, 0, 0, 0, 0, // 32 - 47
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 48 - 63
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 64 - 79
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80 - 96
    };