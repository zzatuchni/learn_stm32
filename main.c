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

#define LED_PIN_BANK 'A'
#define LED_PIN_NUMBER 5

typedef struct {
    char bank;
    uint8_t number;
} STM32_Pin;

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
    gpio->AFR[n >> 3] |= (((uint32_t) af) << ((n & 7) * 4));     //set to new mode
}

static inline void gpio_write(bool value, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->BSRR = (1U << n) << (value ? 0 : 16);
}

////////////////////////////////////////////////////////////
//  LED
////////////////////////////////////////////////////////////

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

    //enable the interrupt
    #define NVIC_ISER1 0xE000E104
    #define UART2_INTERRUPT_NUM 38
    uint32_t *nvic_iser1 = (uint32_t *)NVIC_ISER1;
    (*nvic_iser1) |= (1UL << (UART2_INTERRUPT_NUM % 32));

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
//  Main code
////////////////////////////////////////////////////////////

void handle_command(char *command, uint8_t length) {
    uart_write_buf(command, length);
}

int main(void) {

    //  INITIALIZE SYSTICK
    systick_init(FREQ / 1000);

    //  INITIALIZE LED
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    led_init(led_pin);

    //  INITIALIZE UART
    uart_init();

    // START LOOP
    uart_write_buf(">> ", 3);
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
    0, 0, 0, 0, 0, 0, _on_uart2_interrupt, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 32 - 47
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 48 - 63
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 64 - 79
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80 - 96
    };