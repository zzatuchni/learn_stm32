#include "rtos.h"

////////////////////////////////////////////////////////////
//  Misc
////////////////////////////////////////////////////////////

#define FREQ 16000000 
#define BIT(x) (1UL << x)

 __attribute__((optimize("O0"))) static inline void spin(uint32_t count) {
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
//  Flash
////////////////////////////////////////////////////////////

#define FLASH_REGS_START_ADDRESS 0x40023C00
typedef struct {
    volatile uint32_t ACR, KEYR, OPTKEYR, SR, CR, OPTCR;
} Flash_Regs;

#define FLASH ((Flash_Regs *) FLASH_REGS_START_ADDRESS)

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

static inline void clock_init() {
    // Reset the Flash 'Access Control Register', and
    // then set 1 wait-state and enable the prefetch buffer.
    // (The device header files only show 1 bit for the F0
    //  line, but the reference manual shows 3...)
    FLASH->ACR &= ~(0x00000017U);
    FLASH->ACR |=  (0x1U |
                    (0x1U << 4U));

    // Configure the PLL to (HSI) * 3 = 48MHz.
    // Use a PLLMUL of 0xA for *12, and keep PLLSRC at 0
    // to use (HSI / PREDIV) as the core source. HSI = 8MHz.
   
    RCC->PLLCFGR  &= ~(uint32_t)(0x111111111111111U);
    RCC->PLLCFGR  &=  ~BIT(22);
    RCC->PLLCFGR  |=  (0x3U << 6U);
    // Turn the PLL on and wait for it to be ready.
    RCC->CR    |=  (BIT(24));
    while (!(RCC->CR & BIT(25))) {};

    // Select the PLL as the system clock source.
    RCC->CFGR  &= ~(0x3U);

    RCC->CFGR  |=  (0x00000002U);
    spin(2000000);
    

    while (!(RCC->CFGR & (BIT(3)))) {};
}

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

typedef enum { 
    GPIO_SPEED_LOW, GPIO_SPEED_MED, GPIO_SPEED_FAST, GPIO_SPEED_HIGH 
} GPIO_Speed;

typedef enum { 
    GPIO_PP_PULL_DOWN_ONLY, GPIO_PP_PULL_UP, GPIO_PP_PULL_DOWN 
} GPIO_PushPull;

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

static inline void gpio_set_speed(GPIO_Speed spd, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->OSPEEDR &= ~(3U << (n * 2));           //reset speed
    gpio->OSPEEDR |= ((spd & 3U) << (n * 2));    //set to new speed
}

static inline void gpio_set_otyper(bool is_open_drain, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->OTYPER &= ~(1U << (n));                     //reset otyper
    gpio->OTYPER |= ((is_open_drain & 1U) << (n));    //set to new otyper
}

static inline void gpio_set_pupdr(GPIO_PushPull pp, STM32_Pin pin) {
    GPIO_Bank *gpio = GET_GPIO_BANK(pin.bank); 
    int n = pin.number;
    gpio->PUPDR &= ~(3U << (n * 2));            //reset pupdr
    gpio->PUPDR |= ((pp & 3U) << (n * 2));      //set to new pupdr
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

//__attribute__((used)) static volatile uint32_t num_ticks;
//void _on_systick_interrupt(void) {
//    num_ticks++;
//}

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
} UART_Regs;
#define UART2 ((UART_Regs *) UART2_START_ADDRESS)

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
//__attribute__((used)) static volatile bool command_finished = false;


////////////////////////////////////////////////////////////
//  Button
////////////////////////////////////////////////////////////

#define BUTTON_PIN_BANK 'C'
#define BUTTON_PIN_NUMBER 13

static inline void button_init(STM32_Pin button_pin) { 
    gpio_set_mode(GPIO_MODE_INPUT, button_pin);

    exti_enable(button_pin, EXTI_TRIGGER_ON_FALL);
}

////////////////////////////////////////////////////////////
//  SPI / LCD
////////////////////////////////////////////////////////////

#define SPI_CS_PIN      (STM32_Pin){'B', 6}
//#define SPI_SCLK_PIN    (STM32_Pin){'B', 3}
#define SPI_SCLK_PIN    (STM32_Pin){'A', 5}
#define SPI_MISO_PIN    (STM32_Pin){'A', 6}
#define SPI_MOSI_PIN    (STM32_Pin){'A', 7}
#define SPI_DC_PIN      (STM32_Pin){'A', 10}
#define SPI_RST_PIN     (STM32_Pin){'A', 8}

#define SPI1_START_ADDRESS 0x40013000
#define SPI1_BAUD_CONTROL 0x0
typedef struct {
    volatile uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR;
} SPI_Regs;

#define SPI1 ((SPI_Regs *) SPI1_START_ADDRESS)

typedef struct {
    volatile STM32_Pin CS, SCLK, MISO, MOSI, RST, DC;
} SPI_Pins;

static inline void spi_gpio_init(SPI_Pins *spi_pins) {
    // set up gpio pins
    
    gpio_set_mode(GPIO_MODE_OUTPUT, spi_pins->CS);
    gpio_set_pupdr(GPIO_PP_PULL_DOWN_ONLY, spi_pins->CS);
    gpio_set_otyper(false, spi_pins->CS);

    gpio_set_mode(GPIO_MODE_AF, spi_pins->SCLK);
    gpio_set_speed(GPIO_SPEED_HIGH, spi_pins->SCLK);
    gpio_set_pupdr(GPIO_PP_PULL_UP, spi_pins->SCLK);
    gpio_set_af(5, spi_pins->SCLK);
    gpio_set_otyper(false, spi_pins->SCLK);

    gpio_set_mode(GPIO_MODE_AF, spi_pins->MISO);
    gpio_set_speed(GPIO_SPEED_HIGH, spi_pins->MISO);
    gpio_set_pupdr(GPIO_PP_PULL_UP, spi_pins->MISO);
    gpio_set_af(5, spi_pins->MISO);
    gpio_set_otyper(false, spi_pins->MISO);

    gpio_set_mode(GPIO_MODE_AF, spi_pins->MOSI);
    gpio_set_speed(GPIO_SPEED_FAST, spi_pins->MOSI);
    gpio_set_pupdr(GPIO_PP_PULL_UP, spi_pins->MOSI);
    gpio_set_af(5, spi_pins->MOSI);
    gpio_set_otyper(false, spi_pins->MOSI);

    gpio_set_mode(GPIO_MODE_OUTPUT, spi_pins->RST);
    gpio_set_pupdr(GPIO_PP_PULL_DOWN_ONLY, spi_pins->RST);
    gpio_set_otyper(false, spi_pins->RST);

    gpio_set_mode(GPIO_MODE_OUTPUT, spi_pins->DC);
    gpio_set_speed(GPIO_SPEED_HIGH, spi_pins->DC);
    gpio_set_pupdr(GPIO_PP_PULL_DOWN_ONLY, spi_pins->DC);
    gpio_set_otyper(false, spi_pins->DC);
    

    RCC->APB2ENR |= BIT(12);
}

static inline void spi_regs_init() {
    SPI1->CR1 &= ~BIT(6);
    RCC->APB2RSTR |=  BIT(12);
    RCC->APB2RSTR &= ~BIT(12);

    SPI1->CR1 |= BIT(0) | BIT(1) | BIT(2) | BIT(8) | BIT(9);

    SPI1->CR1 |= (SPI1_BAUD_CONTROL << 3);
    SPI1->CR1 |= BIT(6);
}

static inline void lcd_write_byte(uint8_t dat) {
    while (!(SPI1->SR & BIT(1))) {};
    *(uint8_t*)&(SPI1->DR) = dat;
}

static inline void lcd_write_word(uint16_t dat) {
    lcd_write_byte((uint8_t)(dat >> 8));
    lcd_write_byte((uint8_t)(dat & 0xFF));
}

static inline void lcd_write_cmd(SPI_Pins *spi, uint8_t cmd) {
    while ((SPI1->SR & BIT(7))) {};
    gpio_write(false, spi->DC);
    lcd_write_byte(cmd);
    while ((SPI1->SR & BIT(7))) {};
    gpio_write(true, spi->DC);
}

/*
static inline void sspi_w(uint8_t dat) {
  uint8_t sspi_i;
  // Send 8 bits, with the MSB first.
  GPIO_Bank *GPIOA = GET_GPIO_BANK('A');
  for (sspi_i = 0x80; sspi_i != 0x00; sspi_i >>= 1) {
    GPIOA->ODR &= ~(1 << PB_SCK);
    if (dat & sspi_i) {
      GPIOA->ODR |=  (1 << PB_MOSI);
    }
    else {
      GPIOA->ODR &= ~(1 << PB_MOSI);
    }
    GPIOA->ODR |=  (1 << PB_SCK);
  }
}
*/

/*
static inline void sspi_cmd(uint8_t cdat) {
  // Pull the 'D/C' pin low, write data, pull 'D/C' high.
  GPIO_Bank *GPIOA = GET_GPIO_BANK('A');
  GPIOA->ODR &= ~(1 << PB_DC);
  sspi_w(cdat);
  GPIOA->ODR |=  (1 << PB_DC);
}
*/

static inline void lcd_init(SPI_Pins *spi) {
    spi_gpio_init(spi);

    // initial values
    gpio_write(true, spi->CS);
    gpio_write(true, spi->DC);
    gpio_write(true, spi->SCLK);

    gpio_write(false, spi->RST);
    spin(2000000);
    gpio_write(true, spi->RST);
    spin(2000000);

    // set up spi register
    spi_regs_init();

    gpio_write(false, spi->CS);

    // lcd setup
    lcd_write_cmd(spi, 0xEF);
    lcd_write_byte(0x03);
    lcd_write_byte(0x80);
    lcd_write_byte(0x02);
    lcd_write_cmd(spi, 0xCF);
    lcd_write_byte(0x00);
    lcd_write_byte(0xC1);
    lcd_write_byte(0x30);
    lcd_write_cmd(spi, 0xED);
    lcd_write_byte(0x64);
    lcd_write_byte(0x03);
    lcd_write_byte(0x12);
    lcd_write_byte(0x81);
    lcd_write_cmd(spi, 0xE8);
    lcd_write_byte(0x85);
    lcd_write_byte(0x00);
    lcd_write_byte(0x78);
    lcd_write_cmd(spi, 0xCB);
    lcd_write_byte(0x39);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x00);
    lcd_write_byte(0x34);
    lcd_write_byte(0x02);
    lcd_write_cmd(spi, 0xF7);
    lcd_write_byte(0x20);
    lcd_write_cmd(spi, 0xEA);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    // PWCTR1
    lcd_write_cmd(spi, 0xC0);
    lcd_write_byte(0x23);
    // PWCTR2
    lcd_write_cmd(spi, 0xC1);
    lcd_write_byte(0x10);
    // VMCTR1
    lcd_write_cmd(spi, 0xC5);
    lcd_write_byte(0x3E);
    lcd_write_byte(0x28);
    // VMCTR2
    lcd_write_cmd(spi, 0xC7);
    lcd_write_byte(0x86);
    // MADCTL
    lcd_write_cmd(spi, 0x36);
    lcd_write_byte(0x48);
    // VSCRSADD
    lcd_write_cmd(spi, 0x37);
    lcd_write_byte(0x00);
    // PIXFMT
    lcd_write_cmd(spi, 0x3A);
    lcd_write_byte(0x55);
    // FRMCTR1
    lcd_write_cmd(spi, 0xB1);
    lcd_write_byte(0x00);
    lcd_write_byte(0x18);
    // DFUNCTR
    lcd_write_cmd(spi, 0xB6);
    lcd_write_byte(0x08);
    lcd_write_byte(0x82);
    lcd_write_byte(0x27);
    lcd_write_cmd(spi, 0xF2);
    lcd_write_byte(0x00);
    // GAMMASET
    lcd_write_cmd(spi, 0x26);
    lcd_write_byte(0x01);
    // (Actual gamma settings)
    lcd_write_cmd(spi, 0xE0);
    lcd_write_byte(0x0F);
    lcd_write_byte(0x31);
    lcd_write_byte(0x2B);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x0E);
    lcd_write_byte(0x08);
    lcd_write_byte(0x4E);
    lcd_write_byte(0xF1);
    lcd_write_byte(0x37);
    lcd_write_byte(0x07);
    lcd_write_byte(0x10);
    lcd_write_byte(0x03);
    lcd_write_byte(0x0E);
    lcd_write_byte(0x09);
    lcd_write_byte(0x00);
    lcd_write_cmd(spi, 0xE1);
    lcd_write_byte(0x00);
    lcd_write_byte(0x0E);
    lcd_write_byte(0x14);
    lcd_write_byte(0x03);
    lcd_write_byte(0x11);
    lcd_write_byte(0x07);
    lcd_write_byte(0x31);
    lcd_write_byte(0xC1);
    lcd_write_byte(0x48);
    lcd_write_byte(0x08);
    lcd_write_byte(0x0F);
    lcd_write_byte(0x0C);
    lcd_write_byte(0x31);
    lcd_write_byte(0x36);
    lcd_write_byte(0x0F);
    
    // Exit sleep mode.
    lcd_write_cmd(spi, 0x11);
    spin(2000000);
    // Display on.
    lcd_write_cmd(spi, 0x29);
    spin(2000000);
    // 'Normal' display mode.
    lcd_write_cmd(spi, 0x13);
}


////////////////////////////////////////////////////////////
//  Tasks
////////////////////////////////////////////////////////////

void handle_command(char *command, uint8_t length) {
    uart_write_buf(command, length);
}

__attribute__((used)) volatile bool         led_on = false;
void led_task() {
    STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    for (;;) {
        gpio_write(led_on, led_pin);
        led_on = !led_on;
        delay_current_task(1000);
        
    }
}

__attribute__((used)) volatile uint8_t     lcd_color_num = 0;
void button_task() {
    for(;;) {
        set_current_task_state(TASK_BLOCKED);
        lcd_color_num = (uint8_t)(lcd_color_num + 1) % 4;
    }
}

__attribute__((used)) volatile uint8_t      uart_byte = 0x00;
void uart_task() {
    for (;;) {
        set_current_task_state(TASK_BLOCKED);
        uint8_t byte = uart_byte;
        bool bkspc_check = true;
        if (byte == 0x08 || byte == 0x7F) {
            if (current_command_length > 0) {
                current_command_length--;
                uart_write_byte(byte);
            }
            bkspc_check = false;
        }
        if (!bkspc_check) continue;
        if (current_command_length < COMMAND_MAX_LENGTH) {
            command_buffer[current_command_length] = byte;
            current_command_length++;
            uart_write_byte(byte);
        }
        if (byte == '\r') {
            uart_write_buf("\r\n", 2);
            handle_command((char *)command_buffer, (uint8_t)current_command_length);
            uart_write_buf("\r\n>> ", 5);
            current_command_length = 0;
        }
    }
}

void lcd_task() {

    SPI_Pins spi;
    spi.CS      = SPI_CS_PIN;
    spi.SCLK    = SPI_SCLK_PIN;
    spi.MISO    = SPI_MISO_PIN;
    spi.MOSI    = SPI_MOSI_PIN;
    spi.DC      = SPI_DC_PIN;
    spi.RST     = SPI_RST_PIN;

    // Set column range.
    lcd_write_cmd(&spi, 0x2A);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(239U >> 8U);
    lcd_write_byte(239U & 0xFFU);
    // Set row range.
    lcd_write_cmd(&spi, 0x2B);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(319U >> 8U);
    lcd_write_byte(319U & 0xFFU);
    // Set 'write to RAM'
    lcd_write_cmd(&spi, 0x2C);
    while (1) {
        if (lcd_color_num == 0) {
            lcd_write_word(0xF000);
            //lcd_write_byte(0xF0);
            //lcd_write_byte(0x00);
        }
        else if (lcd_color_num == 1) {
            lcd_write_word(0x0F00);
            //lcd_write_byte(0x0F);
            //lcd_write_byte(0x00);
        }
        else if (lcd_color_num == 2) {
            lcd_write_word(0x00F0);
            //lcd_write_byte(0x00);
            //lcd_write_byte(0xF0);
        }
        else if (lcd_color_num == 3) {
            lcd_write_word(0x0C0C);
            //lcd_write_byte(0x0C);
            //lcd_write_byte(0x0C);
        }
    }
}

////////////////////////////////////////////////////////////
//  Interrupts
////////////////////////////////////////////////////////////

void _on_button_press(void) {
    EXTI_Regs *exti = (EXTI_Regs *)EXTI_START_ADDRESS;
    exti->PR |= (1UL << BUTTON_PIN_NUMBER);

    set_task_state_from_func(&button_task, TASK_READY);
}


void _on_uart2_interrupt(void) {
    uart_byte = uart_read_byte();
    set_task_state_from_func(&uart_task, TASK_READY);
}

void _on_hard_fault(void) {
    uart_write_buf("*HF*", 4);
}

void _on_mem_fault(void) {
    uart_write_buf("*MF*", 4);
}

void _on_bus_fault(void) {
    uart_write_buf("*BF*", 4);
}

void _on_usage_fault(void) {
    uart_write_buf("*UF*", 4);
}


////////////////////////////////////////////////////////////
//  Main code
////////////////////////////////////////////////////////////

int main(void) {
    //  INITIALIZE CLOCK
    clock_init();

    //  INITIALIZE SYSTICK
    systick_init(FREQ / 1000);

    //  INITIALIZE UART
    uart_init();

    //  INITIALIZE LED
    //STM32_Pin led_pin = {LED_PIN_BANK, LED_PIN_NUMBER};
    //led_init(led_pin);

    //  INITIALIZE BUTTON
    STM32_Pin button_pin = {BUTTON_PIN_BANK, BUTTON_PIN_NUMBER};
    button_init(button_pin);

    //  INITIALIZE LCD
    SPI_Pins spi;
    spi.CS      = SPI_CS_PIN;
    spi.SCLK    = SPI_SCLK_PIN;
    spi.MISO    = SPI_MISO_PIN;
    spi.MOSI    = SPI_MOSI_PIN;
    spi.DC      = SPI_DC_PIN;
    spi.RST     = SPI_RST_PIN;

    lcd_init(&spi);

    //  INITIALIZE SCHEDULER
    add_task(&button_task, 4);
    add_task(&uart_task, 3);
    add_task(&lcd_task, 2);
    scheduler_init();

    for (;;) {}
    
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
    _estack, _reset, 0, _on_hard_fault, _on_mem_fault, _on_bus_fault, _on_usage_fault, 0, 0, 0, 0, 0, 0, 0, 0, _on_scheduler_invoked,  //ARM core interrupts
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0 - 15
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16 - 31
    0, 0, 0, 0, 0, 0, _on_uart2_interrupt, 0, _on_button_press, 0, 0, 0, 0, 0, 0, 0, // 32 - 47
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 48 - 63
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 64 - 79
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80 - 96
    };
