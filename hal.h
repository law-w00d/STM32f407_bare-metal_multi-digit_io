#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#define FREQ 16000000
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count)
{
  	while (count--) (void) 0;
}

struct systick
{
  	volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)

struct rcc
{
	volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
		RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
		RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
		AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
		RESERVED6[2], SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc *) 0x40023800)

struct tim
{
	volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT,
		PSC, ARR, RESERVED1, CCR1, CCR2, CCR3, CCR4, RESERVED2, DCR, DMAR, OR;
};
#define TIM4 ((struct tim *) 0x40000800)

static inline void systick_init(uint32_t ticks)
{
	if ((ticks - 1) > 0xffffff) return;
	SYSTICK->LOAD = ticks - 1;
	SYSTICK->VAL = 0;
	SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); 
	RCC->APB2ENR |= BIT(14);                   
}

static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now)
{
	if (now + prd < *t) *t = 0;                    
	if (*t == 0) *t = now + prd;                   
	if (*t > now) return false;                    
	*t = (now - *t) > prd ? now + prd : *t + prd;  
	return true;                                   
}

struct cooldown
{
	uint32_t duration, end;
	volatile uint32_t *now;		// Points to s_ticks when initialized, but this could be removed and s_ticks could be accessed via extern
};

// Potential problem here with overflow, plan to fix later
static inline void cooldown_set(struct cooldown *cooldown)
{
	cooldown->end = *cooldown->now + cooldown->duration;
}

static inline bool cooldown_finished(struct cooldown *cooldown)
{
	return (*cooldown->now >= cooldown->end);
}

static inline void delay(uint16_t duration)
{
	RCC->APB1ENR |= BIT(2);
	TIM4->PSC = (16000 - 1);
	TIM4->ARR = duration;
	TIM4->CNT = 0;
	TIM4->CR1 |= BIT(0);
	while(!(TIM4->SR & BIT(0))){}
	TIM4->SR &= ~(BIT(0));
}

struct gpio
{
  	volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
#define GPIOB ((struct gpio *) 0x40020400)

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode)
{
	struct gpio *gpio = GPIO(PINBANK(pin)); 
	int n = PINNO(pin);                    
	RCC->AHB1ENR |= BIT(PINBANK(pin));     
	gpio->MODER &= ~(3U << (n * 2));       
	gpio->MODER |= (mode & 3U) << (n * 2);
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num)
{
	struct gpio *gpio = GPIO(PINBANK(pin));  
	int n = PINNO(pin);                      
	gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
	gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val)
{
	struct gpio *gpio = GPIO(PINBANK(pin));
	gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

static inline bool gpio_read(uint16_t pin)
{
  	struct gpio *gpio = GPIO(PINBANK(pin));
	uint32_t status = ((gpio->IDR) & BIT(PINNO(pin))) >> PINNO(pin);
	if (status)
	{
		return true;
	}
	else
	{
		return false;
	}
}

struct uart
{
  	volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)

static inline void uart_init(struct uart *uart, unsigned long baud)
{
	uint8_t af = 7;           
	uint16_t rx = 0, tx = 0;  

	if (uart == UART1) RCC->APB2ENR |= BIT(4);
	if (uart == UART2) RCC->APB1ENR |= BIT(17);
	if (uart == UART3) RCC->APB1ENR |= BIT(18);

	if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
	if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
	if (uart == UART3) tx = PIN('B', 10), rx = PIN('B', 11);

	gpio_set_mode(tx, GPIO_MODE_AF);
	gpio_set_af(tx, af);
	gpio_set_mode(rx, GPIO_MODE_AF);
	gpio_set_af(rx, af);
	uart->CR1 = 0;                           
	uart->BRR = FREQ / baud;                 
	uart->CR1 |= BIT(13) | BIT(2) | BIT(3);  
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte)
{
	uart->DR = byte;
	while ((uart->SR & BIT(7)) == 0) spin(1);
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len)
{
  	while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct uart *uart)
{
  	return uart->SR & BIT(5);  
}

static inline uint8_t uart_read_byte(struct uart *uart) 
{
  	return (uint8_t) (uart->DR & 255);
}

#define KEY_COL_SIZE 4
#define KEY_ROW_SIZE 4

static inline void keypad_init()
{
	extern const uint16_t cols[KEY_COL_SIZE];
	extern const uint16_t rows[KEY_ROW_SIZE];

	for (int c = 0; c < KEY_COL_SIZE; c++)
	{
		gpio_set_mode(cols[c], GPIO_MODE_OUTPUT);
	}

	for (int r = 0; r < KEY_ROW_SIZE; r++)
	{
		struct gpio *gpio = GPIO(PINBANK(rows[r]));
		gpio_set_mode(rows[r], GPIO_MODE_INPUT);
		gpio->PUPDR |= (BIT(1) << (PINNO(rows[r]) * 2));
	}
}

/*	Scan the keypad matrix, checking if a button is pressed and further checking the cooldown status
	If cooldown is not over for that key, or if no button was pressed, return 0
	Otherwise, return the character from the keymap */
static inline uint8_t get_key(struct cooldown *cooldown, uint8_t *lastKey)
{
	extern const uint16_t cols[KEY_COL_SIZE];
	extern const uint16_t rows[KEY_ROW_SIZE];
	extern const uint8_t keymap[KEY_COL_SIZE][KEY_ROW_SIZE];

	for (int i = 0; i < KEY_COL_SIZE; i++)
	{
		gpio_write(cols[i], true);
		for (int j = 0; j < KEY_ROW_SIZE; j++)
		{
			if (gpio_read(rows[j]))
			{
				gpio_write(cols[i], false);
				uint8_t c = keymap[i][j];
				if (c == *lastKey && !cooldown_finished(cooldown)) return 0;
				*lastKey = c;
				cooldown_set(cooldown);
				return c;
			}
		}
		gpio_write(cols[i], false);
	}
	return 0;
}

#define NUM_SEG_PINS 8
#define NUM_DIG_PINS 4
#define DISPLAY_DELAY_MS
#define PATTERN_BUF_SIZE 4

// Binary patterns for each segment of a digit
#define SEG_A BIT(0)
#define SEG_B BIT(1)
#define SEG_C BIT(2)
#define SEG_D BIT(3)
#define SEG_E BIT(4)
#define SEG_F BIT(5)
#define SEG_G BIT(6)
#define SEG_DP BIT(7)		// To write a decimal point, use the write_pattern function, but OR the pattern argument with SEG_DP

static inline void display_init()
{
	extern const uint16_t dig_pins[NUM_DIG_PINS];
	extern const uint16_t seg_pins[NUM_SEG_PINS];

	for (int i = 0; i < NUM_DIG_PINS; i++)
	{
		struct gpio *gpio = GPIO(PINBANK(dig_pins[i]));
		gpio_set_mode(dig_pins[i], GPIO_MODE_OUTPUT);
		gpio->PUPDR |= (BIT(1) << (PINNO(dig_pins[i]) * 2));
	}

	for (int i = 0; i < NUM_SEG_PINS; i++)
	{
		gpio_set_mode(seg_pins[i], GPIO_MODE_OUTPUT);
	}
}

/*	Given a character, return the index of patterns[] that will display given character */
static inline int pattern_lookup(uint8_t c)
{
	if (c >= '0' && c <= '9')
		return (c - '0');
	else
		return (0);
}

static inline uint8_t read_pattern()
{
	extern const uint16_t seg_pins[NUM_SEG_PINS];

	uint8_t pattern = 0;
	for (int i = 0; i < NUM_SEG_PINS; i++)
	{
		pattern |= (uint8_t) (gpio_read(seg_pins[i]) << BIT(i));
	}
	return pattern;
}

static inline void empty_pattern_buf(uint8_t buf[])
{
	for (int i = 0; i < PATTERN_BUF_SIZE; i++) buf[i] = 0;
}

static inline int update_pattern_buf(uint8_t buf[], uint8_t pattern)
{
	int i;
	for (i = 0; buf[i] != 0 && i < PATTERN_BUF_SIZE; i++);	// Find the first buf index that equals 0 using linear search
	if (buf[i] == 0) buf[i] = pattern;
	return i;
}

static inline void write_pattern(uint8_t pattern)
{
	extern const uint16_t seg_pins[NUM_SEG_PINS];

	for (int i = 0; i < NUM_SEG_PINS; i++)
	{
		gpio_write(seg_pins[i], (pattern & BIT(i)));
	}
}

/*	This can be used, or you can simply write_pattern(0) - where zero indicates no segments should be activated */
static inline void write_blank()
{
	extern const uint16_t seg_pins[NUM_SEG_PINS];

	for (int i = 0; i < NUM_SEG_PINS; i++)
	{
		gpio_write(seg_pins[i], false);
	}
}

/*	For each digit (from right to left), determine what its pattern should be given the current sequence of patterns
	Account for whether input is actively being accepted or not
	Account for how many (and which) digits should be blank */
static inline void display(uint8_t pattern_buf[], int buf_count, bool open)
{
	extern const uint16_t dig_pins[NUM_DIG_PINS];
	extern const uint16_t seg_pins[NUM_SEG_PINS];

	for (int i = 0; i < NUM_DIG_PINS; i++)
	{
		if (open)
		{
			write_pattern((uint8_t) 0 | SEG_G);						// Draw a dash to indicate input is currently being accepted
		}
		else if (buf_count - i < 0)
		{
			write_pattern(0);										// Draw a blank (no segments activated) to "pad" the display
		}
		else
		{
			write_pattern(pattern_buf[buf_count - i]);				// Draw patterns from end to beginning because we draw from right to left
		}

		gpio_write(dig_pins[i], true);
		delay(2);
		gpio_write(dig_pins[i], false);
	}
}