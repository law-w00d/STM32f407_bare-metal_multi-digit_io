#include "hal.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void)
{
	s_ticks++;
}

// Maps the physical keypad keys to a 2D array
const uint8_t keymap[KEY_COL_SIZE][KEY_ROW_SIZE] = {
	{'1', '4', '7', '*'},
	{'2', '5', '8', '0'},
	{'3', '6', '9', '#'},
	{'A', 'B', 'C', 'D'}
};

// Set segment patterns for displaying digits 0 - 9
const uint8_t seg_patterns[] = {
	(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F), 			// 0
	(SEG_B | SEG_C),											// 1
	(SEG_A | SEG_B | SEG_D | SEG_E | SEG_G),					// 2
	(SEG_A | SEG_B | SEG_C | SEG_D | SEG_G),					// 3
	(SEG_B | SEG_C | SEG_F | SEG_G),							// 4
	(SEG_A | SEG_C | SEG_D | SEG_F | SEG_G),					// 5
	(SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),			// 6
	(SEG_A | SEG_B | SEG_C),									// 7
	(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),	// 8
	(SEG_A | SEG_B | SEG_C | SEG_F | SEG_G)						// 9
};

// Setup the keypad pins for its rows and cols
const uint16_t cols[KEY_COL_SIZE] = {PIN('D', 5), PIN('D', 7), PIN('B', 4), PIN('B', 6)};
const uint16_t rows[KEY_ROW_SIZE] = {PIN('C', 10), PIN('C', 12), PIN('D', 1), PIN('D', 3)};

// Setup the display's 8 output pins {A, B, C, D, E, F, G, DP} and 4 digit pins {d0, d1, d2, d3}
const uint16_t dig_pins[NUM_DIG_PINS] = {PIN('C', 15), PIN('C', 13), PIN('E', 5), PIN('E', 3)};
const uint16_t seg_pins[NUM_SEG_PINS] = {PIN('B', 5), PIN('B', 3), PIN('D', 6), PIN('D', 4), PIN('D', 2), PIN('D', 0), PIN('C', 11), PIN('A', 15)};

int main(void)
{
	uint16_t blue = PIN('D', 15);
	gpio_set_mode(blue, GPIO_MODE_OUTPUT);

	uart_init(UART2, 115200);
	printf("uart initialized\r\n");
	systick_init(FREQ / 1000);
	printf("systick initialized\r\n");

	uint32_t t_led = 0, p_led = 500;						// Timer and period for LED blink
	struct cooldown key_cooldown = {500, 0, &s_ticks};		// Cooldown to prevent accidental duplicate input
	uint8_t last_key = 0;									// Track the key we don't want duplicated (cooldown only applies to this key)

	keypad_init();
	printf("keypad pins initialized\r\n");
	
	display_init();
	printf("display initialized\r\n");

	bool open = false;										// Track whether input can be accepted or not
	uint8_t pattern_buf[PATTERN_BUF_SIZE];					// Store the sequence of patterns for current input			
	int buf_count;
	empty_pattern_buf(pattern_buf);							// Initialize buffer

	printf("main loop starting\r\n");
	for (;;)
	{
		uint8_t input = get_key(&key_cooldown, &last_key);
		if (input) printf("%c\r\n", input);
		if (input == '#')		// '#' key toggles input mode ON and OFF
		{
			if (!open) empty_pattern_buf(pattern_buf);
			open = !open;
			input = 0;			// We already made use of '#' so we can zero this variable now - makes the below IF condition simpler
		}

		if (open && input)
		{
			buf_count = update_pattern_buf(pattern_buf, seg_patterns[pattern_lookup(input)]);
			if (buf_count >= PATTERN_BUF_SIZE - 1) open = false;		// Detect if buffer is now full - if so, toggle input mode OFF
		}
		
		display(pattern_buf, buf_count, open);

		// Blue LED control
		if (timer_expired(&t_led, p_led, s_ticks))
		{
			static bool on;       
			gpio_write(blue, on);
			on = !on;
		}
	}
	return 0;
}		