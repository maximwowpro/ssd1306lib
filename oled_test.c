#define F_CPU 16000000UL

#include "oled.h"
#include <avr/io.h>


int main()
{
	sei();
	OLED oled;
	uint8_t fb[1024] = {0};
	OLED_init(&oled, 128, 64, fb, 50000, 0b0111100);

	OLED_WITH_SPINLOCK(&oled) {
		OLED_put_rectangle(&oled, 1, 1, 126, 62, OLED_FILL | 1); /* fill black */
	}

	OLED_put_triangle(&oled, 5, 5, 50, 20, 30, 63, 2);
	OLED_put_triangle(&oled, 110, 10, 100, 40, 70, 20, 2);
	OLED_put_triangle(&oled, 60, 20, 80, 40, 50, 55, 0);
	OLED_put_triangle(&oled, 115, 40, 120, 55, 70, 50, 0);

	OLED_refresh(&oled);
	while (1) {
		OLED_refresh(&oled);
		OLED_put_triangle(&oled, 110, 10, 100, 40, 70, 20, OLED_FILL | 1);
		OLED_refresh(&oled);
		OLED_put_triangle(&oled, 110, 10, 100, 40, 70, 20, 0);
		OLED_refresh(&oled);
		OLED_put_triangle(&oled, 110, 10, 100, 40, 70, 20, 2);
		OLED_refresh(&oled);
	}
}
