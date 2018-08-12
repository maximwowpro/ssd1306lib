#define F_CPU 16000000UL

#include "oled.h"
#include <avr/io.h>


int main()
{
	/* output by letter? (not related to lib) */
	bool byletter = true;

	sei();
	OLED oled;
	uint8_t fb[1024] = {0};
	OLED_init(&oled, 128, 64, fb, 50000, 0b0111100);

	/* Try to decrease frequency and see what happens without lock */
	OLED_WITH_SPINLOCK(&oled) {
		OLED_put_rectangle(&oled, 1, 1, 126, 62, OLED_FILL | 1); /* fill black */
// 		OLED_put_rectangle(&oled, 4, 4, 123, 57, 0);		 /* fill black */
	}
	if (byletter) OLED_refresh(&oled);
	
// 	OLED_put_line(&oled, 5, 5, 5, 30, 0); /* vertical  */
// 	
// 	OLED_put_line(&oled, 5, 5, 50, 5, 0); /* horizontal */
// 	
// 	OLED_put_line(&oled, 5, 5, 45, 45, 0); /* 45 degrees */
// 	
// 	OLED_put_line(&oled, 5, 5, 25, 45, 0); /* delta_big % delta_small == 0 */
// 	
// 	OLED_put_line(&oled, 5, 5, 15, 45, 0); /* delta_big % delta_small == 0 */
// 	
// 	OLED_put_line(&oled, 5, 5, 100, 15, 0); /* delta_big % delta_small == 0 */

//	OLED_put_line(&oled, 100, 17, 0, 0, 0); /* 45 degrees */
//	OLED_put_line(&oled, 0, 50, 50, 0, 0); /* 45 degrees */
	//OLED_put_line(&oled, 50, 0, 0, 50, 0); /* 45 degrees */
	OLED_put_line(&oled, 0, 0, 50, 50, 0); /* 45 degrees */
	OLED_put_line(&oled, 0, 45, 30, 0, 0); /* 45 degrees */
	OLED_put_line(&oled, 30, 0, 0, 45, 0); /* 45 degrees */
//	OLED_put_line(&oled, 0, 0, 50, 50, 0); /* 45 degrees */
//	OLED_put_line(&oled, 50, 0, 0, 50, 0); /* 45 degrees */
	
// 	// T letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 17, 22, 30, 25, OLED_FILL | 0);
// 	OLED_put_rectangle(&oled, 22, 26, 25, 39, OLED_FILL | 0);
// 	if (byletter) OLED_refresh(&oled);
// 
// 	// E letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 34, 22, 45, 39, OLED_FILL | 0);
// 	OLED_put_rectangle(&oled, 38, 26, 45, 28, OLED_FILL | 1);
// 	OLED_put_rectangle(&oled, 42, 29, 45, 32, OLED_FILL | 1);
// 	OLED_put_rectangle(&oled, 38, 33, 45, 35, OLED_FILL | 1);
// 	if (byletter) OLED_refresh(&oled);
// 
// 	// S letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 48, 22, 59, 39, OLED_FILL | 0);
// 	OLED_put_rectangle(&oled, 52, 26, 59, 28, OLED_FILL | 1);
// 	OLED_put_rectangle(&oled, 48, 33, 55, 35, OLED_FILL | 1);
// 	if (byletter) OLED_refresh(&oled);
// 
// 	// T letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 62, 22, 75, 25, OLED_FILL | 0);
// 	OLED_put_rectangle(&oled, 67, 25, 70, 39, OLED_FILL | 0);
// 	if (byletter) OLED_refresh(&oled);
// 
// 	// O letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 86, 22, 96, 39, OLED_FILL | 0);
// 	OLED_put_rectangle(&oled, 89, 25, 93, 36, OLED_FILL | 1);
// 	if (byletter) OLED_refresh(&oled);
// 
// 	// K letter drawn with rectangles
// 	OLED_put_rectangle(&oled, 99, 22, 102, 39, OLED_FILL | OLED_WHITE);
// 	OLED_put_rectangle(&oled, 105, 31, 100, 28, OLED_FILL | OLED_WHITE);
// 	OLED_put_rectangle(&oled, 109, 32, 106, 39, OLED_FILL | OLED_WHITE);
// 	OLED_put_rectangle(&oled, 104, 30, 107, 33, OLED_FILL | OLED_WHITE);
// 	OLED_put_rectangle(&oled, 107, 29, 104, 26, OLED_FILL | OLED_WHITE);
// 	OLED_put_rectangle(&oled, 109, 27, 106, 22, OLED_FILL | OLED_WHITE);
// 
// 	if (byletter) OLED_refresh(&oled);

	bool color = OLED_BLACK;
	while (1) {
		/* Horizontal line of 1 px width */
		OLED_WITH_SPINLOCK(&oled) {
//			OLED_put_rectangle(&oled, 10, 47, 117, 47, color);
			
		}
		color = !color;
		OLED_refresh(&oled);
	}
}
