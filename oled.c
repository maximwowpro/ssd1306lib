/* MIT License
 * 
 * Copyright 2018, Tymofii Khodniev <thodnev @ github>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 */

#include "oled.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if !defined(OLED_NO_I2C)
/***** I2C-related logic *****/
uint8_t OLED_cmdbuffer[OLED_CMDBUFFER_LEN];

static uint8_t _i2c_cmd_init[] = {
	0x80, 0x8D, 0x80, 0x14	/* Enable charge pump	 */
	,0x80, 0xAF		/* Display on	      	 */
	,0x80, 0x81, 0x80, 0xFF /* Set brightness to 255 */
	,0x80, 0xA7		/* Enable inversion 	 */
};

static uint8_t _i2c_cmd_setpage[] = {
	0x80, 0x00, 0x80, 0x10, /* Set column cursor to 0 */
	0x80, 0xB0 /* Last nibble in 0xB0 defines page (0xB0..0xB7) */
};

static uint8_t _i2c_cmd_setbrightness[] = {
	0x80, 0x81, 0x80, 0xFF  /* Last byte is brightness level (0..255) */
};

static uint8_t _i2c_cmd_dataprefix[] = {0x40};

static uint8_t i2c_devaddr;
static uint8_t *i2c_prefix_ptr;
static uint8_t *i2c_prefix_count;
static uint8_t *i2c_data_ptr;
static uint16_t i2c_data_count;
static bool i2c_is_fastfail;
static void (*i2c_callback)(void *); /* called after transaction finish */
static void *i2c_callback_args;

/* States used in ISR FSM */
enum I2C_State_e {
	I2C_STATE_IDLE = 0,
	I2C_STATE_STOP,
	I2C_STATE_SLAVEADDR,
	I2C_STATE_WRITEPREFIX,
	I2C_STATE_WRITEBYTE
};
static enum I2C_State_e i2c_state = I2C_STATE_IDLE;


static void I2C_init(uint32_t hz_freq)
{
	i2c_state = I2C_STATE_IDLE;
	/* Enable the Two Wire Interface module */
	power_twi_enable();

	/* Select TWBR and TWPS based on frequency. Quite tricky, the main point */
	/* is that prescaler is a pow(4, TWPS)				 	 */
	/* TWBR * TWPS_prescaler value */
	uint32_t twbr = F_CPU / (2 * hz_freq) - 8;
	uint8_t twps;
	for (twps = 0; twps < 4; twps++) {
		if (twbr <= 255)
			break;
		twbr /= 4;
	}

	TWBR = (uint8_t)twbr;
	TWSR = (TWSR & 0xFC) | (twps & 0x03);

	TWCR = (1 << TWEN) | (1 << TWIE);
}


bool OLED_i2c_tx_shed(uint8_t addr, uint8_t *prefix, uint8_t prefix_len, uint8_t *bytes, uint16_t bytes_len, 
		      void (*end_cbk)(void *), void *cbk_args, bool fastfail)
{
	bool ret = false;
	/* No interrupts can occur while this block is executed */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (i2c_state == I2C_STATE_IDLE) {
			i2c_prefix_ptr = prefix;
			i2c_prefix_count = prefix_len;
			i2c_data_ptr = bytes;
			i2c_data_count = bytes_len;
			i2c_is_fastfail = fastfail;
			i2c_callback = end_cbk;
			i2c_callback_args = cbk_args;
			/* Send START signal and initiating new transaction */
			i2c_state = I2C_STATE_SLAVEADDR;
			i2c_devaddr = (addr << 1);
			TWCR |= (1 << TWSTA) | (1 << TWINT);
			ret = true;
		}
	}
	return ret;
}


ISR(TWI_vect, ISR_BLOCK)
{
	switch(i2c_state) {
	case(I2C_STATE_IDLE):
	case(I2C_STATE_STOP):
		/* transfer stop and go to IDLE*/
		/* signal with callback that transaction is over */
		TWCR |= (1 << TWSTO) | (1 << TWINT);
		i2c_state = I2C_STATE_IDLE;
		(*i2c_callback)(i2c_callback_args);
		break;
	case(I2C_STATE_SLAVEADDR):
		// load value
		TWDR = i2c_devaddr;
		TWCR = (TWCR & ~(1 << TWSTA)) | (1 << TWINT);
		if ((NULL == i2c_prefix_ptr) && (NULL == i2c_data_ptr)) {
			i2c_state = I2C_STATE_STOP;
		} else if (NULL == i2c_prefix_ptr) {
			i2c_state = I2C_STATE_WRITEBYTE;
		} else {
			i2c_state = I2C_STATE_WRITEPREFIX;
		}
		break;
	case(I2C_STATE_WRITEPREFIX):
		// load next byte of prefix
		TWDR = *i2c_prefix_ptr++;
		i2c_prefix_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_prefix_count) {
			i2c_state = (NULL == i2c_data_ptr) ? I2C_STATE_STOP : I2C_STATE_WRITEBYTE;
		}
		break;
	case(I2C_STATE_WRITEBYTE):
		// load next byte
		TWDR = *i2c_data_ptr++;
		i2c_data_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_data_count)
			i2c_state = I2C_STATE_STOP;
		break;
	}
}


/* Callback which essentially does nothing */
static void OLED_cbk_empty(void *args)
{
	// empty callback
}


/* A dummy callback which simply unlocks the oled lock */
static void OLED_cbk_unlock(void *args)
{
	OLED *oled = args;
	OLED_unlock(oled);
}


/* Callbacks which are used to write each page */
static void OLED_cbk_writepage(void *args);
static void OLED_cbk_setwritepage(void *args);
/* Writes page. This is called after OLED_cbk_setwritepage */
static void OLED_cbk_writepage(void *args)
{
	OLED *oled = args;
	if (oled->cur_page >= oled->num_pages) {
		OLED_unlock(oled);
		return;
	}
	uint8_t *lineptr = &oled->frame_buffer[oled->cur_page * (uint16_t)oled->width];
	oled->cur_page++;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_dataprefix, OLED_ARR_SIZE(_i2c_cmd_dataprefix), 
				lineptr, oled->width,
				&OLED_cbk_setwritepage, oled, true)) {
		// nop
	}
}

/* Sets page index and calls OLED_cbk_writepage via callback */
static void OLED_cbk_setwritepage(void *args)
{
	OLED *oled = args;
	_i2c_cmd_setpage[OLED_ARR_SIZE(_i2c_cmd_setpage) - 1] = 0xB0 | oled->cur_page;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setpage, 
                                OLED_ARR_SIZE(_i2c_cmd_setpage), NULL, 0,
				&OLED_cbk_writepage, oled, true)) {
		// nop
	}
}



void OLED_cmd_setbrightness(OLED *oled, uint8_t level)
{
	_i2c_cmd_setbrightness[OLED_ARR_SIZE(_i2c_cmd_setbrightness) - 1] = level;
	OLED_spinlock(oled);
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setbrightness, 
                                OLED_ARR_SIZE(_i2c_cmd_setbrightness), NULL, 0,
				&OLED_cbk_unlock, oled, true)) {
		// nop
	}
}


void OLED_refresh(OLED *oled)
{
	OLED_spinlock(oled);
	/* Code below is executed under lock */
	oled->cur_page = 0;
	OLED_cbk_setwritepage(oled);
	/* Lock is unlocked after series of callbacks, in the last one */
}
#endif // OLED_NO_I2C


/***** Display-related logic *****/
OLED_err __OLED_init(OLED *oled, uint8_t width, uint8_t height, uint8_t *frame_buffer, uint32_t i2c_freq_hz, uint8_t i2c_addr)
{
	oled->width = width;
	oled->height = height;
	oled->frame_buffer = frame_buffer;
	oled->busy_lock = 1;	/* Initially: 1 - unlocked */

	OLED_I2CWRAP(
		oled->i2c_addr = i2c_addr;
		oled->cur_page = 0;
		oled->num_pages = 8;

		I2C_init(i2c_freq_hz);
		
		if (!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_init, OLED_ARR_SIZE(_i2c_cmd_init),
				      NULL, 0, OLED_cbk_empty, NULL, true)) {
			return OLED_EBUSY;
		}
	) // OLED_I2CWRAP

	return OLED_EOK;
}


OLED_err OLED_put_pixel(OLED *oled, uint8_t x, uint8_t y, bool pixel_state)
{
	if ((x >= oled->width) || (y >= oled->height))
		return OLED_EBOUNDS;
	OLED_put_pixel_(oled, x, y, pixel_state);	/* Use inline */
	return OLED_EOK;
}


OLED_err OLED_put_rectangle(OLED *oled, uint8_t x_from, uint8_t y_from, uint8_t x_to, uint8_t y_to, enum OLED_params params)
{
	if (params > (OLED_BLACK | OLED_FILL))
		return OLED_EPARAMS;
	bool pixel_color = (OLED_BLACK & params) != 0;
	bool is_fill = (OLED_FILL & params) != 0;

	/* Limit coordinates to display bounds */
	uint8_t size_errors = 0;
	uint8_t w_max = oled->width - 1;
	uint8_t h_max = oled->height - 1;
	if (x_from > w_max) {
		x_from = w_max;
		size_errors++;
	}
	if (x_to > w_max) {
		x_to = w_max;
		size_errors++;
	}
	if (y_from > h_max) {
		y_from = h_max;
		size_errors++;
	}
	if (y_to > h_max) {
		y_to = h_max;
		size_errors++;
	}
	/* If all coordinates are out of bounds */
	if (size_errors >= 4)
		return OLED_EBOUNDS;

	//OLED_WITH_SPINLOCK(oled) {
		/* Normalize coordinates */
		/* start_@ indicates coordinates of upper left corner  */
		/* stop_@ indicates coordinates of bottom right corner */
		uint8_t start_x = x_to < x_from ? x_to : x_from; /* x min */
		uint8_t start_y = y_to < y_from ? y_to : y_from; /* y min */
		uint8_t stop_x = x_to > x_from ? x_to : x_from;  /* x max */
		uint8_t stop_y = y_to > y_from ? y_to : y_from;  /* y max */

		if (is_fill) {
			/* Fill whole area */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				for (uint8_t y = start_y; y <= stop_y; y++) {
					OLED_put_pixel_(oled, x, y, pixel_color);
				}
			}
		} else {
			/* Draw outer frame */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				OLED_put_pixel_(oled, x, start_y, pixel_color);
				OLED_put_pixel_(oled, x, stop_y, pixel_color);
			}
			for (uint8_t y = start_y; y <= stop_y; y++) {
				OLED_put_pixel_(oled, start_x, y, pixel_color);
				OLED_put_pixel_(oled, stop_x, y, pixel_color);
			}
		}
	//}

	return OLED_EOK;
}


OLED_err OLED_put_line(OLED *oled, uint8_t x_from, uint8_t y_from, uint8_t x_to, uint8_t y_to, enum OLED_params params)
{
	if (params > (OLED_BLACK | OLED_FILL))
		return OLED_EPARAMS;
	bool pixel_color = (OLED_BLACK & params) != 0;
	bool is_fill = (OLED_FILL & params) != 0;
	
	/* Limit coordinates to display bounds */
	uint8_t size_errors_x = 0;
	uint8_t size_errors_y = 0;
	uint8_t w_max = oled->width - 1;
	uint8_t h_max = oled->height - 1;
	if (x_from > w_max) {
		x_from = w_max;
		size_errors_x++;
	}
	if (x_to > w_max) {
		x_to = w_max;
		size_errors_x++;
	}
	if (y_from > h_max) {
		y_from = h_max;
		size_errors_y++;
	}
	if (y_to > h_max) {
		y_to = h_max;
		size_errors_y++;
	}
	/* If we can't draw the line at display */
	if (size_errors_x >= 2 || size_errors_y >= 2 )
		return OLED_EBOUNDS;
	

	/* Work with coordinates:
	 * 
	* There may be 4 situations:
	* 1) start - top    left  ; end - bottom right
	* 2) start - bottom right ; end - top    left
	* 3) start - top    right ; end - bottom left
	* 4) start - bottom left  ; end - top    right
	*/

	uint8_t start_x = x_from; 
	uint8_t start_y = y_from; 
	uint8_t stop_x = x_to;     
	uint8_t stop_y = y_to;    
	
	/* This code has handlers only for situation_1 and situation_4, so
	 * if we have situation_2 or situation_3 - convert them to situation_1 or situation_4 
	 */
	if ( (x_from >= x_to && y_from >= y_to)
	  || (x_from >  x_to && y_from <  y_to) ) {
		start_x = x_to;
		start_y = y_to;
		stop_x = x_from;
		stop_y = y_from;
	}
	
	/* If the line is horizontal */
	if (start_y == stop_y) {
		for (uint8_t x = start_x; x <= stop_x; x++)
			OLED_put_pixel_(oled, x, start_y, pixel_color);
		return OLED_EOK;
	}
	
	/* If the line is vertical */
	if (start_x == stop_x) {
		for (uint8_t y = start_y; y <= stop_y; y++)
			OLED_put_pixel_(oled, start_x, y, pixel_color);
		return OLED_EOK;
	}

	/* If the line is  45-degrees inclined line */
	if ( (stop_x - start_x == start_y - stop_y)
	  && (start_x < stop_x && start_y > stop_y) ) {		/* situation_4 handler */
		uint8_t x = start_x, y = start_y;
		for (; x <= stop_x; x++, y--)
			OLED_put_pixel_(oled, x, y, pixel_color);
		return OLED_EOK;
	} else if (stop_x - start_x == stop_y - start_y) {	/* situation_1 handler */
		uint8_t x = start_x, y = start_y;
		for (; x <= stop_x, y <= stop_y; x++, y++)
				OLED_put_pixel_(oled, x, y, pixel_color);
		return OLED_EOK;
	}
	
	/* If the line is inclined in NOT 45 degrees*/
	uint8_t delta_small = 0;
	uint8_t delta_big   = 0;
	
	if (start_x < stop_x && start_y < stop_y) { 	  	/* situation_1 handler */
		if (stop_x - start_x > stop_y - start_y) {
			delta_big = stop_x - start_x;
			delta_small = stop_y - start_y;
		} else if (stop_x - start_x < stop_y - start_y) {
			delta_big = stop_y - start_y;
			delta_small = stop_x - start_x;
		}
	} else if (start_x < stop_x && start_y > stop_y) { 	/* situation_4 handler */
		if (stop_x - start_x > start_y - stop_y) {
			delta_big = stop_x - start_x;
			delta_small = start_y - stop_y;
		} else if (stop_x - start_x < start_y - stop_y) {
			delta_big = start_y - stop_y;
			delta_small = stop_x - start_x;
		}
	}
	
	/* In the matrix-like display we can't draw inclined line, 
	 * but we can represent it in the form of stairs, 
	 * which will approximated to inclined line by human's vision.
	 * num_lines is a number of this stairs.
	 * Example:
	 * 000000
	 *       000000
	 * 	       00000
	 * 	            000000
	 * 		          000000
	 * 		                00000
	 * is approximated form of inclined line.
	 */
	
	uint8_t num_lines = delta_small;
	
	/* There can be 2 situations:
	 * - delta_big % delta_small == 0
	 * - delta_big % delta_small != 0
	 * In the first case the lenght of every line (stair) == delta_big / delta_small.
	 */
	uint8_t line_lenght = delta_big / delta_small; /* line_lenght means short_line lenght */
	
	/* Now we work with first case (delta_big % delta_small == 0) */
	if (delta_big % delta_small == 0) {
		if (start_x < stop_x && start_y < stop_y) { 	  	/* situation_1 handler */
			if (delta_small == stop_x - start_x) {
				uint8_t x = 0, y = 0;
				for (; y <= delta_big; y++) {
					if (0 == y % line_lenght)
						x++;
					OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
				}
				return OLED_EOK;
			}
			if (delta_small == stop_y - start_y) {
				uint8_t x = 0, y = 0;
				for (; x <= delta_big; x++) {
					if (0 == x % line_lenght)
						y++;
					OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
				}
				return OLED_EOK;
			}
		} else if (start_x < stop_x && start_y > stop_y) { 	/* situation_4 handler */
			if (delta_small == stop_x - start_x) {
				uint8_t x = 0, y = 0;
				for (; y <= delta_big; y++) {
					if (0 == y % line_lenght)
						x++;
					OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
				}
				return OLED_EOK;
			}
			if (delta_small == start_y - stop_y) {
				uint8_t x = 0, y = 0;
				for (; x <= delta_big; x++) {
					if (0 == x % line_lenght)
						y++;
					OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
				}
				return OLED_EOK;
			}
		}
	}
	
	/* Now we work with second case (delta_big % delta_small != 0)
	 * In the second case we will have lines with two different lenghts:
	 * - short_line lenght = delta_big / delta_small
	 * - long_line  lenght = delta_big / delta_small + 1
	 * Number of long_lines can be counted by such formula:
	 * num_long_lines = delta_big % delta_small
	 *
	 * Next we should determine when we must draw short_line and when long_line.
	 * long_line_pos is a line(stair) position on which we should draw short_line.
	 * the condition of drawing short_line is:
	 * if (0 == current_line_position % long_line_pos)
	 * For this stuff I use such formula:
	 * long_line_pos = delta_small / (delta_small - (delta_big % delta_small))
	 */
	uint8_t long_line_pos = delta_small / (delta_small - (delta_big % delta_small));
	
	if (start_x < stop_x && start_y < stop_y) { 	  	/* situation_1 handler */
		if (delta_small == stop_x - start_x) {
			uint8_t x = 0, y = 0;
			for (; y < delta_big; y++) {
				if (0 == y % line_lenght)
					x++;
				if (0 == (x + 1) % long_line_pos) {
					OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
					y++;
				}
				OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
			}
				return OLED_EOK;
		}
		if (delta_small == stop_y - start_y) {
			uint8_t x = 0, y = 0;
			for (; x < delta_big; x++) {
				if (0 == x % line_lenght)
					y++;
				if (0 == (y + 1) % long_line_pos) {
					OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
					x++;
				}
				OLED_put_pixel_(oled, start_x + x, start_y + y, pixel_color);
			}
				return OLED_EOK;
		}
	} else if (start_x < stop_x && start_y > stop_y) { 	/* situation_4 handler */
		if (delta_small == stop_x - start_x) {
			uint8_t x = 0, y = 0;
			for (; y < delta_big; y++) {
				if (0 == y % line_lenght)
					x++;
				if (0 == (x + 1) % long_line_pos) {
					OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
					y++;
				}
				OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
			}
				return OLED_EOK;
		}
		if (delta_small == start_y - stop_y) {
			uint8_t x = 0, y = 0;
			for (; x < delta_big; x++) {
				if (0 == x % line_lenght)
					y++;
				if (0 == (y + 1) % long_line_pos) {
					OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
					x++;
				}
				OLED_put_pixel_(oled, start_x + x, start_y - y, pixel_color);
			}
				return OLED_EOK;
		}
	}
	
}
