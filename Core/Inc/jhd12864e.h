/*
 * jhd12864e.h
 *
 *  Created on: Feb 9, 2021
 *      Author: praveen
 *			Purpose : Glcd 128X64 display driver
 */
#ifndef _JHD12864E_H_
#define _JHD12864E_H_

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define GLCD_CTRL GPIOB
#define GLCD_DATA GPIOD
#define GLCD_RS GPIO_PIN_9
#define GLCD_EN GPIO_PIN_7
#define GLCD_RW GPIO_PIN_8
#define GLCD_RST GPIO_PIN_4
#define GLCD_CS1 GPIO_PIN_6
#define GLCD_CS2 GPIO_PIN_5

#define GLCD_DB7 GPIO_PIN_7
#define GLCD_DB6 GPIO_PIN_6
#define GLCD_DB5 GPIO_PIN_5
#define GLCD_DB4 GPIO_PIN_4
#define GLCD_DB3 GPIO_PIN_3
#define GLCD_DB2 GPIO_PIN_2
#define GLCD_DB1 GPIO_PIN_1
#define GLCD_DB0 GPIO_PIN_0

#define rst_high()   (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RST,GPIO_PIN_SET))
#define rst_low()    (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RST,GPIO_PIN_RESET))
#define cs1_high()   (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_CS1,GPIO_PIN_SET))
#define cs1_low()    (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_CS1,GPIO_PIN_RESET))
#define cs2_high()   (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_CS2,GPIO_PIN_SET))
#define cs2_low()    (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_CS2,GPIO_PIN_RESET))
#define rs_high()    (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RS,GPIO_PIN_SET))
#define rs_low()     (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RS,GPIO_PIN_RESET))
#define en_high()    (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_EN,GPIO_PIN_SET))
#define en_low()     (HAL_GPIO_WritePin(GLCD_CTRL,GLCD_EN,GPIO_PIN_RESET))
#define cs1_toggle() (HAL_GPIO_TogglePin(GLCD_CTRL,GLCD_CS1))
#define cs2_toggle() (HAL_GPIO_TogglePin(GLCD_CTRL,GLCD_CS2))
#define port_data(value) (GLCD_DATA->ODR=value)

#define rw_high()	(HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RW,GPIO_PIN_SET))
#define rw_low()	(HAL_GPIO_WritePin(GLCD_CTRL,GLCD_RW,GPIO_PIN_RESET))

#define GLCD_CMD_OFF					0x3E		//turn off the lcd
#define GLCD_CMD_ON						0x3F		//turn on the lcd
#define GLCD_CMD_PAGE0				0xB8		//x-address
#define GLCD_CMD_COL0					0x40		//y-address
#define GLCD_CMD_STRT_LINE		0xC0		//z-address

#define GLCD_SendCommand(cmd) GLCD_writeDataLine(cmd_mode, cmd)
#define GLCD_SendData(data) 	GLCD_writeDataLine(data_mode, data)

typedef enum {
	left_segment = 0,
	right_segment = 1,
	both_segment =2
} chip_select;

typedef enum {
	cmd_mode = 0,
	data_mode = 1
} cmd_or_data;

typedef enum {
	curr = 0,
	decr = 1,
	incr = 2,
	clr = -1
}cursor_modify;

typedef struct {
	uint8_t current_page;		//x-addr (between 0-7)
	uint8_t current_column;		//y-addr (between 0-63)
	uint8_t start_line;			//z-addr (between 0-63)
	uint8_t cursor_pos;
	uint8_t cursor_max;
	uint8_t cursor_min;
	chip_select current_segment;
	char *text_buf;
	char display_list[8][25];	//list to store file names
	uint8_t listCount;
}GLCD_t;

void GLCD_Init(void);
void GLCD_Clear_All(void);
void GLCD_printLine(uint8_t page_num, char* str);
void GLCD_Cursor_update(cursor_modify);
//Note : during the porting to some other controller we need to do changes here itself in glcd.c we use microsec delay using cmsis
#ifdef __cplusplus
}
#endif

#endif 
