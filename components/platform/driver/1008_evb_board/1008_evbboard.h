/**
 * Copyright  2022 by MooreSilicon.All rights reserved
 * @file  evb borad.h
 * @brief
 * @author pengyu.xue
 * @date 2022-2-22
 * @version 1.0
 * @Revision
 */



void board_keypad_gpio_init(void);
uint32_t  board_get_keyValue(uint8_t column, uint8_t row);
uint32_t board_get_combinedkey(uint32_t firstkeyvalue, uint32_t secondkeyvalue);




