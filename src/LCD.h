#ifndef __LCD_H__
#define __LCD_H__

#include <stdio.h>

#define LCD_DEBUG

void LCD_init() ;
void LCD_drawBitmap(int x_start, int y_start, int x_end, int y_end, const void* color_data) ;

#endif
