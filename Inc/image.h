#ifndef __image_H
#define __image_H

#include "stm32f4xx_hal.h"
#include "ff.h"
#include "lcd.h"

extern FATFS FS;

typedef struct  
{ 
	 WORD    bfType; 
	 DWORD   bfSize; 
	 WORD    bfReserved1; 
	 WORD    bfReserved2; 
	 DWORD   bfOffBits; 
} BITMAPFILEHEADER; //, *PBITMAPFILEHEADER;

typedef struct 
{
	 DWORD  biSize; 
	 LONG   biWidth; 
	 LONG   biHeight; 
	 WORD   biPlanes; 
	 WORD   biBitCount; 
	 DWORD  biCompression; 
	 DWORD  biSizeImage; 
	 LONG   biXPelsPerMeter; 
	 LONG   biYPelsPerMeter; 
	 DWORD  biClrUsed; 
	 DWORD  biClrImportant; 
} BITMAPINFOHEADER; //, *PBITMAPINFOHEADER;
 
//FRESULT img_read(void);
FRESULT img_save(void);
//FRESULT img_bmp_save(void);

#endif /*__ image_H */

