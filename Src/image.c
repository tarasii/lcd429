#include "image.h"


//FRESULT img_read(){
//	FATFS FS;
//	FIL fil;
//	FRESULT fres;
//	UINT nWritten;
//	
//	//if (f_mount(&FS, "SD:", 1) == FR_OK) {
//	if (f_mount(&FS, "0", 1) == FR_OK) {
//			
//			fres = f_open(&fil, "img.raw", FA_OPEN_EXISTING | FA_READ );
//			if (fres == FR_OK) {

//					f_read(&fil, (uint8_t *)(LCD_FRAME_BUFFER), LCD_PIXEL_WIDTH*LCD_PIXEL_WIDTH*LCD_PIXEL_SIZE, &nWritten);
//					
//					/* Close file */
//					f_close(&fil);
//					
//			}
//			
//			/* Unmount SDCARD */
//			f_mount(NULL, "SD:", 1);
//	}
//	return fres;
//}

FRESULT img_save(){
	//FATFS FS;
	FIL fil;
	FRESULT fres;
	UINT nWritten;
	UINT toWrite;
	UINT totalWrite;
	UINT totalWriten;
	
	totalWrite = LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * LCD_PIXEL_SIZE;
	//fres = f_mount(&FS, "SD:", 1);
	//fres = f_mount(&FS, "0", 1);
	//if (fres == FR_OK) {

		fres = f_open(&fil, "img2.raw", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (fres == FR_OK) {

			nWritten = 0;
			totalWriten = 0;
			toWrite = totalWrite;
			while (totalWrite > totalWriten){			
				
				f_write(&fil, (uint8_t *)(LCD_FRAME_BUFFER + totalWriten), toWrite, &nWritten);
				toWrite -= nWritten;
				totalWriten += nWritten; 
				nWritten = 0;
			
			}
			f_close(&fil);			
		}
		
		//f_mount(NULL, "SD:", 1);
	//}
	return fres;
}


FRESULT img_bmp_save(){
	FATFS FS;
	FIL fil;
	FRESULT fres;
	UINT nWritten;
	UINT toWrite;
	UINT totalWrite;
	UINT totalWriten;
	
  BITMAPFILEHEADER bfh;
  BITMAPINFOHEADER bih;

	totalWrite = LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * LCD_PIXEL_SIZE;
	
  //memset (&bfh, 0, sizeof(bfh));
	bfh.bfType = 0x4D42;                           
	bfh.bfOffBits = sizeof(bfh) + sizeof(bih);    
	bfh.bfSize = bfh.bfOffBits + totalWrite;    
	//memset (&bih, 0, sizeof(bih));
	bih.biSize = sizeof(bih);                      
	bih.biBitCount = 16;                           
	bih.biClrUsed = 0;                         
	bih.biCompression = 0;                    
	bih.biHeight = LCD_PIXEL_HEIGHT;
	bih.biWidth = LCD_PIXEL_WIDTH;
	bih.biPlanes = 1; 
	
	fres = f_mount(&FS, "0", 1);
	if (fres == FR_OK) {

		fres = f_open(&fil, "img.bmp", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (fres == FR_OK) {


			f_write(&fil, (uint8_t *)(&bfh), sizeof(bfh), &nWritten);
			f_write(&fil, (uint8_t *)(&bih), sizeof(bih), &nWritten);

			nWritten = 0;
			totalWriten = 0;
			toWrite = totalWrite;
			while (totalWrite > totalWriten){			
				
				f_write(&fil, (uint8_t *)(LCD_FRAME_BUFFER + totalWriten), toWrite, &nWritten);
				toWrite -= nWritten;
				totalWriten += nWritten; 
				nWritten = 0;
			
			}
			f_close(&fil);			
		}
		
		f_mount(NULL, "SD:", 1);
	}
	return fres;
}


