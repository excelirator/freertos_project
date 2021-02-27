/*
 * usb_FH.h
 *
 *  Created on: Feb 22, 2021
 *      Author: purush_10
 */

#ifndef INC_USB_FH_H_
#define INC_USB_FH_H_

#include "fatfs.h"
#include "jhd12864e.h"
#include <string.h>
#include <stdio.h>


#define USB_GetFileCount(pat) Scan_USB (file_count, pat)
#define USB_GetFileName(pat) Scan_USB (file_name,pat)

typedef enum {
	file_count = 0,
	file_name = 1
} count_or_name;

/* mounts the USB*/
void Mount_USB (void);

/* unmounts the USB*/
void Unmount_USB (void);


void getfilecount(void);


void getfilename(void);


/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_USB (count_or_name flag, char* pat);

FRESULT Open_File(char *fname);

/* read data from the file
 * @ name : is the path to the file*/
FRESULT Read_File (void *buffer, uint8_t readSize, UINT* br);

FRESULT Close_File(void);


#endif /* INC_USB_FH_H_ */
