/*
 * usb_FH.c
 *
 *  Created on: Feb 22, 2021
 *      Author: purush_10
 */

#include "usb_FH.h"
#include "stm32f4xx_hal.h"

extern char USBHPath[4];   /* USBH logical drive path */
extern FATFS USBHFatFS;    /* File system object for USBH logical drive */
extern FIL USBHFile;       /* File object for USBH */
extern GLCD_t glcd_handle;
extern int filecount;
char path[20];
//char buf[20];

FILINFO USBHfno;
FRESULT fresult;

void Mount_USB (void)
{
	fresult = f_mount(&USBHFatFS, USBHPath, 1);
	GLCD_Clear_All();
	if (fresult != FR_OK)
		GLCD_printLine(2, "USB drive mount error!");
	else
		GLCD_printLine(2, "USB drive mounted!");
}

void Unmount_USB (void)
{
	fresult = f_mount(NULL, USBHPath, 1);
	GLCD_Clear_All();
	if (fresult == FR_OK)
		GLCD_printLine(2, "USB drive unmounted!");
	else
		GLCD_printLine(2, "USB drive unmount error!");
}


void getfilecount(void)
{
	USB_GetFileCount("/");
//	char buf[20];
//	sprintf(buf,"file count is %d", filecount);
	//GLCD_printLine(1,buf);
}

void getfilename(void)
{
	USB_GetFileName("/");
	GLCD_Clear_All();
	for(int cnt=0; cnt<glcd_handle.listCount; cnt++)
	{
		GLCD_printLine(cnt,glcd_handle.display_list[cnt]);
	}
	glcd_handle.cursor_pos = 0;
	GLCD_Cursor_update(curr);
}


FRESULT Scan_USB (count_or_name flag, char* pat)
{
    DIR dir;
//    UINT i;
    strcpy (path, pat);
    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &USBHfno);                   /* Read a directory item */
            if (fresult != FR_OK || USBHfno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (USBHfno.fattrib & AM_DIR)     /* It is a directory */
            {
            	/*
            	if (!(strcmp ("SYSTEM~1", USBHfno.fname))) continue;
            	if (!(strcmp("System Volume Information", USBHfno.fname))) continue;
            	//char *buf = malloc(30*sizeof(char));
            	sprintf (buf, "Dir: %s\r\n", USBHfno.fname);
            	Send_Uart(buf);
            	//free(buf);
                i = strlen(path);
                sprintf(&path[i], "/%s", USBHfno.fname);
                fresult = Scan_USB(path);                     // Enter the directory
                if (fresult != FR_OK) break;
                path[i] = 0;
                */
            }
            else
            {   /* It is a file. */
            	if(flag == file_count)
					filecount++;//filecount
				if(flag == file_name)
				{
					//sprintf(buf,"%s", USBHfno.fname);
					strcpy(glcd_handle.display_list[glcd_handle.listCount], USBHfno.fname);
					glcd_handle.listCount++;
				}
            }
        }
        if(f_closedir(&dir)!=FR_OK)
        	Error_Handler();
    }
    return fresult;
}


FRESULT Open_File(char *fname)
{
	fresult = f_stat (fname, &USBHfno);
	if (fresult != FR_OK)
	{
		GLCD_Clear_All();
		GLCD_printLine(1, "File error!");
		return fresult;
	}

	fresult = f_open(&USBHFile, fname, FA_READ);
	if (fresult != FR_OK)
	{
		GLCD_Clear_All();
		GLCD_printLine(1,"ERROR Opening file!");
	}
	return fresult;
}



FRESULT Read_File (void *buffer, uint8_t readSize, UINT* br)
{
	fresult = f_read (&USBHFile, buffer, readSize, br);
	if (fresult != FR_OK)
	{
		GLCD_Clear_All();
		GLCD_printLine(1,"ERROR Reading file!");
	}
	return fresult;
}


FRESULT Close_File(void)
{
	fresult = f_close(&USBHFile);
	GLCD_Clear_All();
	if (fresult != FR_OK)
		GLCD_printLine(1,"ERROR closing file");
	else
		GLCD_printLine(1,"CLOSED successfully");

	return fresult;
}
