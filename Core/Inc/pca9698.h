/*
 * pca9698.h
 *
 *  Created on: Jan 11, 2021
 *      Author: puru1
 */

#ifndef INC_PCA9698_H_
#define INC_PCA9698_H_

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

// Include the hw-specific stm32fxxx_hal.h
#include "stm32f4xx_hal.h"
#include "main.h"


//uint16_t slave_Adresses_line1[2] = { 0x40, 0x42 };
//uint16_t slave_Adresses_line2[2] = { 0x40, 0x42 };
//uint16_t slave_Adresses_line3[2] = { 0x40, 0x42 };

typedef struct {
	I2C_HandleTypeDef *hi2cx;
	uint16_t *slaveAddress_base;	//fixed, doesn't get modified
	uint16_t *selected_slave;
	uint8_t number_of_slaves;
	uint8_t *data_to_write_base;	//fixed, doesn't get modified
	uint8_t *selected_data_to_write;
	EventBits_t uxBitstoSet;
}PCA9698_t;

/* Commands */
	//---- 5-bank register category ----//
#define PCA9698_CMD_AI_MSK			0x80U 	// bit mask for auto-increment command
#define PCA9698_CMD_IP				0x00U
#define PCA9698_CMD_OP 				0x08U	// command for O/P register access
#define PCA9698_CMD_PI				0x10U	// command for Polarity Inversion register access
#define PCA9698_CMD_IO_CONFIG 		0x18U	// command for I/O Config register access
#define PCA9698_CMD_MI				0x20U	// command for Mask-Interrupt register access
	//---- 1-bank register category ----//
#define PCA9698_CMD_OSC 			0x28U	// command for Output Structure Configuration
#define PCA9698_CMD_ABC 			0x29U	// command for All-Bank Control Configuration
#define PCA9698_CMD_MSC 			0x2AU	// command for Mode Selection Configuration


#define PCA9698_IO_ALL_PINS_OUT		0x00U

/* Mode selection register presets */
#define PCA9698_MSC_OE_ACTIVEHIGH	0x01U
#define PCA9698_MSC_CHANGE_ON_STOP 	0x00U
#define PCA9698_MSC_CHANGE_ON_ACK 	0x02U
#define PCA9698_MSC_ALLCALL_ENABLE	0x08U
#define PCA9698_MSC_SMBA_ENABLE		0x10U

/* Register-bank Addresses */
typedef enum {
	IN_BANK0 = 0x00U,
	IN_BANK1 = 0x01U,
	IN_BANK2 = 0x02U,
	IN_BANK3 = 0x03U,
	IN_BANK4 = 0x04U
} pca9698_input_bank;

typedef enum {
	OUT_BANK0 = 0x08U,
	OUT_BANK1 = 0x09U,
	OUT_BANK2 = 0x0AU,
	OUT_BANK3 = 0x0BU,
	OUT_BANK4 = 0x0CU
} pca9698_output_bank;

typedef enum {
	PI_BANK0 = 0x10,
	PI_BANK1 = 0x11,
	PI_BANK2 = 0x12,
	PI_BANK3 = 0x13,
	PI_BANK4 = 0x14
} pca9698_polInv_bank;

typedef enum {
	CONFIG_BANK0 = 0x18U,
	CONFIG_BANK1 = 0x19U,
	CONFIG_BANK2 = 0x1AU,
	CONFIG_BANK3 = 0x1BU,
	CONFIG_BANK4 = 0x1CU
} pca9698_config_bank;

typedef enum {
	MI_BANK0 = 0x20,
	MI_BANK1 = 0x21,
	MI_BANK2 = 0x22,
	MI_BANK3 = 0x23,
	MI_BANK4 = 0x24
} pca9698_mskInt_bank;

typedef enum {
	I2C_LINE_1 = 0,
	I2C_LINE_2 = 1,
	I2C_LINE_3 = 2
} pca9698_i2cLine;

typedef struct {
	uint8_t Bankx;		// can be any value from pca9698_xxxxx_bank
	uint8_t data;
} PCA9698_RegBank;

typedef struct {
	uint8_t bankData[5];
} PCA9698_data_t;

typedef struct {
	uint8_t cmd;
	PCA9698_data_t fullData;
} PCA9698_MultiRegBank;

extern EventGroupHandle_t i2cTxEvent;

uint8_t PCA9698_Init(I2C_HandleTypeDef *, uint16_t);
uint8_t PCA9698_Config_AllOut(I2C_HandleTypeDef *, uint16_t);
void PCA9698_BankWrite(I2C_HandleTypeDef *, uint16_t, PCA9698_RegBank *);
void PCA9698_MultiBankWrite(I2C_HandleTypeDef *, uint8_t , uint16_t, PCA9698_MultiRegBank *);
HAL_StatusTypeDef PCA9698_LineX_Write_Start(pca9698_i2cLine);
HAL_StatusTypeDef PCA9698_Adv_Init(I2C_HandleTypeDef *, uint16_t *, uint8_t , uint8_t *);

/* C++ detection */
#ifdef __cplusplus
}
#endif



#endif /* INC_PCA9698_H_ */
