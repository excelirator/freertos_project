/*
 * pca9698.c
 *
 *  Created on: Jan 11, 2021
 *      Author: puru1
 */

#include "pca9698.h"
#include "cmsis_os.h"

static PCA9698_t slave_lines[3];

/*
 * Remarks for modifying MODE register:
 *
 * After programming a PCA9698, its state machine will be in a
 * �wait-for-STOP-condition� until a STOP condition is received to
 * update the Output Port registers. Since this state machine will be
 * in a �wait-state�, the part will not respond to its own address
 * until this state machine gets out to the idle condition, which means
 * that the device can be programmed only once and is not addressable again
 * until a STOP condition has been received.
 *
 * Remarks for writing data to the 5-register banks:
 *
 * The PCA9698 has one level of buffers to store 5 bytes of data, and the
 * actual Output Port registers will get updated on the STOP condition.
 * If the master sends more than 5 bytes of data (with AI = 1), the data
 * in the buffer will get overwritten.
 *
 */

/**
  * @brief  Checks if target PCA9698 is ready for communication and configures it.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  SlaveAddress : The 7-bits address value of target PCA9698
  *         in datasheet must be shifted to the left before calling the interface
  * @retval non-zero value signifies failure; zero signifies success
  */
uint8_t PCA9698_Init(I2C_HandleTypeDef *hi2c, uint16_t SlaveAddress)
{
	if(HAL_I2C_IsDeviceReady(hi2c, SlaveAddress, 1, 1000) != HAL_OK) {
		return 1;
	}

	//PCA9698 is configured to change state of all pins on STOP from the i2c master
	uint8_t initData[2] = { PCA9698_CMD_MSC, PCA9698_MSC_CHANGE_ON_STOP };
	HAL_I2C_Master_Transmit(hi2c, SlaveAddress, initData, 2, 10);

	return 0;	//Initialized successfully
}


/**
  * @brief  Configures target PCA9698 to act only as an output expander.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  SlaveAddress : The 7-bits address value of target PCA9698
  *         in datasheet must be shifted to the left before calling the interface
  * @retval non-zero value signifies failure; zero signifies success
  */
uint8_t PCA9698_Config_AllOut(I2C_HandleTypeDef *hi2c, uint16_t SlaveAddress)
{
	uint8_t initData[6] = { PCA9698_CMD_AI_MSK | PCA9698_CMD_IO_CONFIG };

	if(HAL_I2C_Master_Transmit(hi2c, SlaveAddress, initData, 6, 100) != HAL_OK)
		return 1;
	else
		return 0;
}


/**
  * @brief  Writes a byte of data to the o/p register bank of target PCA9698
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  SlaveAddress : The 7-bits address value of target PCA9698
  *         in datasheet must be shifted to the left before calling the interface
  * @param	regBank : Pointer to a PCA9698_RegBank instance
  */
void PCA9698_BankWrite(I2C_HandleTypeDef *hi2c,
		uint16_t SlaveAddress, PCA9698_RegBank *regBank)
{
	HAL_I2C_Master_Transmit(hi2c, SlaveAddress, (uint8_t*)regBank, 2, 10);
}


/**
  * @brief  Writes data to the o/p register bank of target PCA9698
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  SlaveAddress : The 7-bits address value of target PCA9698
  *         in datasheet must be shifted to the left before calling the interface
  * @param	size : number of banks to be written to
  * @param	regBank : Pointer to a PCA9698_MultiRegBank instance
  */
void PCA9698_MultiBankWrite(I2C_HandleTypeDef *hi2c, uint8_t size,
		uint16_t SlaveAddress, PCA9698_MultiRegBank *regBank)
{
	regBank->cmd |= PCA9698_CMD_AI_MSK;	//enable auto increment for the command
	HAL_I2C_Master_Transmit(hi2c, SlaveAddress, (uint8_t*)regBank, size, 10);
}


void PCA9698_live_scanner(I2C_HandleTypeDef *hi2c, uint16_t *addresses, uint8_t *scanStop_flag)
{

	while(1)
	{
		if(*scanStop_flag != 0)
			break;

		//implement a function to get the i2c addresses connected to
		//an i2c line
	}
}


///////////////////////////////////////////////////////
//
//		Advanced functions for minimal latency
//	/* APIs for several devices on single I2C line */
//
//
//////////////////////////////////////////////////////

/**
  * @brief  Initializes all the PCA9698 slaves in an I2C line, and also an
  * 		internal data structure with the necessary configuration.
  *
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  SlaveAddress_base : Pointer to the first slave address in the i2c line.
  *         Address in datasheet must be shifted to the left before calling the interface
  * @param	slaveCount : number of slave devices to be initialised
  * @param	data_base : pointer to the first data element
  * @retval HAL status
  */
HAL_StatusTypeDef PCA9698_Adv_Init(I2C_HandleTypeDef *hi2c,
		uint16_t *SlaveAddress_base, uint8_t slaveCount, uint8_t *data_base)
{
	PCA9698_t *line_selected;
	if(hi2c->Instance==I2C1)
		line_selected = &slave_lines[0];
	if(hi2c->Instance==I2C2)
		line_selected = &slave_lines[1];
	if(hi2c->Instance==I2C3)
		line_selected = &slave_lines[2];

	// Initializing structure variables
	line_selected->hi2cx = hi2c;
	line_selected->slaveAddress_base = SlaveAddress_base;
	line_selected->selected_slave = SlaveAddress_base;
	line_selected->data_to_write_base = data_base;
	line_selected->selected_data_to_write = data_base;
	line_selected->number_of_slaves = slaveCount;

	uint8_t initData[8] = { PCA9698_CMD_MSC, PCA9698_MSC_CHANGE_ON_STOP, PCA9698_CMD_AI_MSK | PCA9698_CMD_IO_CONFIG };
	while(slaveCount > 0)
	{
		//PCA9698 is configured to change state of all pins on STOP
		//from the i2c master
		if(HAL_I2C_Master_Transmit(hi2c, *SlaveAddress_base, initData, 2, 100) != HAL_OK) {
			return HAL_ERROR;
		}

		//PCA9698 target configured to act only as an output expander
		if(HAL_I2C_Master_Transmit(hi2c, *SlaveAddress_base, &initData[2], 6, 100) != HAL_OK) {
			return HAL_ERROR;
		}

		slaveCount--;
		SlaveAddress_base++;
	}

	return HAL_OK;	//Initialized successfully
}

/**
  * @brief  Begin transmission on the specified I2C line
  * @param	lineX : The i2c line over which communication should be started
  * @retval HAL status
  */
HAL_StatusTypeDef PCA9698_LineX_Write_Start(pca9698_i2cLine lineX)
{
	PCA9698_t *line_selected;
	//point to the appropriate slave_line
	switch (lineX) {
		case I2C_LINE_1 : { line_selected = &slave_lines[0]; break; }
		case I2C_LINE_2 : { line_selected = &slave_lines[1]; break; }
		case I2C_LINE_3 : { line_selected = &slave_lines[2];break; }
	}

	if(HAL_I2C_Master_Transmit_IT(line_selected->hi2cx, line_selected->slaveAddress_base[0], \
			line_selected->data_to_write_base, 6) != HAL_OK)
		return HAL_ERROR;

	else
		return HAL_OK;
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	PCA9698_t *active_line;
	//point to the appropriate slave_line
	if(hi2c->Instance==I2C1)
		active_line = &slave_lines[0];
	if(hi2c->Instance==I2C2)
		active_line = &slave_lines[1];
	if(hi2c->Instance==I2C3)
		active_line = &slave_lines[2];

	if(active_line->selected_slave - active_line->slaveAddress_base ==	\
			(active_line->number_of_slaves-1))
	{
	  /* Control reaches when all the slaves have been communicated with */
	  active_line->selected_slave = active_line->slaveAddress_base;	//reset the pointer to point to first slave
	  active_line->selected_data_to_write = active_line->data_to_write_base; // reset pointer to point to first data element
	  /* last slave of last line is communicated with, notify i2c task to resume */
		if(hi2c->Instance==I2C1)
			active_line->uxBitstoSet = 0x01;
		if(hi2c->Instance==I2C2)
			active_line->uxBitstoSet = 0x02;
		if(hi2c->Instance==I2C3)
			active_line->uxBitstoSet = 0x04;
		// --- the above logic can be optimized --- //

	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xEventGroupSetBitsFromISR(i2cTxEvent, active_line->uxBitstoSet, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
	else
	{
	  /* communicate with next slave device on the I2C line */
	  active_line->selected_slave++;	//point to the next slave in the line
	  active_line->selected_data_to_write += 6; // modify data pointer for the next slave device
	  HAL_I2C_Master_Transmit_IT(hi2c, *(active_line->selected_slave),	\
				active_line->selected_data_to_write, 6);
	}
}


/* Generate Stop */
//    SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);

//HAL_StatusTypeDef PCA9698_LineX_Write_Start(pca9698_i2cLine lineX)
//{
//	PCA9698_t *line_selected;
//	//point to the appropriate slave_line
//	switch (lineX) {
//		case I2C_LINE_1 : { line_selected = &slave_lines[0]; break; }
//		case I2C_LINE_2 : { line_selected = &slave_lines[1]; break; }
//		case I2C_LINE_3 : { line_selected = &slave_lines[2]; break; }
//	}
//
//
//	if(HAL_I2C_Master_Seq_Transmit_IT(line_selected->hi2cx, line_selected->slaveAddress_base[0], \
//			line_selected->data_to_write_base, 6, I2C_FIRST_AND_NEXT_FRAME) != HAL_OK)
//	{
//		return HAL_ERROR;
//	}
//	else
//	{
//		return HAL_OK;
//	}
//}


//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	PCA9698_t *line_selected;
//	if(hi2c->Instance==I2C1)
//		line_selected = &slave_lines[0];
//	if(hi2c->Instance==I2C2)
//			line_selected = &slave_lines[1];
//	if(hi2c->Instance==I2C3)
//		line_selected = &slave_lines[2];
//
//	uint32_t XferOptions;
//
//	if(line_selected->selected_slave - line_selected->slaveAddress_base ==	\
//			(line_selected->number_of_slaves-1))
//	{
//		/* all devices on the line have been communicated with */
//
//		line_selected->selected_slave = line_selected->slaveAddress_base;	//point to the first slave in the line
//		line_selected->selected_data_to_write = line_selected->data_to_write_base; // modify data pointer for the next slave device
//	}
//	else
//	{
//		if(line_selected->selected_slave - line_selected->slaveAddress_base ==	\
//			(line_selected->number_of_slaves-2))
//				XferOptions = I2C_OTHER_AND_LAST_FRAME;
//		else
//			XferOptions = I2C_OTHER_FRAME;
//		line_selected->selected_slave++;	//point to the next slave in the line
//		line_selected->selected_data_to_write += 6; // modify data pointer for the next slave device
//
//		/* communicate with next slave device on the I2C line */
//		HAL_I2C_Master_Seq_Transmit_IT(hi2c, *(line_selected->selected_slave),	\
//				line_selected->selected_data_to_write, 6, XferOptions);
//	}
//
//}


