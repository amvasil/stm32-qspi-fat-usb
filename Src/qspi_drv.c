//qspi_drv.c

#include "qspi_drv.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include <string.h>
#include <assert.h>

static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi);
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static QSPI_STATUS QSPI_Driver_write_chunk(QSPI_HandleTypeDef *, uint8_t *, uint32_t , uint32_t );

extern QSPI_HandleTypeDef hqspi;

volatile uint8_t rx_complete = 0;
volatile uint8_t tx_complete = 0;
volatile uint8_t status_matched = 0;
volatile uint8_t command_complete = 0;

volatile int8_t qspi_locked = 0;

uint32_t dbg_last_read_address = 0;
uint32_t dbg_last_read_size = 0;
uint32_t dbg_succ_reads = 0;

uint8_t opening_root_dir = 0;

uint8_t qspi_init_state = 0;

QSPI_CommandTypeDef sCommand;


/**
 * @brief  This function configure the dummy cycles on memory side.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef sCommand;
	uint8_t reg;

	/* Read Volatile Configuration register --------------------------- */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_1_LINE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData            = 1;

	if (HAL_QSPI_Command(hqspi, &sCommand, QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		Error_Handler();
	}

	rx_complete = 0;
	if (HAL_QSPI_Receive_IT(hqspi, &reg) != HAL_OK)
	{
		Error_Handler();
	}
	while(!rx_complete);

	/* Enable write operations ---------------------------------------- */
	QSPI_WriteEnable(hqspi);

	/* Write Volatile Configuration register (with new dummy cycles) -- */
	sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
	MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ_QUAD << POSITION_VAL(0xF0)));

	if (HAL_QSPI_Command(hqspi, &sCommand, QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		Error_Handler();
	}

	tx_complete = 0;
	if (HAL_QSPI_Transmit_IT(hqspi, &reg) != HAL_OK)
	{
		Error_Handler();
	}
	while(!tx_complete);
}

/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef     sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	/* Enable write operations ------------------------------------------ */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = WRITE_ENABLE_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	command_complete = 0;
	if (HAL_QSPI_Command_IT(hqspi, &sCommand) != HAL_OK)
	{
		Error_Handler();
	}
	while(!command_complete);

	/* Configure automatic polling mode to wait for write enabling ---- */
	sConfig.Match           = 0x02;
	sConfig.Mask            = 0x02;
	sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval        = 0x10;
	sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	sCommand.Instruction    = READ_STATUS_REG_CMD;
	sCommand.DataMode       = QSPI_DATA_1_LINE;

	status_matched = 0;
	if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	while(!status_matched);
}

QSPI_STATUS QSPI_Driver_init()
{
	qspi_locked++;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);


	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
	sCommand.DataMode    = QSPI_DATA_4_LINES;
	sCommand.NbData      = MAX_READ_SIZE;


	/* Configure Volatile Configuration register (with new dummy cycles) */
	QSPI_DummyCyclesCfg(&hqspi);
	const char *str_fat = "mkfs.fat";

	/* Reading Sequence ------------------------------------------------ */
	uint32_t address = 0;
	sCommand.Address     = address;
	sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;

	if (HAL_QSPI_Command(&hqspi, &sCommand, QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		qspi_locked--;
		return QSPI_STATUS_ERROR;
	}

	uint8_t rx_buf[MAX_READ_SIZE];
	memset(rx_buf,0,MAX_READ_SIZE);
	rx_complete = 0;
	if (HAL_QSPI_Receive_DMA(&hqspi, rx_buf) != HAL_OK)
	{
		qspi_locked--;
		return QSPI_STATUS_ERROR;
	}
	while(!rx_complete);
	rx_buf[MAX_READ_SIZE-1] = 0;
	if(strncmp((char*)&rx_buf[3], str_fat, strlen(str_fat)))
	{
		qspi_locked--;
		return QSPI_STATUS_ERROR;
	}

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);

	qspi_init_state = 1;
	qspi_locked--;
	return QSPI_STATUS_OK;

}

QSPI_STATUS QSPI_Driver_read(uint8_t* buff, uint32_t address, uint32_t size)
{
	qspi_locked++;
	assert(size <= MAX_READ_SIZE);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
	/* Reading Sequence ------------------------------------------------ */
	sCommand.NbData      = size;
	sCommand.Address     = address;
	sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
	sCommand.DataMode    = QSPI_DATA_4_LINES;

	dbg_last_read_address = address;
	dbg_last_read_size = size;

	if (HAL_QSPI_Command(&hqspi, &sCommand, QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		qspi_locked--;
		return QSPI_STATUS_ERROR;
	}

	rx_complete = 0;
	if (HAL_QSPI_Receive_DMA(&hqspi, buff) != HAL_OK)
	{
		qspi_locked--;
		return QSPI_STATUS_ERROR;
	}
	while(!rx_complete);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	qspi_locked--;
	return QSPI_STATUS_OK;
}

uint8_t QSPI_Driver_state()
{
	return qspi_init_state;
}

uint8_t check_qspi_avaliable()
{    
	qspi_locked++;
	/* Reading Sequence ------------------------------------------------ */
	sCommand.NbData      = MAX_READ_SIZE;
	sCommand.Address     = 0;
	sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
	sCommand.DataMode    = QSPI_DATA_4_LINES;

	if (HAL_QSPI_Command(&hqspi, &sCommand,QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		qspi_locked--;
		return 0;
	}

	char buff[MAX_READ_SIZE];
	rx_complete = 0;
	if (HAL_QSPI_Receive_DMA(&hqspi, (uint8_t*)buff) != HAL_OK)
	{
		qspi_locked--;
		return 0;
	}
	while(!rx_complete);
	const char *str = "mkfs.fat";
	qspi_locked--;
	if(strncmp(str, &buff[3], strlen(str)))
	{
		return 0;
	}
	else return 1;
}



uint8_t QSPI_Driver_locked()
{
	return qspi_locked;
}


/**
 * @brief  This function read the SR of the memory and wait the EOP.
 * @param  hqspi: QSPI handle
 * @retval None
 */
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef     sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	/* Configure automatic polling mode to wait for memory ready ------ */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = READ_STATUS_REG_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_1_LINE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	sConfig.Match           = 0x00;
	sConfig.Mask            = 0x01;
	sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval        = 0x10;
	sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	status_matched = 0;
	if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	while(!status_matched);
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	rx_complete = 1;
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	tx_complete = 1;
}

/**
 * @brief  Status Match callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
	status_matched = 1;
}

/**
  * @brief  Command completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	command_complete = 1;
}

/**
 * @brief  Timeout callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_TimeOutCallback(QSPI_HandleTypeDef *hqspi)
{
	Error_Handler();
}

/**
 * @brief  Transfer Error callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
	Error_Handler();
}



QSPI_STATUS QSPI_Driver_eraze_subsector(uint32_t address)
{
	qspi_locked++;
	/* Enable write operations ------------------------------------------- */
	QSPI_WriteEnable(&hqspi);
	/* Erasing Sequence -------------------------------------------------- */
	sCommand.Instruction = SUBSECTOR_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.Address     = address;
	sCommand.DataMode    = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	command_complete = 0;
	if (HAL_QSPI_Command_IT(&hqspi, &sCommand) != HAL_OK)
	{
		Error_Handler();
	}
	while(!command_complete);


	QSPI_AutoPollingMemReady(&hqspi);

	qspi_locked--;
	return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_eraze(uint32_t address, uint32_t size)
{

	uint32_t start_address = address;
	uint32_t erased_size = 0;
	while(erased_size < size)
	{
		if(QSPI_Driver_eraze_subsector(start_address) != QSPI_STATUS_OK){
			return QSPI_STATUS_ERROR;
		}
		erased_size += QSPI_SUBSECTOR_SIZE;
		start_address += QSPI_SUBSECTOR_SIZE;
	}

	return QSPI_STATUS_OK;
}

static QSPI_STATUS QSPI_Driver_write_chunk(QSPI_HandleTypeDef *QSPI_handle, uint8_t *buf, uint32_t address, uint32_t size)
{
	assert(size <= MAX_READ_SIZE);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	qspi_locked++;
	/* Enable write operations ----------------------------------------- */
	QSPI_WriteEnable(&hqspi);

	/* Writing Sequence ------------------------------------------------ */
	sCommand.Instruction = EXT_QUAD_IN_FAST_PROG_CMD;
	sCommand.Address     = address;
	sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
	sCommand.DataMode    = QSPI_DATA_4_LINES;
	sCommand.NbData      = size;

	if (HAL_QSPI_Command(QSPI_handle, &sCommand, QSPI_COMMAND_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		Error_Handler();
	}

	tx_complete = 0;
	if (HAL_QSPI_Transmit_DMA(QSPI_handle, buf) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		Error_Handler();
	}
	while(!tx_complete);

	QSPI_AutoPollingMemReady(&hqspi);

	qspi_locked--;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	return QSPI_STATUS_OK;
}

QSPI_STATUS QSPI_Driver_write(uint8_t *buf, uint32_t address, uint32_t size)
{
	uint32_t start_address = address;
	uint32_t written_size = 0;
	while(written_size < size)
	{
		uint32_t left_size = size - written_size;
		uint32_t to_write_size = (left_size > MAX_READ_SIZE) ? MAX_READ_SIZE : left_size;
		if(QSPI_Driver_write_chunk(&hqspi, &buf[written_size],start_address, to_write_size) != QSPI_STATUS_OK)
		{
			return QSPI_STATUS_ERROR;
		}
		written_size += to_write_size;
		start_address += to_write_size;
	}

	return QSPI_STATUS_OK;
}


