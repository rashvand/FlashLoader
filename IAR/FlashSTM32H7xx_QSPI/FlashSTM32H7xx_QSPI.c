/*************************************************************************
 *
 *   Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2019
 *
 *    File name   : FlashSTM32H7xx_QSPI.c
 *    Description : Flash Loader For Serial NOR flash
 *
 *    History :
 *    1. Date        : July, 2018
 *       Author      : Atanas Uzunov
 *       Description : Initial QSPI flashloader for Micron MT25T/MT25Q
 *
 *    $Revision: 39 $
 **************************************************************************/

#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

/* The flash loader framework API declarations */
#include "flash_loader.h"
#include "flash_loader_extra.h"
   
/** default settings **/
#define QSPI_FLASH_SIZE_MAX        31


#define QUAD_IN_FAST_PROG_CMD      0x32
#define READ_FLAG_STATUS_REG_CMD   0x70
#define SECTOR_ERASE_CMD           0xD8
#define WRITE_ENABLE_CMD           0x06
#define READ_STATUS_REG_CMD        0x05
#define ENTER_4BYTE_ADDR_MODE_CMD  0xB7
#define READ_ID_CMD                0x9F
   
#define QSPI_ERASE_TIMEOUT_VALUE   0x100000
#define QSPI_WRITE_TIMEOUT_VALUE   0x1000
#define QSPI_WREN_TIMEOUT_VALUE    0x10
#define QSPI_TC_TIMEOUT_VALUE      0x10



/** external functions **/

/** external data **/

/** internal functions **/

/** private functions **/
#if USE_ARGC_ARGV
static const char* FlFindOption(char* option, int with_value, int argc, char const* argv[]);
#endif
static uint32_t QSPI_Init(void);
static uint32_t QSPI_WritePage(unsigned long adr, unsigned long sz, unsigned char *buf);
static uint32_t QSPI_EraseSector(uint32_t adr);
static uint32_t QSPI_WriteEnable(void);

/** public data **/
__no_init __IO uint32_t uwTick;

/** private data **/
__no_init IWDG_HandleTypeDef IWDG1Handle;
__no_init QSPI_HandleTypeDef QSPIHandle;
__no_init QSPI_CommandTypeDef sCommand;
__no_init QSPI_AutoPollingTypeDef sConfig;

__no_init uint32_t FlashAddressSize;
__no_init uint32_t FlashPageSize;

static const char csWriteErr[] = "Write failed!";
static const char csEraseErr[] = "Erase failed!";
static const char csInitErr[] = "Init failed!";
static const char csTimeoutErr[] = " Serial flash timeout error.";
static const char csNoFlashErr[] = " Serial flash not found!";

/** public functions **/

/*************************************************************************
 * Function Name: FlashInit
 * Parameters: Flash Base Address
 *
 * Return: RESULT_OK
 *         RESULT_ERROR
 *
 * Description: Init QSPI flash driver.
 *************************************************************************/
#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags,
                   int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags)
#endif  /* USE_ARGC_ARGV */
{
  /* WORKAROUND for BL: Disable interrupts to avoid SPI/DMA interrupts enabled by BL */
  __disable_irq();

  /* Check if hardware IWDG1 is enabled */
  if (0 == (FLASH->OPTSR_CUR & FLASH_OPTSR_IWDG1_SW))
  {
    IWDG1Handle.Instance = IWDG1;
    /* IWDG1 is already initialized by the flashloader macro script */
  }
  else
  {
    IWDG1Handle.Instance = 0;
  }

  /* Init counter used for timeouts */
  uwTick = 0;

  return QSPI_Init();
}

/*************************************************************************
 * Function Name: FlashWrite
 * Parameters: block base address, offset in block, data size, ram buffer
 *             pointer
 * Return: 0
 *
 * Description. Writes data to QSPI flash
 *************************************************************************/
uint32_t FlashWrite(void *block_start,
                    uint32_t offset_into_block,
                    uint32_t count,
                    char const *buffer)
{
uint32_t size = 0;
/* Set destination address */
uint32_t dest = (uint32_t)block_start + offset_into_block;
/* Set source address */
uint8_t * src = (uint8_t*)buffer;
uint32_t result;

  /* Feed the watchdogs */
  if (IWDG1Handle.Instance) HAL_IWDG_Refresh(&IWDG1Handle);

  while(size < count)
  {
    /* Write one page */
    result = QSPI_WritePage(dest, FlashPageSize, src);
    if(result != RESULT_OK)
    {
      return result;
    }

    size += FlashPageSize;
    dest += FlashPageSize;
    src  += FlashPageSize;
  }
  return RESULT_OK;
}

/*************************************************************************
 * Function Name: FlashErase
 * Parameters:  Block Address, Block Size
 *
 * Return: 0
 *
 * Description: Erase block
 *************************************************************************/
uint32_t FlashErase(void *block_start,
                    uint32_t block_size)
{
  /* Feed the watchdogs */
  if (IWDG1Handle.Instance) HAL_IWDG_Refresh(&IWDG1Handle);

  return QSPI_EraseSector((uint32_t)block_start);
}

/** private functions **/
#if USE_ARGC_ARGV
/** private functions **/
static const char* FlFindOption(char* option, int with_value, int argc, char const* argv[])
{
int i;

  for (i = 0; i < argc; i++)
  {
    if (strcmp(option, argv[i]) == 0)
    {
      if (with_value)
      {
        if (i + 1 < argc)
          return argv[i + 1]; // The next argument is the value.
        else
          return 0; // The option was found but there is no value to return.
      }
      else
      {
        return argv[i]; // Return the flag argument itself just to get a non-zero pointer.
      }
    }
  }
  return 0;
}
#endif // USE_ARGC_ARGV

static uint32_t QSPI_Init(void)
{
  uint8_t  RData[6];
  uint16_t Flash1DevId;
  uint8_t  Flash1Size = 0;
  uint32_t FlashBlockSize;

  /* Zero Init structs */
  memset(&QSPIHandle,0,sizeof(QSPIHandle));
  memset(&sCommand,0,sizeof(sCommand));
  memset(&sConfig,0,sizeof(sConfig));

  __QSPI_CLK_ENABLE();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  QSPIHandle.Instance = QUADSPI;
  HAL_QSPI_DeInit(&QSPIHandle) ;

  /* Using default clock source HSI = 64MHz */
  /* ClockPrescaler set to 1, so QSPI clock = 32MHz */
  QSPIHandle.Init.ClockPrescaler     = 1;
  QSPIHandle.Init.FifoThreshold      = 1;
  QSPIHandle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
  QSPIHandle.Init.FlashSize          = QSPI_FLASH_SIZE_MAX;
  QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  QSPIHandle.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  QSPIHandle.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  QSPIHandle.Init.FlashID            = QSPI_FLASH_ID_1;

  if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Reset Memory */

  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Reset memory config, Cmd in 1 line */
  /* Send RESET ENABLE command (0x66) to be able to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2166;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Send RESET command (0x99) to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2199;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Reset memory config, Cmd in 2 lines*/
  /* Send RESET ENABLE command (0x66) to be able to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2266;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Send RESET command (0x99) to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2299;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Reset memory config, Cmd in 4 lines*/
  /* Send RESET ENABLE command (0x66) to be able to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2366;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */

  /* Send RESET command (0x99) to reset the memory registers */
  QSPIHandle.Instance->CCR = 0x2399;
  __DSB();
  while(QSPIHandle.Instance->SR & QSPI_FLAG_BUSY);  /* Wait for busy flag to be cleared */
  
  QSPI_WriteEnable();

  QSPIHandle.State           = HAL_QSPI_STATE_READY;

  /* Read ID */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_ID_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.NbData            = 6;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
    return RESULT_ERROR_WITH_MSG;
  }

  if (HAL_QSPI_Receive(&QSPIHandle, RData, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Analyze FLASH1 memory */
  if(RData[0] == 0xEF){
    Flash1DevId = (uint16_t)RData[2] | (uint16_t)RData[1]<<8;
    
    if(Flash1DevId == 0x4019){
      Flash1Size = 24;
      FlashPageSize = 256;
    }
    else if(Flash1DevId == 0x4018){
      Flash1Size = 23;
      FlashPageSize = 128;
    }
    else if(Flash1DevId == 0x4017){
      Flash1Size = 22;
      FlashPageSize = 64;
    }
    else if(Flash1DevId == 0x4016){
      Flash1Size = 21;
      FlashPageSize = 32;
    }
    else if(Flash1DevId == 0x4015){
      Flash1Size = 20;
      FlashPageSize = 16;
    }
  }
  
  FlashBlockSize = 0x10000;

  if(Flash1Size == 0)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
    strcat(ERROR_MESSAGE_BUFFER, csNoFlashErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Configure QSPI controller with the correct flash size and mode */
  QSPIHandle.Init.FlashSize = Flash1Size;

  QSPIHandle.Init.FifoThreshold = 4;
  if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* If connected memory is larger than 128Mb enable 32-bit address mode */
  if (Flash1Size > 23)
  {
    FlashAddressSize = QSPI_ADDRESS_32_BITS;
    /* Enable 32-bit address mode */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = ENTER_4BYTE_ADDR_MODE_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;

    if (HAL_QSPI_Command(&QSPIHandle, &sCommand, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
    {
      strcpy(ERROR_MESSAGE_BUFFER, csInitErr);
      return RESULT_ERROR_WITH_MSG;
    }
  }
  else
  {
    FlashAddressSize = QSPI_ADDRESS_24_BITS;
  }

  sprintf(LAYOUT_OVERRIDE_BUFFER, "%d 0x%x", (1 << (QSPIHandle.Init.FlashSize + 1)) / FlashBlockSize, FlashBlockSize);

  SET_PAGESIZE_OVERRIDE(FlashPageSize);

  return OVERRIDE_LAYOUT | OVERRIDE_PAGESIZE;
}

static uint32_t QSPI_WritePage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  /* Enable write operations ------------------------------------------- */
  if (QSPI_WriteEnable() != RESULT_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csWriteErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* 4- Writing Sequence ---------------------------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = QUAD_IN_FAST_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = FlashAddressSize;
  sCommand.Address           = adr;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.NbData            = sz;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csWriteErr);
    return RESULT_ERROR_WITH_MSG;
  }

  if (HAL_QSPI_Transmit(&QSPIHandle, buf, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csWriteErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Reconfigure QUADSPI to automatic polling mode to wait for end of program */
  sConfig.Match           = 0x00; //0x80;
  sConfig.Mask            = 0x01; //0x80;
  sConfig.StatusBytesSize = 1;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = 0x05; //READ_FLAG_STATUS_REG_CMD;
  sCommand.AddressMode    = QSPI_ADDRESS_NONE;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&QSPIHandle, &sCommand, &sConfig, QSPI_WRITE_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csWriteErr);
    if (QSPIHandle.ErrorCode == HAL_QSPI_ERROR_TIMEOUT)
    {
      strcat(ERROR_MESSAGE_BUFFER, csTimeoutErr);
    }
    return RESULT_ERROR_WITH_MSG;
  }

  return RESULT_OK;
}

static uint32_t QSPI_EraseSector(uint32_t adr)
{
  /* Enable write operations ------------------------------------------- */
  if (QSPI_WriteEnable() != RESULT_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csEraseErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Erasing Sequence -------------------------------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = FlashAddressSize;
  sCommand.Address           = adr;
  sCommand.DataMode          = QSPI_DATA_NONE;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csEraseErr);
    return RESULT_ERROR_WITH_MSG;
  }

  /* Reconfigure Quadspi to automatic polling mode to wait for end of erase */
  sConfig.Match           = 0x00; //0x80;
  sConfig.Mask            = 0x01; //0x80;
  sConfig.StatusBytesSize = 1;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = 0x05; //READ_FLAG_STATUS_REG_CMD;
  sCommand.AddressMode    = QSPI_ADDRESS_NONE;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&QSPIHandle, &sCommand, &sConfig, QSPI_ERASE_TIMEOUT_VALUE) != HAL_OK)
  {
    strcpy(ERROR_MESSAGE_BUFFER, csEraseErr);
    if (QSPIHandle.ErrorCode == HAL_QSPI_ERROR_TIMEOUT)
    {
      strcat(ERROR_MESSAGE_BUFFER, csTimeoutErr);
    }
    return RESULT_ERROR_WITH_MSG;
  }

  return RESULT_OK;
}

static uint32_t QSPI_WriteEnable(void)
{
  /* Enable write operations ------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, QSPI_TC_TIMEOUT_VALUE) != HAL_OK)
  {
    return RESULT_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.StatusBytesSize = 1;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;
  sCommand.NbData         = 1;

  if (HAL_QSPI_AutoPolling(&QSPIHandle, &sCommand, &sConfig, QSPI_WREN_TIMEOUT_VALUE) != HAL_OK)
  {
    return RESULT_ERROR;
  }

  return RESULT_OK;
}

/* Override default implementation and avoid using interrupts */
uint32_t HAL_GetTick(void)
{
  return uwTick++;
}
