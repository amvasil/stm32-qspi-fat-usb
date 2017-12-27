//fat_qspi_drv.c

#include "fat_qspi_drv.h"

#include "ff_gen_drv.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS QSPI_initialize (BYTE);
DSTATUS QSPI_status (BYTE);
DRESULT QSPI_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT QSPI_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT QSPI_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  QSPI_Driver =
{
    QSPI_initialize,
    QSPI_status,
    QSPI_read, 
#if  _USE_WRITE == 1
    QSPI_write,
#endif /* _USE_WRITE == 1 */
    
#if  _USE_IOCTL == 1
    QSPI_ioctl,
#endif /* _USE_IOCTL == 1 */
};



/**
* @brief  Initializes a Drive
* @param  lun : not used 
* @retval DSTATUS: Operation status
*/
DSTATUS QSPI_initialize(BYTE lun)
{
    Stat = STA_NOINIT;
    
    /* Configure the uSD device */
    if(  QSPI_Driver_init() == QSPI_STATUS_OK)
    {
        Stat &= ~STA_NOINIT;
    }
    
    return Stat;
}

/**
* @brief  Gets Disk Status
* @param  lun : not used
* @retval DSTATUS: Operation status
*/
DSTATUS QSPI_status(BYTE lun)
{
    Stat = STA_NOINIT;
    
    if(QSPI_Driver_state())
    {
        Stat &= ~STA_NOINIT;
    }
    
    return Stat;
}

/**
* @brief  Reads Sector(s)
* @param  lun : not used
* @param  *buff: Data buffer to store read data
* @param  sector: Sector address (LBA)
* @param  count: Number of sectors to read (1..128)
* @retval DRESULT: Operation result
*/
DRESULT QSPI_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    uint32_t bufferSize = (BLOCK_SIZE * count); 
    uint32_t address =  (sector * BLOCK_SIZE); 
    uint32_t data_read = 0;
    //printf("read %d blocks, sector %d\n",count,sector);
    while(data_read < bufferSize)
    {
        uint32_t incr = bufferSize < MAX_READ_SIZE ? bufferSize : MAX_READ_SIZE;
        if(QSPI_Driver_read(&buff[data_read],address,incr) != QSPI_STATUS_OK)
        {
            return RES_ERROR;
        }
        data_read += incr;
        address += incr;
    }
    
    return RES_OK;
}

/**
* @brief  Writes Sector(s)
* @param  lun : not used
* @param  *buff: Data to be written
* @param  sector: Sector address (LBA)
* @param  count: Number of sectors to write (1..128)
* @retval DRESULT: Operation result
*/
#if _USE_WRITE == 1
DRESULT QSPI_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
//    uint32_t timeout = 100000;
//
//    __disable_irq();
//
//    if(BSP_SD_WriteBlocks((uint32_t*)buff,
//                          (uint32_t)(sector),
//                          count, SD_DATATIMEOUT) == MSD_OK)
//    {
//        while(BSP_SD_GetCardState()!= MSD_OK)
//        {
//            if (timeout-- == 0)
//            {
//                return RES_ERROR;
//            }
//        }
//        res = RES_OK;
//    }
//
//    __enable_irq();
    
    return res;
}
#endif /* _USE_WRITE == 1 */

/**
* @brief  I/O control operation
* @param  lun : not used
* @param  cmd: Control code
* @param  *buff: Buffer to send/receive control data
* @retval DRESULT: Operation result
*/
#if _USE_IOCTL == 1
DRESULT QSPI_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
        /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

        /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        *(DWORD*)buff = QSPI_SUBSECTOR_COUNT;
        res = RES_OK;
        break;

        /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        *(WORD*)buff = QSPI_SUBSECTOR_SIZE;
        res = RES_OK;
        break;

        /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        *(DWORD*)buff = QSPI_SUBSECTOR_SIZE;
        break;

    default:
        res = RES_PARERR;
    }
    
    return res;
}
#endif /* _USE_IOCTL == 1 */
