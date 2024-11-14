/*
 ## Cypress USB 3.0 Platform source file (cyfxslfifosync.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2018,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the Slave FIFO Synchronous mode example */

/*
   This example comprises of two USB bulk endpoints. A bulk OUT endpoint acts as the
   producer of data from the host. A bulk IN endpoint acts as the consumer of data to
   the host. Appropriate vendor class USB enumeration descriptors with these two bulk
   endpoints are implemented.

   The GPIF configuration data for the Synchronous Slave FIFO operation is loaded onto
   the appropriate GPIF registers. The p-port data transfers are done via the producer
   p-port socket and the consumer p-port socket.

   This example implements two DMA Channels in MANUAL mode one for P to U data transfer
   and one for U to P data transfer.

   The U to P DMA channel connects the USB producer (OUT) endpoint to the consumer p-port
   socket. And the P to U DMA channel connects the producer p-port socket to the USB 
   consumer (IN) endpoint.

   Upon every reception of data in the DMA buffer from the host or from the p-port, the
   CPU is signalled using DMA callbacks. There are two DMA callback functions implemented
   each for U to P and P to U data paths. The CPU then commits the DMA buffer received so
   that the data is transferred to the consumer.

   The DMA buffer size for each channel is defined based on the USB speed. 64 for full
   speed, 512 for high speed and 1024 for super speed. CY_FX_SLFIFO_DMA_BUF_COUNT in the
   header file defines the number of DMA buffers per channel.

   The constant CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT in the header file is used to
   select 16bit or 32bit GPIF data bus configuration.
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3i2c.h"
#include "cyu3spi.h"
#include "cyu3uart.h"
#include "cyfxslfifosync.h"
#include "cyfxflashprog.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3utils.h"
#include "pib_regs.h"
#include <uart_regs.h>
#include <cyu3gpio.h>

/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
#include "cyfxgpif2config.h"

CyU3PThread       USBUARTAppThread;
CyU3PDmaChannel   glChHandleUsbtoUart;          /* DMA AUTO (USB TO UART) channel handle.*/
CyU3PDmaChannel   glChHandleUarttoUsb;          /* DMA AUTO_SIG(UART TO USB) channel handle.*/

CyU3PUartConfig_t glUartConfig = {0};           /* Current UART configuration. */
volatile uint16_t glPktsPending = 0;            /* Number of packets that have been committed since last check. */

/* CDC Class specific requests to be handled by this application. */
#define SET_LINE_CODING        0x20
#define GET_LINE_CODING        0x21
#define SET_CONTROL_LINE_STATE 0x22


CyBool_t          glIsApplnActive = CyFalse;
CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;   /* DMA Channel handle for P2U transfer. */

uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */

/* Firmware ID variable that may be used to verify flash programmer firmware. */
const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = { 'F', 'X', '3', 'P', 'R', 'O', 'G', '\0' };

uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32)));

uint16_t glI2cPageSize = 0x40;   /* I2C Page size to be used for transfers. */
uint16_t glSpiPageSize = 0x100;  /* SPI Page size to be used for transfers. */

CyU3PDmaChannel glI2cTxHandle;   /* I2C Tx channel handle */
CyU3PDmaChannel glI2cRxHandle;   /* I2C Rx channel handle */
CyU3PDmaChannel glSpiTxHandle;   /* SPI Tx channel handle */
CyU3PDmaChannel glSpiRxHandle;   /* SPI Rx channel handle */

void CyFxSlFifoApplnInit(void);

/* I2c initialization for EEPROM programming. */
CyU3PReturnStatus_t
CyFxFlashProgI2cInit (uint16_t pageLen)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = 100000;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyTrue;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Now create the DMA channels required for read and write. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this will be used
     * only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Create a channel to write to the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_I2C_CONS;
    status = CyU3PDmaChannelCreate (&glI2cTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create a channel to read from the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_I2C_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glI2cRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glI2cPageSize = pageLen;
    }

    return status;
}

/* I2C read / write for programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgI2cTransfer (
        uint16_t  byteAddress,
        uint8_t   devAddr,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
    }

    CyU3PDebugPrint (2, "I2C access - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            devAddr, byteAddress, byteCount, pageCount);

    /* Update the buffer address and status. */
    buf_p.buffer = buffer;
    buf_p.status = 0;

    while (pageCount != 0)
    {
        if (isRead)
        {
            /* Update the preamble information. */
            preamble.length    = 4;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelSetupRecvBuffer (&glI2cRxHandle, &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cRxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }
        else /* Write */
        {
            /* Update the preamble information. */
            preamble.length    = 3;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.ctrlMask  = 0x0000;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PDmaChannelSetupSendBuffer (&glI2cTxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cTxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }

        /* Update the parameters */
        byteAddress  += glI2cPageSize;
        buf_p.buffer += glI2cPageSize;
        pageCount --;

        /* Need a delay between write operations. */
        CyU3PThreadSleep (10);
    }

    return CY_U3P_SUCCESS;
}

/* SPI initialization for flash programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgSpiInit (uint16_t pageLen)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSpiConfig_t spiConfig;
    CyU3PDmaChannelConfig_t dmaConfig;

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the SPI master block. Run the SPI clock at 8MHz
     * and configure the word length to 8 bits. Also configure
     * the slave select using FW.
     *
     * Note: This application uses the DMA mode of transfer to read/write data from/to
     * the SPI flash device. The DMA transfer mode does not work correctly when the clock
     * frequency is lower than 4 MHz. Please change the CyFxFlashProgSpiTransfer() function
     * to use the register mode (CyU3PSpiTransferWords) if the
     * clock frequency is being dropped below 4 MHz.
     */
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 8000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create the DMA channels for SPI write and read. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this channel
     * will be used only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Channel to write to SPI flash. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
    status = CyU3PDmaChannelCreate (&glSpiTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Channel to read from SPI flash. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glSpiRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glSpiPageSize = pageLen;
    }

    return status;
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t
CyFxFlashProgSpiWaitForStatus (
        void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do
    {
        buf[0] = 0x06;  /* Write enable command. */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransferWords (buf, 1, 0, 0);
        CyU3PSpiSetSsnLine (CyTrue);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransferWords (buf, 1, 0, 0);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI READ_STATUS command failed\n\r");
            CyU3PSpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyU3PSpiTransferWords (0, 0, rd_buf, 2);
        CyU3PSpiSetSsnLine (CyTrue);
        if(status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI status read failed\n\r");
            return status;
        }

    } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgSpiTransfer (
        uint16_t  pageAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PDmaBuffer_t buf_p;
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / glSpiPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }
    if ((byteCount % glSpiPageSize) != 0)
    {
        pageCount ++;
    }

    buf_p.buffer = buffer;
    buf_p.status = 0;

    byteAddress  = pageAddress * glSpiPageSize;
    CyU3PDebugPrint (2, "SPI access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            byteAddress, byteCount, pageCount);

    while (pageCount != 0)
    {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead)
        {
            location[0] = 0x03; /* Read command. */

            buf_p.size  = glSpiPageSize;
            buf_p.count = glSpiPageSize;

            status = CyFxFlashProgSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }

            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransferWords (location, 4, 0, 0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "SPI READ command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetBlockXfer (0, glSpiPageSize);

            status = CyU3PDmaChannelSetupRecvBuffer (&glSpiRxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion (&glSpiRxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);
        }
        else /* Write */
        {
            location[0] = 0x02; /* Write command */

            buf_p.size  = glSpiPageSize;
            buf_p.count = glSpiPageSize;

            status = CyFxFlashProgSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransferWords (location, 4, 0, 0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "SPI WRITE command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetBlockXfer (glSpiPageSize, 0);

            status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyTrue, CyFalse);
        }

        /* Update the parameters */
        byteAddress  += glSpiPageSize;
        buf_p.buffer += glSpiPageSize;
        pageCount --;

        CyU3PThreadSleep (10);
    }
    return CY_U3P_SUCCESS;
}

/* Function to read SPI flash device ID */
static CyU3PReturnStatus_t
FlashReadID (uint8_t  *wip)
{
    uint8_t  location[4];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x90;  /* Write enable. */
    location[1] = 0;
    location[2] = 0;
    location[3] = 0;


    CyU3PSpiSetSsnLine (CyFalse);

    status = CyU3PSpiTransferWords (location, 4, 0, 0);
    status = CyU3PSpiTransferWords (0, 0, wip, 2);
    CyU3PSpiSetSsnLine (CyTrue);

    return status;
}


/* Function to erase SPI flash sectors. */
static CyU3PReturnStatus_t
CyFxFlashProgEraseSector (
     CyBool_t  isErase,
     uint8_t   sector,
     uint8_t  *wip)
{
    uint32_t temp = 0;
    uint8_t  location[4];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((!isErase) && (wip == NULL))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    location[0] = 0x06;  /* Write enable. */

    CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransferWords (location, 1, 0, 0);
    CyU3PSpiSetSsnLine (CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    if (isErase)
    {
        location[0] = 0xD8; /* Sector erase. */
        temp        = sector * 0x10000;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransferWords (location, 4, 0, 0);
        CyU3PSpiSetSsnLine (CyTrue);
    }
    else
    {
        location[0] = 0x05; /* Read status */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransferWords (location, 1, 0, 0);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PSpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyU3PSpiTransferWords (0, 0, wip, 2);
        CyU3PSpiSetSsnLine (CyTrue);
    }

    return status;
}


/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

void
CyFxUSBUARTDmaCallback(
        CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
        CyU3PDmaCbType_t   type,     /* Callback type.             */
        CyU3PDmaCBInput_t *input)    /* Callback status.           */
{
    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        CyU3PDmaChannelCommitBuffer (&glChHandleUarttoUsb, input->buffer_p.count, 0);
        glPktsPending++;
    }
}

/* This function starts the USBUART application */
void
CyFxUSBUARTAppStart(
        void )
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* Based on the Bus speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case  CY_U3P_SUPER_SPEED:
            /* Turning low power mode off to avoid USB transfer delays. */
            CyU3PUsbLPMDisable ();
            size = 1024;
            break;

        default:
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER , &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Interrupt endpoint configuration */
    epCfg.epType = CY_U3P_USB_EP_INTR;
    epCfg.pcktSize = 64;
    epCfg.isoPkts = 1;

    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_INTERRUPT, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }


    /* Create a DMA_AUTO channel between usb producer socket and uart consumer socket */
    dmaCfg.size = size;
    dmaCfg.count = CY_FX_USBUART_DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_FX_EP_PRODUCER1_SOCKET;
    dmaCfg.consSckId = CY_FX_EP_CONSUMER1_SOCKET;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleUsbtoUart,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Create a DMA_MANUAL channel between uart producer socket and usb consumer socket */
    /* Use a smaller buffer size (32 bytes) to ensure that packets get filled in a short time. */
    dmaCfg.size         = 32;
    dmaCfg.prodSckId    = CY_FX_EP_PRODUCER2_SOCKET;
    dmaCfg.consSckId    = CY_FX_EP_CONSUMER2_SOCKET;
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dmaCfg.cb           = CyFxUSBUARTDmaCallback;

    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleUarttoUsb,
            CY_U3P_DMA_TYPE_MANUAL, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set DMA Channel transfer size */
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleUsbtoUart,0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleUarttoUsb, 0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Update the status flag. */
    // glIsApplnActive = CyTrue;
} 

void
CyFxUSBUARTAppStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    // glIsApplnActive = CyFalse;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
    CyU3PUsbFlushEp(CY_FX_EP_INTERRUPT);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleUsbtoUart);
    CyU3PDmaChannelDestroy (&glChHandleUarttoUsb);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Interrupt endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_INTERRUPT, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }
}

/* This is the callback function to handle the USB events. */
void
CyFxSlFifoUtoPDmaCallback (
        CyU3PDmaChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is 
         * received upon reception of every buffer. The buffer will not be sent
         * out unless it is explicitly committed. The call shall fail if there
         * is a bus reset / usb disconnect or if there is any application error. */
        status = CyU3PDmaChannelCommitBuffer (chHandle, input->buffer_p.count, 0);
        if (status != CY_U3P_SUCCESS)
        {
           // CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
        }

        /* Increment the counter. */
        glDMARxCount++;
    }
}

/* DMA callback function to handle the produce events for P to U transfers. */
void
CyFxSlFifoPtoUDmaCallback (
        CyU3PDmaChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is 
         * received upon reception of every buffer. The buffer will not be sent
         * out unless it is explicitly committed. The call shall fail if there
         * is a bus reset / usb disconnect or if there is any application error. */
        status = CyU3PDmaChannelCommitBuffer (chHandle, input->buffer_p.count, 0);
        if (status != CY_U3P_SUCCESS)
        {
           // CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
        }

        /* Increment the counter. */
        glDMATxCount++;
    }
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxSlFifoApplnStart (
        void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            break;

        default:
           // CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(SLAVE_FIFO_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(SLAVE_FIFO_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Create a DMA MANUAL channel for U2P transfer.
     * DMA size is set based on the USB speed. */
    dmaCfg.size  = DMA_BUF_SIZE_TX* 1024;
    dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT_U_2_P;
    dmaCfg.prodSckId = CY_FX_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_PPORT_SOCKET;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    /* Enabling the callback for produce event. */
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dmaCfg.cb = CyFxSlFifoUtoPDmaCallback;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoUtoP,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Create a DMA AUTO channel for P2U transfer. */
    dmaCfg.size  = DMA_BUF_SIZE_RX*1024; //increase buffer size for higher performance
    dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U; // increase buffer count for higher performance
    dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET;
    dmaCfg.cb = NULL;
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

	if (apiRetStatus != CY_U3P_SUCCESS)
	{
	 CyFxAppErrorHandler(apiRetStatus);
	}

	/* Flush the Endpoint memory */
	CyU3PUsbFlushEp(SLAVE_FIFO_EP_PRODUCER);
	CyU3PUsbFlushEp(SLAVE_FIFO_EP_CONSUMER);

	/* Set DMA channel transfer size. */
	apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
	 //CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
	 CyFxAppErrorHandler(apiRetStatus);
	}
	apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
	 //CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
	 CyFxAppErrorHandler(apiRetStatus);
	}

    /* Update the status flag. */
    // glIsApplnActive = CyTrue;
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void
CyFxSlFifoApplnStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    // glIsApplnActive = CyFalse;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(SLAVE_FIFO_EP_PRODUCER);
    CyU3PUsbFlushEp(SLAVE_FIFO_EP_CONSUMER);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleSlFifoUtoP);
    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(SLAVE_FIFO_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(SLAVE_FIFO_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t
CyFxUSBSetupCB (
        uint32_t setupdat0,
        uint32_t setupdat1
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    uint8_t  i2cAddr = 0;
    uint32_t *addr = NULL;
    int32_t offset = 0;
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;
    uint16_t readCount = 0;
    uint8_t config_data[7];
    //CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUartConfig_t uartConfig;
    CyU3PIoMatrixConfig_t io_cfg;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == SLAVE_FIFO_EP_PRODUCER)
                {
                    CyU3PUsbSetEpNak (SLAVE_FIFO_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
                    CyU3PUsbFlushEp(SLAVE_FIFO_EP_PRODUCER);
                    CyU3PUsbResetEp (SLAVE_FIFO_EP_PRODUCER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);

                    CyU3PUsbSetEpNak (SLAVE_FIFO_EP_PRODUCER, CyFalse);
                }

                if (wIndex == SLAVE_FIFO_EP_CONSUMER)
                {
                    CyU3PUsbSetEpNak (SLAVE_FIFO_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(SLAVE_FIFO_EP_CONSUMER);
                    CyU3PUsbResetEp (SLAVE_FIFO_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);

                    CyU3PUsbSetEpNak (SLAVE_FIFO_EP_PRODUCER, CyFalse);
                }

                CyU3PUsbStall (wIndex, CyFalse, CyTrue);

                CyU3PUsbAckSetup ();
                isHandled = CyTrue;
            }
        }
    }

    /* Handle supported vendor requests. */
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
        isHandled = CyTrue;

        switch (bRequest)
        {
            case CY_FX_RQT_ID_CHECK:
                CyU3PUsbSendEP0Data (8, (uint8_t *)glFirmwareID);
                break;

            case CY_FX_RQT_I2C_EEPROM_WRITE:
                i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                if (status == CY_U3P_SUCCESS)
                {
                    CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength,
                            glEp0Buffer, CyFalse);
                }
                break;

            case CY_FX_RQT_I2C_EEPROM_READ:
                i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                status = CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength,
                        glEp0Buffer, CyTrue);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
                }
                break;

            case CY_FX_RQT_SYS_MEM_READ:
                addr = (uint32_t *)((wIndex << 16) | wValue);
                offset = 0;
                if (wLength)
                {
                    while (offset < (wLength / 4))
                    {
                        ((uint32_t *)glEp0Buffer)[offset++] = *addr;
                        addr++;
                    }

                    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                }
                else
                {
                    /* Zero length read. Just ACK.*/
                    CyU3PUsbAckSetup ();
                }
                break;

            case CY_FX_RQT_SPI_FLASH_WRITE:
                status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyFxFlashProgSpiTransfer (wIndex, wLength,
                            glEp0Buffer, CyFalse);
                }
                break;

            case CY_FX_RQT_SPI_FLASH_ID_READ:
                FlashReadID(glEp0Buffer);
                CyU3PUsbSendEP0Data (2, glEp0Buffer);
                break;

            case CY_FX_RQT_SPI_FLASH_READ:
                CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                status = CyFxFlashProgSpiTransfer (wIndex, wLength,
                        glEp0Buffer, CyTrue);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                }
                break;

            case CY_FX_RQT_SPI_FLASH_ERASE_POLL:
                status = CyFxFlashProgEraseSector ((wValue) ? CyTrue : CyFalse,
                        (wIndex & 0xFF), glEp0Buffer);
                if (status == CY_U3P_SUCCESS)
                {
                    if (wValue == 0)
                    {
                        status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                    }
                    else
                    {
                    	CyU3PUsbSendEP0Data (1, (uint8_t *)glFirmwareID);
                        // CyU3PUsbAckSetup ();
                    }
                }
                break;

            case CY_FX_RQT_GET_FW_VERSION:
                {
                    CyU3PSysGetApiVersion (
                            (uint16_t *)(glEp0Buffer),
                            (uint16_t *)(glEp0Buffer + 2),
                            (uint16_t *)(glEp0Buffer + 4),
                            (uint16_t *)(glEp0Buffer + 6)
                            );
                    CyU3PUsbSendEP0Data (8, glEp0Buffer);
                }
                break;

            case CY_FX_RQT_DISABLE_UART_ENABLE_SPI:
            	CyFxUSBUARTAppStop();
            	CyFxSlFifoApplnStop();
            	// Disable peripherals before
            	CyU3PUartDeInit ();
            	CyU3PI2cDeInit();
            	// CyU3PGpioDeInit();
            	CyU3PGpifDisable(CyTrue);
            	CyU3PPibDeInit();

                io_cfg.isDQ32Bit = CyFalse;
                io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
                io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
                io_cfg.useUart   = CyTrue;
                io_cfg.useI2C    = CyTrue;
                io_cfg.useI2S    = CyFalse;
                io_cfg.useSpi    = CyTrue;
                io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
                /* No GPIOs are enabled. */
                io_cfg.gpioSimpleEn[0]  = 0;
                io_cfg.gpioSimpleEn[1]  = 0;
                io_cfg.gpioComplexEn[0] = 0;
                io_cfg.gpioComplexEn[1] = 0;
                CyU3PDeviceConfigureIOMatrix (&io_cfg);

                CyFxSlFifoApplnInit();
            	/* Initialize the SPI interface for flash of page size 256 bytes. */
            	CyFxFlashProgSpiInit (0x100);

            	CyFxSlFifoApplnStart();

            	CyU3PUsbSendEP0Data (1, (uint8_t *)glFirmwareID);
                break;

            case 0xE0:
                CyU3PUsbAckSetup ();
                CyU3PDeviceReset (CyFalse);
                break;

            default:
                /* This is unknown request. */
                isHandled = CyFalse;
                break;
        }
        /* If there was any error, return not handled so that the library will
         * stall the request. Alternatively EP0 can be stalled here and return
         * CyTrue. */
        if (status != CY_U3P_SUCCESS)
        {
            isHandled = CyFalse;
        }
    }

    /* Check for CDC Class Requests */
    if (bType == CY_U3P_USB_CLASS_RQT)
    {
        isHandled = CyTrue;

        /* CDC Specific Requests */
        /* set_line_coding */
        if (bRequest == SET_LINE_CODING)
        {
            status = CyU3PUsbGetEP0Data(0x07, config_data, &readCount);
            if (status != CY_U3P_SUCCESS)
            {
                CyFxAppErrorHandler(status);
            }
            if (readCount != 0x07)
            {
                CyFxAppErrorHandler(CY_U3P_ERROR_BAD_SIZE);
            }
            else
            {
                CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
                uartConfig.baudRate = (CyU3PUartBaudrate_t)(config_data[0] | (config_data[1]<<8)|
                        (config_data[2]<<16)|(config_data[3]<<24));
                if (config_data[4] == 0)
                {
                    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
                }
                else if (config_data[4] == 2)
                {
                    uartConfig.stopBit = CY_U3P_UART_TWO_STOP_BIT;
                }
                else
                {
                    /* Give invalid value. */
                    uartConfig.stopBit = (CyU3PUartStopBit_t)0;
                }
                if (config_data[5] == 1)
                {
                    uartConfig.parity = CY_U3P_UART_ODD_PARITY;
                }
                else if (config_data[5] == 2)
                {
                    uartConfig.parity = CY_U3P_UART_EVEN_PARITY;
                }
                else
                {
                    /* 0 = no parity; any other value - invalid parity. */
                    uartConfig.parity = CY_U3P_UART_NO_PARITY;
                }

                uartConfig.txEnable = CyTrue;
                uartConfig.rxEnable = CyTrue;
                uartConfig.flowCtrl = CyFalse;
                uartConfig.isDma = CyTrue;

                /* Set the uart configuration */
                apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
                if (apiRetStatus == CY_U3P_SUCCESS)
                {
                    CyU3PMemCopy ((uint8_t *)&glUartConfig, (uint8_t *)&uartConfig,
                            sizeof (CyU3PUartConfig_t));
                }
            }
        }
        /* get_line_coding */
        else if (bRequest == GET_LINE_CODING )
        {
            /* get current uart config */
            config_data[0] = glUartConfig.baudRate&(0x000000FF);
            config_data[1] = ((glUartConfig.baudRate&(0x0000FF00))>> 8);
            config_data[2] = ((glUartConfig.baudRate&(0x00FF0000))>>16);
            config_data[3] = ((glUartConfig.baudRate&(0xFF000000))>>24);
            if (glUartConfig.stopBit == CY_U3P_UART_ONE_STOP_BIT)
            {
                config_data[4] = 0;
            }
            else /* CY_U3P_UART_TWO_STOP_BIT */
            {
                config_data[4] = 2;
            }

            if (glUartConfig.parity == CY_U3P_UART_EVEN_PARITY)
            {
                config_data[5] = 2;
            }
            else if (glUartConfig.parity == CY_U3P_UART_ODD_PARITY)
            {
                config_data[5] = 1;
            }
            else
            {
                config_data[5] = 0;
            }
            config_data[6] =  0x08;
            status = CyU3PUsbSendEP0Data( 0x07, config_data);
            if (status != CY_U3P_SUCCESS)
            {
                CyFxAppErrorHandler(status);
            }
        }
        /* SET_CONTROL_LINE_STATE */
        else if (bRequest == SET_CONTROL_LINE_STATE)
        {
            if (glIsApplnActive)
            {
                CyU3PUsbAckSetup ();
            }
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);
        }
        else
        {
            status = CY_U3P_ERROR_FAILURE;
        }

        if (status != CY_U3P_SUCCESS)
        {
            isHandled = CyFalse;
        }
    }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
CyFxUSBEventCB (
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
    )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                /* Update the flag. */
                glIsApplnActive = CyFalse;
                CyFxSlFifoApplnStop ();
                CyFxUSBUARTAppStop ();
            }
            /* Start the loop back function. */
            CyFxSlFifoApplnStart ();
            CyFxUSBUARTAppStart ();
            /* Update the status flag. */
            glIsApplnActive = CyTrue;
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            if (glIsApplnActive)
            {
                /* Update the flag. */
                glIsApplnActive = CyFalse;
                CyFxSlFifoApplnStop ();
                CyFxUSBUARTAppStop ();
            }
            /* Reset the I2C and SPI DMA channels. */
            CyU3PDmaChannelReset (&glI2cTxHandle);
            CyU3PDmaChannelReset (&glI2cRxHandle);
            CyU3PDmaChannelReset (&glSpiTxHandle);
            CyU3PDmaChannelReset (&glSpiRxHandle);
            break;

        default:
            break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}



void
CyFxUSBUARTAppInit (
        void )

{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the UART module */
    apiRetStatus = CyU3PUartInit ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Configure the UART */
    CyU3PMemSet ((uint8_t *)&glUartConfig, 0, sizeof (glUartConfig));
    glUartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    glUartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    glUartConfig.parity = CY_U3P_UART_NO_PARITY;
    glUartConfig.flowCtrl = CyFalse;
    glUartConfig.txEnable = CyTrue;
    glUartConfig.rxEnable = CyTrue;
    glUartConfig.isDma = CyTrue;

    /* Set the UART configuration */
    apiRetStatus = CyU3PUartSetConfig (&glUartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS )
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);    

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void
CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the I2C interface for the EEPROM of page size 64 bytes. */
    CyFxFlashProgI2cInit (0x40);

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "P-port Initialization failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PGpifLoad failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

// set the watermark for thread_0 (Ingress) to 5
    CyU3PGpifSocketConfigure (0,CY_U3P_PIB_SOCKET_0,5,CyFalse,1);
// set the watermark for thread_3 (Egress) to 0
    CyU3PGpifSocketConfigure (3,CY_U3P_PIB_SOCKET_3,0,CyFalse,1);

    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart (RESET, ALPHA_RESET);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}

void
USBUARTAppThread_Entry (
        uint32_t input)
{
    uint32_t regValueEn = 0, regValueDs = 0;

    /* Initialize the USBUART Example Application */
   CyFxUSBUARTAppInit();

    /* UART Config Value for Enabling Rx Block */
    regValueEn = UART->lpp_uart_config;

    /* UART Config Value for Disabling the Rx Block  */
    regValueDs = UART->lpp_uart_config & (~(CY_U3P_LPP_UART_RTS | CY_U3P_LPP_UART_RX_ENABLE));

    for (;;)
    {
        if (glIsApplnActive)
        {
            /* While the application is active, check for data sent during the last 50 ms. If no data
               has been sent to the host, use the channel wrap-up feature to send any partial buffer to
               the USB host.
            */
            if (glPktsPending == 0)
            {
                /* Disable UART Receiver Block */
                UART->lpp_uart_config = regValueDs;

                CyU3PDmaChannelSetWrapUp (&glChHandleUarttoUsb);

                /* Enable UART Receiver Block */
                UART->lpp_uart_config = regValueEn;
            }

            glPktsPending = 0;
        }

        CyU3PThreadSleep (50);
    }
}


/* Entry function for the slFifoAppThread. */
void
SlFifoAppThread_Entry (
        uint32_t input)
{
    /* Initialize the slave FIFO application */
    CyFxSlFifoApplnInit();

    for (;;)
    {
        CyU3PThreadSleep (1000);
        if (glIsApplnActive)
        {
            /* Print the number of buffers received so far from the USB host. */
            //CyU3PDebugPrint (6, "Data tracker: buffers received: %d, buffers sent: %d.\n",
                  //  glDMARxCount, glDMATxCount);
        }
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    void *ptr1 = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);
    ptr1 = CyU3PMemAlloc(1000);

    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
                          "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
                          SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

    // USB start is called in Slave fifo init. If UART is init before slave fifo, then we need to add it in UART init.
    retThrdCreate = CyU3PThreadCreate (&USBUARTAppThread,          /* USBUART Example App Thread structure */
                "22:USBUART_DMA_mode",                   /* Thread ID and Thread name */
                USBUARTAppThread_Entry,                  /* USBUART Example App Thread Entry function */
                0,                                       /* No input parameter to thread */
                ptr1,                                     /* Pointer to the allocated thread stack */
                1000,                                    /* USBUART Example App Thread stack size */
                7,//CY_FX_USBUART_THREAD_PRIORITY,            /* USBUART Example App Thread priority */
                7,//CY_FX_USBUART_THREAD_PRIORITY,            /* USBUART Example App Thread priority */
                CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                CYU3P_AUTO_START                         /* Start the Thread immediately */
                );


    /* Check the return code */
    if (retThrdCreate != 0)
    {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }

}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

	/* setSysClk400 clock configurations */
	clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
	clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
	clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
	clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
	clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
	clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

    /* Initialize the device */
    status = CyU3PDeviceInit (&clkCfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large 
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
     * isDQ32Bit for 32-bit slave FIFO configuration. */
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_UART_ONLY;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

