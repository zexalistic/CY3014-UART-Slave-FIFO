/*
 ## Cypress USB 3.0 Platform header file (cyfxslfifosync.h)
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

/* This file contains the constants and definitions used by the Slave FIFO application example */

#ifndef _INCLUDED_CYFXSLFIFOASYNC_H_
#define _INCLUDED_CYFXSLFIFOASYNC_H_

#include "cyu3externcstart.h"
#include "cyu3types.h"
#include "cyu3usbconst.h"

/* 16/32 bit GPIF Configuration select */
/* Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 0 for 16 bit GPIF data bus.
 * Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 1 for 32 bit GPIF data bus.
 */

#define DMA_BUF_SIZE						  (2)
#define CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U      (2)                       /* Slave FIFO P_2_U channel buffer count */
#define CY_FX_SLFIFO_DMA_BUF_COUNT_U_2_P 	  (2)						/* Slave FIFO U_2_P channel buffer count */
#define  CY_FX_USBUART_DMA_BUF_COUNT      (4)

#define  cdc

#define CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT (1)

#define CY_FX_SLFIFO_DMA_TX_SIZE        (0)	                  /* DMA transfer size is set to infinite */
#define CY_FX_SLFIFO_DMA_RX_SIZE        (0)	                  /* DMA transfer size is set to infinite */
#define CY_FX_SLFIFO_THREAD_STACK       (0x0400)                  /* Slave FIFO application thread stack size */
#define CY_FX_SLFIFO_THREAD_PRIORITY    (8)                       /* Slave FIFO application thread priority */

#define SLAVE_FIFO_EP_PRODUCER               0x02    /* EP 2 OUT */
#define SLAVE_FIFO_EP_CONSUMER               0x86    /* EP 6 IN */
#define CY_FX_EP_PRODUCER               0x05                             /* uart EP 5 OUT */
#define CY_FX_EP_CONSUMER               0x84                             /* UART EP 4 IN */
#define CY_FX_EP_INTERRUPT 				0x81

#define CY_FX_EP_PRODUCER1_SOCKET        CY_U3P_UIB_SOCKET_PROD_5
#define CY_FX_EP_CONSUMER1_SOCKET        CY_U3P_LPP_SOCKET_UART_CONS
#define CY_FX_EP_PRODUCER2_SOCKET        CY_U3P_LPP_SOCKET_UART_PROD
#define CY_FX_EP_CONSUMER2_SOCKET        CY_U3P_UIB_SOCKET_CONS_4
#define CY_FX_EP_INTR_CONSUMER1_SOCKET   CY_U3P_UIB_SOCKET_CONS_1

#define CY_FX_PRODUCER_USB_SOCKET    CY_U3P_UIB_SOCKET_PROD_2    /* USB Socket 2 is producer */
#define CY_FX_CONSUMER_USB_SOCKET    CY_U3P_UIB_SOCKET_CONS_6    /* USB Socket 6 is consumer */


/* Used with FX3 Silicon. */
#define CY_FX_PRODUCER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_0    /* P-port Socket 0 is producer */
#define CY_FX_CONSUMER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_3    /* P-port Socket 3 is consumer */

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXSLFIFOASYNC_H_ */

/*[]*/
