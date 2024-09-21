/*
 ## Cypress USB 3.0 Platform header file (cyfxslfifousbdscr.c)
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

/* This file contains the USB enumeration descriptors for the slave FIFO application example.
 * The descriptor arrays must be 32 byte aligned and multiple of 32 bytes if the D-cache is
 * turned on. If the linker used is not capable of supporting the aligned feature for this,
 * either the descriptors must be placed in a different section and the section should be 
 * 32 byte aligned and 32 byte multiple; or dynamically allocated buffer allocated using
 * CyU3PDmaBufferAlloc must be used, and the descriptor must be loaded into it. The example
 * assumes that the aligned attribute for 32 bytes is supported by the linker. Do not add
 * any other variables to this file other than USB descriptors. This is not the only
 * pre-requisite to enabling the D-cache. Refer to the documentation for
 * CyU3PDeviceCacheControl for more information.
 */

#include "cyfxslfifosync.h"

/* Standard device descriptor for USB 3.0 */
const uint8_t CyFxUSB30DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x10,0x03,                      /* USB 3.1 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    0xB4,0x04,                      /* Vendor ID */
    0x04,0x00,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device descriptor for USB 2.0 */
const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x10,0x02,                      /* USB 2.10 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID */
    0x04,0x00,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Binary device object store descriptor */
const uint8_t CyFxUSBBOSDscr[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    CY_U3P_BOS_DESCR,               /* Device descriptor type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_SS_USB_CAPB_TYPE,        /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
    0x03,                           /* Functionality support */
    0x00,                           /* U1 Device Exit latency */
    0x00,0x00                       /* U2 Device Exit latency */
};

/* Standard device qualifier descriptor */
const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (32))) =
{
    0x0A,                           /* Descriptor size */
    CY_U3P_USB_DEVQUAL_DESCR,       /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard super speed configuration descriptor */
const uint8_t CyFxUSBSSConfigDscr[] __attribute__ ((aligned (32))) =
{
	0x09,                           /* Descriptor size */
	CY_U3P_USB_CONFIG_DESCR,         /* Configuration Descriptor type */
	0x44,0x00,                      /* Total length of data returned for this config (68 bytes) */
	0x03,                           /* Number of interfaces (3) */
	0x01,                           /* Configuration value */
	0x00,                           /* Configuration string index */
	0xC0,                           /* Attributes (self-powered) */
	0x32,                           /* Max power consumption (100mA) */

	/* Interface Association Descriptor (IAD) */
	0x08,                           /* Descriptor size */
	0x0B,                           /* Interface Association Descriptor type */
	0x01,                           /* First interface number (Interface 1 - Communication Interface) */
	0x02,                           /* Number of interfaces associated with this function (2: Comm + Data) */
	0x02,                           /* Function class code: Communication Class (CDC) */
	0x02,                           /* Function subclass code */
	0x01,                           /* Function protocol code */
	0x00,                           /* Function descriptor string index */

	/* Interface Descriptor 1 - Communication Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x01,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of endpoints */
	0x02,                           /* Interface class: Communication Interface */
	0x02,                           /* Interface sub class */
	0x01,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Communication Interface (Interrupt IN) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x82,                           /* Endpoint address (IN, Endpoint 2) */
	CY_U3P_USB_EP_INTR,              /* Interrupt endpoint type */
	0x40,0x00,                      /* Max packet size = 64 bytes */
	0x01,                           /* Interval */

	/* Interface Descriptor 2 - Data Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x02,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x02,                           /* Number of endpoints */
	0x0A,                           /* Interface class: Data Interface (for CDC) */
	0x00,                           /* Interface sub class */
	0x00,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Data Interface (Bulk Producer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x05,                           /* Endpoint address (OUT, Endpoint 5) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for producer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */

	/* Endpoint Descriptor for Data Interface (Bulk Consumer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x86,                           /* Endpoint address (IN, Endpoint 6) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for consumer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */
};

/* Standard high speed configuration descriptor */
const uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (32))) =
{
	0x09,                           /* Descriptor size */
	CY_U3P_USB_CONFIG_DESCR,         /* Configuration Descriptor type */
	0x44,0x00,                      /* Total length of data returned for this config (68 bytes) */
	0x03,                           /* Number of interfaces (3) */
	0x01,                           /* Configuration value */
	0x00,                           /* Configuration string index */
	0xC0,                           /* Attributes (self-powered) */
	0x32,                           /* Max power consumption (100mA) */

	/* Interface Association Descriptor (IAD) */
	0x08,                           /* Descriptor size */
	0x0B,                           /* Interface Association Descriptor type */
	0x01,                           /* First interface number (Interface 1 - Communication Interface) */
	0x02,                           /* Number of interfaces associated with this function (2: Comm + Data) */
	0x02,                           /* Function class code: Communication Class (CDC) */
	0x02,                           /* Function subclass code */
	0x01,                           /* Function protocol code */
	0x00,                           /* Function descriptor string index */

	/* Interface Descriptor 1 - Communication Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x01,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of endpoints */
	0x02,                           /* Interface class: Communication Interface */
	0x02,                           /* Interface sub class */
	0x01,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Communication Interface (Interrupt IN) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x82,                           /* Endpoint address (IN, Endpoint 2) */
	CY_U3P_USB_EP_INTR,              /* Interrupt endpoint type */
	0x40,0x00,                      /* Max packet size = 64 bytes */
	0x01,                           /* Interval */

	/* Interface Descriptor 2 - Data Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x02,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x02,                           /* Number of endpoints */
	0x0A,                           /* Interface class: Data Interface (for CDC) */
	0x00,                           /* Interface sub class */
	0x00,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Data Interface (Bulk Producer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x05,                           /* Endpoint address (OUT, Endpoint 5) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for producer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */

	/* Endpoint Descriptor for Data Interface (Bulk Consumer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x86,                           /* Endpoint address (IN, Endpoint 6) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for consumer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */
};


/* Standard full speed configuration descriptor */
const uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (32))) =
{
	0x09,                           /* Descriptor size */
	CY_U3P_USB_CONFIG_DESCR,         /* Configuration Descriptor type */
	0x44,0x00,                      /* Total length of data returned for this config (68 bytes) */
	0x03,                           /* Number of interfaces (3) */
	0x01,                           /* Configuration value */
	0x00,                           /* Configuration string index */
	0xC0,                           /* Attributes (self-powered) */
	0x32,                           /* Max power consumption (100mA) */

	/* Interface Association Descriptor (IAD) */
	0x08,                           /* Descriptor size */
	0x0B,                           /* Interface Association Descriptor type */
	0x01,                           /* First interface number (Interface 1 - Communication Interface) */
	0x02,                           /* Number of interfaces associated with this function (2: Comm + Data) */
	0x02,                           /* Function class code: Communication Class (CDC) */
	0x02,                           /* Function subclass code */
	0x01,                           /* Function protocol code */
	0x00,                           /* Function descriptor string index */

	/* Interface Descriptor 1 - Communication Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x01,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of endpoints */
	0x02,                           /* Interface class: Communication Interface */
	0x02,                           /* Interface sub class */
	0x01,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Communication Interface (Interrupt IN) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x82,                           /* Endpoint address (IN, Endpoint 2) */
	CY_U3P_USB_EP_INTR,              /* Interrupt endpoint type */
	0x40,0x00,                      /* Max packet size = 64 bytes */
	0x01,                           /* Interval */

	/* Interface Descriptor 2 - Data Interface (CDC) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,         /* Interface Descriptor type */
	0x02,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x02,                           /* Number of endpoints */
	0x0A,                           /* Interface class: Data Interface (for CDC) */
	0x00,                           /* Interface sub class */
	0x00,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Endpoint Descriptor for Data Interface (Bulk Producer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x05,                           /* Endpoint address (OUT, Endpoint 5) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for producer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */

	/* Endpoint Descriptor for Data Interface (Bulk Consumer) */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,         /* Endpoint descriptor type */
	0x86,                           /* Endpoint address (IN, Endpoint 6) */
	CY_U3P_USB_EP_BULK,              /* Bulk endpoint type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x00,                           /* Servicing interval for data transfers */

	/* Super-speed endpoint companion descriptor for consumer ep */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,        /* SS endpoint companion descriptor type */
	0x00,                           /* Max no. of packets in a Burst: 1 */
	0x00,                           /* Mult.: Max number of packets: 1 */
	0x40,0x00,                      /* Bytes per interval: 1024 */
};


/* Standard language ID string descriptor */
const uint8_t CyFxUSBStringLangIDDscr[] __attribute__ ((aligned (32))) =
{
    0x04,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
const uint8_t CyFxUSBManufactureDscr[] __attribute__ ((aligned (32))) =
{
    0x10,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    'C',0x00,
    'y',0x00,
    'p',0x00,
    'r',0x00,
    'e',0x00,
    's',0x00,
    's',0x00
};

/* Standard product string descriptor */
const uint8_t CyFxUSBProductDscr[] __attribute__ ((aligned (32))) =
{
    0x08,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    'F',0x00,
    'X',0x00,
    '3',0x00
};
//const uint8_t CyFxUSBProductDscr1[] __attribute__ ((aligned (32)))=
//{
//	    0x09,                           /* Descriptor size */
//	    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
//	    'F',0x00,
//	    'X',0x00,
//	    '3',0x00,
//	    '2',0x00
//};
/* Place this buffer as the last buffer so that no other variable / code shares
 * the same cache line. Do not add any other variables / arrays in this file.
 * This will lead to variables sharing the same cache line. */
const uint8_t CyFxUsbDscrAlignBuffer[32] __attribute__ ((aligned (32)));

/* [ ] */

