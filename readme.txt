This code refers to the sample code in https://community.infineon.com/t5/USB-superspeed-peripherals/Slave-FIFO-UART-Driver-Setup/td-p/164284, and fixed the bugs in slave fifo, uart and usb descriptor.
The code is tested on Cy3014 DV board.

### Update
* Add UART
* Add I2C r/w
* Add RAM read and USB reset
* Add SPI r/w
* Add switching from UART mode to SPI mode

### Files:

    * cyfx_gcc_startup.S   : Start-up code for the ARM-9 core on the FX3
      device.  This assembly source file follows the syntax for the GNU
      assembler.

    * cyfxslfifosync.h     : C header file that defines constants used by
      this example implementation.  Can be modified to select USB connection
      speed, endpoint numbers and properties etc.
      
    * cyfxflashprog.h     : C header file that defines vendor commands used by
      this example implementation.

    * cyfxslfifousbdscr.c  : C source file that contains USB descriptors
      used by this example. VID and PID is defined in this file.

    * cyfxgpif2config.h    : C header file that contains the data required
      to configure the GPIF interface to implement the Sync. Slave FIFO
      protocol.

    * cyfxtx.c             : C source file that provides ThreadX RTOS wrapper
      functions and other utilites required by the FX3 firmware library.

    * cyfxslfifosync.c     : Main C source file that implements this example.

    * makefile             : GNU make compliant build script for compiling
      this example.


