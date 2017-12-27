# stm32-qspi-fat-usb
STM32 Cube and System Workbench project for STM32L476 Discovery Board

# description

A microcontroller in this projects functions as a USB flash drive. This drive contains the CONFIG.TXT file with a single decimal number in it between 1 and 999. This number can be altered by user after connecting the board into PC and modifying the file as usual.

After board startup this file is read by MCU and the number is assigned to the LED blinking half-period in milliseconds. The value 500 means that LED blinks every second.

The data is stored in the QSPI chip located on the board. This chip contains FAT file system which is accessed in read-write mode by PC and in read-only mode by MCU. The size of this flash is 16 MBit.

# start guide

Use the .ioc file to create SW ARM project for the Discovery board. Use the .bin image file to program QSPI flash on the Discovery board via STM32 St-Link utility (add external memory loader for the N25Q128A QSPI chip on the STM32L476 board).
Make sure the files in /inc and /src directories are added to project. Compile the project and download the binary into MCU.
