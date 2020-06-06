# Custom-bootloader
In this project we have developed our own bootloader and flashed it in sector 0 to SRAM and also developed a user application in sector 2 to the SRAM .
When the user button is pressed upon reset the microcontroller jumps to our custom bootloader and our bootloader communicates with the host by UART .Otherwise it jumps to user application and turns on a LED for example,.
Host in our case is PC .Also it is a python script that provides our bootlaoder with commands.
