# Software based attestation for wearable medical devices
As part of embedded security research, I have developed a software based attestation algorithm to protect wearable 
medical devices from adversaries. The implementation is based on the Pioneer algorithm: https://cgis.cs.umd.edu/~elaine/docs/pioneer.pdf

This implementation is a working version specific to the nrf51882 microcontroller and Nexus 7, 32 GB running Android 4.3 and above. Further work on this project include further features for customization of the attestation routine and periodic attestation.

## Getting Started
To use the attestation routine, simply flash the .hex file from BLE_C to the device through copy and paste. </br>
In case there is a need to edit any of the features, edit the main.cpp file and make using the arm-gcc toolchain (see below).

### Prerequisites
arm-gcc toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm </br>
srecord (srec_cat): http://srecord.sourceforge.net/man/man1/srec_cat.html 

## Acknowledgments
Mbed OS BLE|UART: https://os.mbed.com/teams/Bluetooth-Low-Energy/code/ </br>
Attestation implementation based on: https://github.com/Chadderz121/trapp </br>
Android Application: https://github.com/tdicola/BTLETest </br>
