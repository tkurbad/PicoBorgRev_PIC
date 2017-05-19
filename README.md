# PicoBorg Reverse PIC code

This is a port of the PIC code for the PicoBorg Reverse Advanced motor controller for the Raspberry Pi, available here:

https://www.piborg.org/picoborgrev

I ported the code from Microchip's proprietary MPLAB(X) IDE / XC8 compiler to the Small Devices C Compiler.
For more info, visit http://sdcc.sf.net.

## Compile
```
sdcc -mpic14 -p16f1824 --use-non-free picoborgrev.c
```
 
## Program with PicKit2 using pk2cmd
```
pk2cmd -M -PPIC16f1824 -Fpicoborgrev.hex
```
