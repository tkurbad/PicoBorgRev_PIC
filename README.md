# PicoBorg Reverse PIC code

This is a port of the PIC code for the PicoBorg Reverse Advanced motor
controller for the Raspberry Pi, available here:

https://www.piborg.org/picoborgrev

## Introduction

I ported the code from Microchip's proprietary MPLAB(X) IDE / XC8 compiler
to the Small Devices C Compiler.

For more information, visit http://sdcc.sf.net.

The intention in doing so was to find an issue with the code, where the
PIC returns `0x00` on all I²C reads, while I²C writes seem to work fine.
This problem occurs when the PicoBorg Reverse is connected to a Beaglebone
Blue or to a Raspberry PI running recent Linux kernels (4.4.50+) on
Raspbian.

A cheap and simple logic analyzer https://www.amazon.de/gp/product/B01MUFRHQ2
revealed two problems with the existing code:

* I²C clock stretching wasn't properly implemented.
* For all `GET_...` commands, i.e. commands where the bus master reads from
  the PIC, the output array was cleared *before* being sent back to the I²C
  master.

The present code has a completely rewritten interrupt service routine that
eliminates both issues.

## Compile
```
make
```
 
## Program the PIC with a PicKit2 compatible programmer using pk2cmd
```
make upload
```

## Acknowledgements

I want to thank the nice people at http://www.piborg.org for their well
written and nicely documented code. Those qualities made it very easy to
port the project to the `sdcc` and fix the problems I was having with the
motor controller.

## License

[creative commons CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/)
