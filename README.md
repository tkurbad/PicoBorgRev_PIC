# PicoBorgRev_PIC

## Compile
```
sdcc -mpic14 -p16f1824 --use-non-free picoborgrev.c
```
 
## Program with PicKit2 using pk2cmd
```
pk2cmd -M -PPIC16f1824 -Fpicoborgrev.hex
```
