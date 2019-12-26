# MCU firmware for CutiePi board 

## Dependency 

    sudo apt install gcc-arm-none-eabi stlink-tools

## Flash 

| CutiePi board J7  | ST-LINK/V2 Pinout  |
| ----------------- | ------------------ |
| 3V3               |  TVCC (2)          |
| SWDIO             |  DIO (7)           |
| SWCLK             |  DCLK (9)          |
| GND               |  GND (20)          |

![](https://i.imgur.com/Nxv41OE.jpg)

    st-flash --format ihex write ./stm32f0.hex 

![](https://i.imgur.com/d8V54W6.png)

## Compile 

    cd Debug/
    make -f makefile all 
