# MCU firmware for CutiePi board 

## Dependency 

    sudo apt install gcc-arm-none-eabi stlink-tools

## Flash 

| CutiePi board J7  | ST-LINK/V2 Pinout  |
| ----------------- | ------------------ |
| 3V3               |  TVCC              |
| SWDIO             |  DIO               |
| SWCLK             |  DCLK              |
| GND               |  GND               |

![](https://i.imgur.com/JhyHX9E.jpg)

    st-flash --format ihex write ./stm32f0.hex 

![](https://i.imgur.com/7rgrJp4.png)

## Compile 

    cd Debug/
    make -f makefile all 
