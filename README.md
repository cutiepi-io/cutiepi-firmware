# MCU firmware for CutiePi board 

## Tools Required

- ST-Link v2 flasher 
- SH1.0 4p connector 

Please refer to the [Hardware Maintenance Guide](https://github.com/cutiepi-io/cutiepi-doc/blob/main/HardwareMaintenanceGuide.md) for opening up the device.

## Dependency 

    sudo apt install gcc-arm-none-eabi stlink-tools

## Flash 

| CutiePi board J7  | ST-LINK/V2 Pinout  |
| ----------------- | ------------------ |
| 3V3               |  TVCC              |
| SWDIO             |  DIO               |
| SWCLK             |  DCLK              |
| GND               |  GND               |

![](https://i.imgur.com/ZxYCMJi.jpg)

    st-flash --format ihex write ./stm32f0.hex 

![](https://i.imgur.com/7rgrJp4.png)

## Compile 

    cd Debug/
    make -f Makefile all 
