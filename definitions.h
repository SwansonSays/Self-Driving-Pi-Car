#ifndef DEFS_H_
#define DEFS_H_
#define PULSES_PER_REV  540.0
#define PI                      3.141592654
#define WHEEL_RADIUS    6.5


#define GPIO00  0               //Physical Pin 27 (ID_SD, I2C ID - Reserved)
#define GPIO01  1               //Physical Pin 28 (ID_SC, I2C ID - Reserved)
#define GPIO02  2               //Physical Pin 3 (SDA1 I2C)
#define GPIO03  3               //Physical Pin 5 (SCL1 I2C)
#define GPIO04  4               //Physical Pin 7
#define GPIO05  5               //Physical Pin 29
#define GPIO06  6               //Physical Pin 31
#define GPIO07  7               //Physical Pin 26 (SPI0_CE1)
#define GPIO08  8               //Physical Pin 24 (SPI0_CE0)
#define GPIO09  9               //Physical Pin 21 (SPI0_MISO)
#define GPIO10  10              //Physical Pin 19 (SPI0_MOSI)
#define GPIO11  11              //Physical Pin 23 (SPI0_SCLK)
#define GPIO12  12              //Physical Pin 32
#define GPIO13  13              //Physical Pin 33
#define GPIO14  14              //Physical Pin 8 (UART0_TX)
#define GPIO15  15              //Physical Pin 10 (UART0_RX)
#define GPIO16  16              //Physical Pin 36
#define GPIO17  17              //Physical Pin 11
#define GPIO18  18              //Physical Pin 12 ((PCM_CLK))
#define GPIO19  19              //Physical Pin 35
#define GPIO20  20              //Physical Pin 38
#define GPIO21  21              //Physical Pin 40
#define GPIO22  22              //Physical Pin 15
#define GPIO23  23              //Physical Pin 16
#define GPIO24  24              //Physical Pin 18
#define GPIO25  25              //Physical Pin 22
#define GPIO26  26              //Physical Pin 37
#define GPIO27  27              //Physical Pin 13

#define SPI0_MOSI       GPIO10          //Physical Pin 19
#define SPI0_MISO       GPIO09          //Physical Pin 21
#define SPI0_SCLK       GPIO11          //Physical Pin 23

#define SPI0_CE0        GPIO08          //Physical Pin 24
#define SPI0_CE1        GPIO07          //Physical Pin 26
#endif
