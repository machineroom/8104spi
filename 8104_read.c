/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include <bcm2835.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode=0;
static uint8_t bits = 8;
static uint32_t speed = 122000;

unsigned char spi_read(unsigned char MAP, int file) {
    unsigned char buf[2];
    unsigned char ret;
    int status;
    struct spi_ioc_transfer xfer[2];
    memset (xfer,0,sizeof(xfer));
     
    buf[0] = 0x00;  // XXXXXXX0 = write
    buf[1] = MAP;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 2; /* Length of  command to write*/
    status = ioctl(file, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return NULL;
    }
 
    memset (xfer,0,sizeof(xfer));
    buf[0] = 0x01;  // XXXXXXX1 = read
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 1; /* Length of  command to write*/
    xfer[1].rx_buf = (unsigned long) &ret;
    xfer[1].len = 1; /* Length of Data to read */
    status = ioctl(file, SPI_IOC_MESSAGE(2), xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return NULL;
    }
 
    return ret;
}
 
//////////
// Write n bytes int the 2 bytes address add1 add2
//////////
void spi_write(unsigned char MAP, unsigned char data, int file) {
    unsigned char buf[3];
    int status;
    struct spi_ioc_transfer xfer[1];
    memset (xfer,0,sizeof(xfer));

    buf[0] = 0x00;  // XXXXXXX0 = write
    buf[1] = MAP;
    buf[2] = data;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 3; /* Length of  command to write*/
    status = ioctl(file, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return;
    }
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
    if (!bcm2835_init()) return 1;
	
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

    //reset 8104
#define OS8104_RESET RPI_V2_GPIO_P1_03
 	bcm2835_gpio_fsel(OS8104_RESET, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(OS8104_RESET, LOW);
    delay(2);
    bcm2835_gpio_write(OS8104_RESET, HIGH);
    delay(2);

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
    
    spi_write (0x82,0x10,fd); // SCK O/P
    spi_write (0x85,0x04,fd); // clear power on interrupt    
    printf ("C4 = 0x%02X\n", spi_read (0xC4,fd));
    printf ("C5 = 0x%02X\n", spi_read (0xC5,fd));
    printf ("C6 = 0x%02X\n", spi_read (0xC6,fd));
    
	close(fd);

	return ret;
}
