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

char * spi_read(unsigned char MAP, int cnt, int file) {
    unsigned char buf[2];
    unsigned char buf2[255];
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
    memset(buf2, 0, sizeof buf2);
    buf[0] = 0x01;  // XXXXXXX1 = read
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 1; /* Length of  command to write*/
    xfer[1].rx_buf = (unsigned long) buf2;
    xfer[1].len = cnt; /* Length of Data to read */
    status = ioctl(file, SPI_IOC_MESSAGE(2), xfer);
    if (status < 0)
    {
        perror("SPI_IOC_MESSAGE");
        return NULL;
    }
    //printf("env: %02x %02x %02x\n", buf[0], buf[1], buf[2]);
    printf("ret: %02x %02x %02x %02x\n", buf2[0], buf2[1], buf2[2], buf2[3]);
 
    return buf2;
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

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
    if (!bcm2835_init()) return 1;
	
	parse_opts(argc, argv);

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
    spi_read (0xC4,3,fd);
    
	close(fd);

	return ret;
}
