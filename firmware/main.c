// This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "i2c0.h"
#include "i2c1.h"
#include "LMS64C_protocol.h"
#include "LimeSDR_XTRX.h"
#include "regremap.h"

#define sbi(p, n) ((p) |= (1UL << (n)))
#define cbi(p, n) ((p) &= ~(1 << (n)))

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define LP8758_I2C_ADDR  0x60

/************************** Variable Definitions *****************************/
uint8_t block, cmd_errors, glEp0Buffer_Rx[64], glEp0Buffer_Tx[64];
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet *)glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet *)glEp0Buffer_Rx;


uint8_t lms64_packet_pending;

//#define FW_VER 1 // Initial version
//#define FW_VER 2 // Fix for PLL config. hang when changing from low to high frequency.
#define FW_VER 3 // Added serial number into GET_INFO cmd

/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if(readchar_nonblock()) {
		c[0] = getchar();
		c[1] = 0;
		switch(c[0]) {
			case 0x7f:
			case 0x08:
				if(ptr > 0) {
					ptr--;
					fputs("\x08 \x08", stdout);
				}
				break;
			case 0x07:
				break;
			case '\r':
			case '\n':
				s[ptr] = 0x00;
				fputs("\n", stdout);
				ptr = 0;
				return s;
			default:
				if(ptr >= (sizeof(s) - 1))
					break;
				fputs(c, stdout);
				s[ptr] = c[0];
				ptr++;
				break;
		}
	}

	return NULL;
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if(c == NULL) {
		d = *str;
		*str = *str+strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c+1;
	return d;
}

static void prompt(void)
{
	printf("\e[92;1mlitex-demo-app\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
	puts("\nLiteX minimal demo app built "__DATE__" "__TIME__"\n");
	puts("Available commands:");
	puts("help               - Show this command");
	puts("reboot             - Reboot CPU");
	puts("i2c_test           - Test I2C Buses");
	puts("init_pmic          - Initialize DC-DC switching regulators");
	puts("dump_pmic          - Dump DC-DC switching regulator configuration");
	puts("tst_cntrl          - Test CNTRL register");
#ifdef CSR_LEDS_BASE
	puts("led                - Led demo");
#endif
#ifdef CSR_GPIO_LED_BASE
	puts("gpioled            - GPIO override demo");
#endif
	puts("donut              - Spinning Donut demo");
	puts("helloc             - Hello C");
#ifdef WITH_CXX
	puts("hellocpp           - Hello C++");
#endif
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE
static void led_cmd(void)
{
	int i;
	printf("Led demo...\n");

	printf("Counter mode...\n");
	for(i=0; i<32; i++) {
		leds_out_write(i);
		busy_wait(100);
	}

	printf("Shift mode...\n");
	for(i=0; i<4; i++) {
		leds_out_write(1<<i);
		busy_wait(200);
	}
	for(i=0; i<4; i++) {
		leds_out_write(1<<(3-i));
		busy_wait(200);
	}

	printf("Dance mode...\n");
	for(i=0; i<4; i++) {
		leds_out_write(0x55);
		busy_wait(200);
		leds_out_write(0xaa);
		busy_wait(200);
	}
}
#endif

#ifdef CSR_GPIO_LED_BASE
static void gpioled_cmd(void)
{
	int i;
	printf("GPIO Led override demo...\n");
	gpio_led_control_gpio_override_dir_write(0x0);
	gpio_led_control_gpio_override_write(0x1);
	for(i=0; i<32; i++) {
		gpio_led_control_gpio_override_val_write(0x0);
		busy_wait(100);
		gpio_led_control_gpio_override_val_write(0x1);
		busy_wait(100);
	}

	gpio_led_control_gpio_override_write(0x0);
	printf("GPIO Led override demo...ended\n");
}
#endif


extern void donut(void);

static void donut_cmd(void)
{
	printf("Donut demo...\n");
	donut();
}

extern void helloc(void);

static void helloc_cmd(void)
{
	printf("Hello C demo...\n");
	helloc();
}

#ifdef WITH_CXX
extern void hellocpp(void);

static void hellocpp_cmd(void)
{
	printf("Hello C++ demo...\n");
	hellocpp();
}
#endif

/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/

static void i2c_test(void)
{
	printf("I2C0 Scan...\n");
	i2c0_scan();

	printf("\n");

	printf("I2C1 Scan...\n");
	i2c1_scan();
}

static void dump_pmic(void){
	unsigned char adr;
	unsigned char dat;
	printf("FPGA_I2C1 PMIC Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
	printf("FPGA_I2C2 PMIC Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
}

static void init_pmic(void)
{
	printf("Initializing DC-DC switching regulators...\n");

	unsigned char adr;
	unsigned char dat;

	printf("PMICs Initialization...\n");
	printf("-----------------------\n");

	printf("FPGA_I2C1 PMIC: Check ID ");
	adr = 0x01;
	i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
	} else {
		printf("OK.\n");

		printf("PMIC: Enable Buck0.\n");
		adr = 0x02;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
		adr = 0x03;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck1.\n");
		adr = 0x04;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
		adr = 0x05;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck2.\n");
		adr = 0x06;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
		adr = 0x07;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck3.\n");
		adr = 0x08;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
		adr = 0x09;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck1 to 3.3V.\n");
		adr = 0x0C;
		dat = 0xFC;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		busy_wait(1);
	}


	printf("FPGA_I2C2 PMIC: Check ID ");
	adr = 0x01;
	i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
	} else {
		printf("OK.\n");

		printf("PMIC: Enable Buck0.\n");
		adr = 0x02;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
		adr = 0x03;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck1.\n");
		adr = 0x04;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
		adr = 0x05;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck2.\n");
		adr = 0x06;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
		adr = 0x07;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck3.\n");
		adr = 0x08;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
		adr = 0x09;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck0 to 1.5V.\n");
		adr = 0x0A;
		dat = 0xA2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck1 to 3.3V.\n");
		adr = 0x0C;
		dat = 0xFC;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck2 to 1.75V.\n");
		adr = 0x0E;
		dat = 0xAF;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck3 to 2.05V.\n");
		adr = 0x10;
		dat = 0xBE;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Clear INT_BUCK_2_3 Status.\n");
		adr = 0x1A;
		dat = 0xFF;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		busy_wait(1);
	}

}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char *str;
	char *token;

	str = readstr();
	if(str == NULL) return;
	token = get_token(&str);
	if(strcmp(token, "help") == 0)
		help();
	else if(strcmp(token, "reboot") == 0)
		reboot_cmd();
	else if(strcmp(token, "i2c_test") == 0)
		i2c_test();
	else if(strcmp(token, "init_pmic") == 0)
		init_pmic();
	else if(strcmp(token, "dump_pmic") == 0)
		dump_pmic();
	else if(strcmp(token, "tst_cntrl") == 0)
		tst_cntrl();
#ifdef CSR_LEDS_BASE
	else if(strcmp(token, "led") == 0)
		led_cmd();
#endif
#ifdef CSR_GPIO_LED_BASE
	else if(strcmp(token, "gpioled") == 0)
		gpioled_cmd();
#endif
	else if(strcmp(token, "donut") == 0)
		donut_cmd();
	else if(strcmp(token, "helloc") == 0)
		helloc_cmd();
#ifdef WITH_CXX
	else if(strcmp(token, "hellocpp") == 0)
		hellocpp_cmd();
#endif
	prompt();
}

/**	This function checks if all blocks could fit in data field.
 *	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks(unsigned char block_size)
{
	if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field) / block_size))
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
		return 1;
	}
	else
		return 0;
	return 1;
}

/**
 * Gets 64 bytes packet
 */
void getLMS64Packet(uint8_t *buf, uint8_t k)
{
	uint8_t cnt = 0;
	uint32_t *dest = (uint32_t *)buf;
	for (cnt = 0; cnt < k / sizeof(uint32_t); cnt++)
	{
		dest[cnt] = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt*4));
	};

}

void tst_cntrl(void){
	uint32_t val;
	long int addr;
	printf("Write to CSR_CNTRL_CNTRL_ADDR begin.\n");

	val = 0;
	for (long int i=0; i <16; i++) {
		printf("0x%08lx: 0x%08lx: \n", (CSR_CNTRL_CNTRL_ADDR + i*4), val);
		csr_write_simple(val, (CSR_CNTRL_CNTRL_ADDR + i*4));
		busy_wait(5);

		val++;
	}
}


void lms64c_isr(void){
	uint32_t *dest = (uint32_t *)glEp0Buffer_Tx;
	uint32_t read_value;

	uint8_t reg_array[4];


	//printf("ISR: LMS64C Entry\n");

	lms64_packet_pending = 1;
	getLMS64Packet(glEp0Buffer_Rx, 64);

	memset(glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); // fill whole tx buffer with zeros
	cmd_errors = 0;

	LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
	LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
	LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
	LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

	switch (LMS_Ctrl_Packet_Rx->Header.Command)
	{
	case CMD_GET_INFO:

		LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
		LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
		LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
		LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
		LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

	case CMD_BRDSPI16_WR:
		if (Check_many_blocks(4))
			break;

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{
			// write reg addr
			//sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit
			// Clearing write bit in address field because we are not using SPI registers here
			cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit

			writeCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], &LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]);
		}
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

	case CMD_BRDSPI16_RD:
		if (Check_many_blocks(4))
			break;

		for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
		{

			// write reg addr
			cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit

			readCSR(&LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], reg_array);
			LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = reg_array[1];
			LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = reg_array[0];

			printf("value: 0x%X\n", reg_array[0]);
			printf("value: 0x%X\n", reg_array[1]);

		}

		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
		break;

	default:
		/* This is unknown request. */
		// isHandled = CyFalse;
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
		break;
	};

	// Send response to the command
	for (int i = 0; i < 64 / sizeof(uint32_t); ++i)
	{
		csr_write_simple(dest[i], (CSR_CNTRL_CNTRL_ADDR + i*4));
	}

	CNTRL_ev_pending_write(1);  //Clear interrupt
	CNTRL_ev_enable_write(1);   // re-enable the event handler
	//printf("ISR: LMS64C exit\n");

}

static void lms64c_init(void){
	uint32_t irq_mask;
	printf("CNTRL IRQ initialization \n");

	CNTRL_ev_pending_write(CNTRL_ev_pending_read());
	irq_setmask(irq_getmask() | (1 << CNTRL_INTERRUPT));
	CNTRL_ev_enable_write(1);

	irq_mask = irq_getmask();
	printf("0x%08lx:\n", irq_mask);
	irq_attach(CNTRL_INTERRUPT, lms64c_isr);

}

int main(void)
{

#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();
	lms64c_init();


	help();
	prompt();

	while(1) {
		console_service();

		if (lms64_packet_pending == 1) {
			printf("Response to LMS64 packet\n");
			for (long int i=0; i <16; i++) {
				printf("0x%08lx: 0x%08lx: \n", (CSR_CNTRL_CNTRL_ADDR + i*4), csr_read_simple((CSR_CNTRL_CNTRL_ADDR + i*4)));
			}
				lms64_packet_pending = 0;

		}
	}

	return 0;
}
