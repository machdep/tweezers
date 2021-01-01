/*-
 * Copyright (c) 2018-2020 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/thread.h>
#include <sys/console.h>

#include <dev/i2c/bitbang/i2c_bitbang.h>

#include <machine/cpuregs.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>

#include <mips/mips/trap.h>
#include <mips/mips/timer.h>

#include <mips/microchip/pic32.h>
#include <mips/microchip/pic32_adc.h>
#include <mips/microchip/pic32mm.h>
#include <mips/microchip/pic32_uart.h>
#include <mips/microchip/pic32_port.h>
#include <mips/microchip/pic32mm_pps.h>
#include <mips/microchip/pic32_intc.h>

PIC32MM_DEVCFG;

/* Software contexts (static allocation). */
static struct pic32_uart_softc uart_sc;
static struct pic32_port_softc port_sc;
static struct pic32_pps_softc pps_sc;
static struct pic32_adc_softc adc_sc;
static struct pic32_ccp_softc ccp_sc;
static struct pic32_intc_softc intc_sc;
static struct i2c_bitbang_softc i2c_bitbang_sc;
static struct mdx_device dev_bitbang = { .sc = &i2c_bitbang_sc };

#define	dprintf(fmt, ...)

#define	C0_COMPARE	11
#define	C0_COUNT	9

struct solder_softc {
	uint8_t button;
	uint8_t enable;
};

struct solder_softc solder_sc;

static void
solder_led_w(struct pic32_port_softc *sc, int enable)
{

	/* Write LED */

	pic32_port_tris(&port_sc, PORT_A, 3, PORT_OUTPUT);
	if (enable)
		pic32_port_lat(&port_sc, PORT_A, 3, 1);
	else
		pic32_port_lat(&port_sc, PORT_A, 3, 0);
}

static void
softintr(void *arg, struct trapframe *frame, int i)
{
	uint32_t cause;

	printf("Soft interrupt %d\n", i);

	cause = mips_rd_cause();
	cause &= ~(1 << (8 + i));
	mips_wr_cause(cause);
};

static void
solder_intr(void *arg, uint32_t cnf)
{
	struct solder_softc *sc;

	sc = &solder_sc;

	dprintf("%s: cnf %x\n", __func__, cnf);

	/* We only expect pin 15 interrupts. */
	if ((cnf & (1 << 15)) == 0)
		return;

	/* b = pic32_port_port(sc, PORT_B, 15); */

	printf("%s: button %d, enable %d\n", __func__, sc->button, sc->enable);

	if (sc->button == 0) {
		sc->button = 1;

		if (sc->enable) {
			sc->enable = 0;
			solder_led_w(&port_sc, 0);
		} else {
			solder_led_w(&port_sc, 1);
			sc->enable = 1;
		}
	} else
		sc->button = 0;
}

static const struct intc_intr_entry intc_intr_map[48] = {
	[_CHANGE_NOTICE_B_VECTOR] = { pic32_port_intr, (void *)&port_sc },
	[_CCP1_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
	[_CCT1_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
#if 0
	[_CCP2_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
	[_CCT2_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
	[_CCP3_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
	[_CCT3_VECTOR] = { pic32_ccp_intr, (void *)&ccp_sc },
#endif
};

static const struct port_intr_entry port_intr_map[48] = {
	[_CHANGE_NOTICE_B_VECTOR] = { PORT_B, solder_intr, (void *)&port_sc },
};

/* This is needed for the i2c bitbang. */
void
udelay(uint32_t usec)
{

	pic32_ccp_delay(&ccp_sc, usec);
}

static void
uart_putchar(int c, void *arg)
{
	struct pic32_uart_softc *sc;

	sc = arg;

	if (c == '\n')
		pic32_putc(sc, '\r');

	pic32_putc(sc, c);
}

static void
pic32_gate(struct pic32_port_softc *sc, uint8_t unit, uint8_t enable)
{

	/* Relay */
	if (unit == 0) {
		pic32_port_ansel(sc, PORT_B, 8, 1);
		pic32_port_tris(sc, PORT_B, 8, PORT_OUTPUT);
		pic32_port_lat(sc, PORT_B, 8, enable);
	} else {
		pic32_port_ansel(sc, PORT_B, 9, 1);
		pic32_port_tris(sc, PORT_B, 9, PORT_OUTPUT);
		pic32_port_lat(sc, PORT_B, 9, enable);
	}
}

static void
i2c_sda(void *arg, bool enable)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (enable)
		pic32_port_tris(sc, PORT_B, 12, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 12, PORT_OUTPUT);
}

static void
i2c_scl(void *arg, bool enable)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (enable)
		pic32_port_tris(sc, PORT_B, 13, PORT_INPUT);
	else
		pic32_port_tris(sc, PORT_B, 13, PORT_OUTPUT);
}

static int
i2c_sda_val(void *arg)
{
	struct pic32_port_softc *sc;

	sc = &port_sc;

	if (pic32_port_port(sc, PORT_B, 12))
		return (1);

	return (0);
}

static int
get_mv(struct pic32_port_softc *sc, uint8_t unit)
{
	struct i2c_msg msgs[1];
	uint8_t data[3];
	int b0, b1, b2;
	int mv;
	int ret;

	if (unit == 0)
		msgs[0].slave = 0x68;
	else
		msgs[0].slave = 0x69;

	msgs[0].buf = data;
	msgs[0].len = 3;
	msgs[0].flags = IIC_M_RD;

	ret = mdx_i2c_transfer(&dev_bitbang, msgs, 1);

	if (ret == 0) {
		b2 = data[0];
		b1 = data[1];
		b0 = data[2];

		printf("%s(%d): read %d %d %02x\n", __func__, unit, b2, b1, b0);
		mv = b2 << 8 | b1;
		return (mv * 1);
	}

	return (-1);
}

static int
get_delay(int mv)
{
	uint32_t val;

	if (mv < 10)
		val = 500000;
	else if (mv < 15)
		val = 300000;
	else if (mv < 17)
		val = 150000;
	else
		val = 100000;

	return (val);
}

static void
tweezers_configure(struct pic32_port_softc *sc)
{
	struct i2c_msg msgs[1];
	uint8_t cfg;
	int ret;

	pic32_port_ansel(sc, PORT_B, 12, 1);
	pic32_port_ansel(sc, PORT_B, 13, 1);

	/*
	 * port input drives 1
	 * port output drives 0
	 */

	pic32_port_ansel(sc, PORT_B, 14, 0);
	pic32_port_tris(sc, PORT_B, 14, PORT_OUTPUT);

	/* Configure the MCP3421A0. */
	cfg = 0x10 | (1 << 2);
	msgs[0].slave = 0x68;
	msgs[0].buf = &cfg;
	msgs[0].len = 1;
	msgs[0].flags =	0;
	ret = mdx_i2c_transfer(&dev_bitbang, msgs, 1);
	if (ret == 0)
		printf("A0 cfg wrotten\n");

	/* Configure the MCP3421A1. */
	msgs[0].slave = 0x69;
	msgs[0].buf = &cfg;
	msgs[0].len = 1;
	msgs[0].flags =	0;
	ret = mdx_i2c_transfer(&dev_bitbang, msgs, 1);
	if (ret == 0)
		printf("A1 cfg wrotten\n");

	pic32_port_ansel(sc, PORT_A, 3, 0);
	pic32_port_tris(sc, PORT_A, 3, PORT_OUTPUT);

	/* Button */
	pic32_port_ansel(sc, PORT_B, 15, 1);
	pic32_port_tris(sc, PORT_B, 15, PORT_INPUT);
	pic32_port_cnpu(sc, PORT_B, 15, 1);

	pic32_port_install_intr_map(sc, port_intr_map);
	pic32_port_cnen(sc, PORT_B, 15, 1, 1);
	pic32_port_cncon(sc, PORT_B, 1, 1);

	pic32_adc_init(&adc_sc, ADC1_BASE);
}

static void
tweezers(struct pic32_port_softc *sc, struct solder_softc *ssc)
{
	uint32_t mv0;
	uint32_t mv1;
	int t0, t1;

	solder_led_w(&port_sc, 0);

	while (1) {
		mdx_usleep(30000);
		if (ssc->enable == 0) {
			pic32_gate(&port_sc, 0, 0);
			pic32_gate(&port_sc, 1, 0);
			continue;
		}

		pic32_port_lat(sc, PORT_B, 15, 1);
		printf("b %d\n", pic32_port_port(sc, PORT_B, 15));

		mv0 = get_mv(sc, 0);
		mv1 = get_mv(sc, 1);

		t0 = 0;
		t1 = 0;

		/* Give power to the both channels. */

		if (mv0 >= 0 && mv0 < 19) {
			pic32_gate(&port_sc, 0, 1);
			t0 = get_delay(mv0);
		}

		if (mv1 >= 0 && mv1 < 19) {
			pic32_gate(&port_sc, 1, 1);
			t1 = get_delay(mv1);
		}

		/* Delay and remove power from both channels. */

		if (t1 > t0) {
			mdx_usleep(t0);
			pic32_gate(&port_sc, 0, 0);
			mdx_usleep(t1 - t0);
			pic32_gate(&port_sc, 1, 0);
		} else {
			mdx_usleep(t1);
			pic32_gate(&port_sc, 1, 0);
			mdx_usleep(t0 - t1);
			pic32_gate(&port_sc, 0, 0);
		}
	}
}

void
board_init(void)
{
	int status;

	mtc0(C0_COUNT, 0, 0);
	mtc0(C0_COMPARE, 0, -1);
	mtc0(12, 0, 1);

	pic32_pps_init(&pps_sc, PPS_BASE);
	pic32_port_init(&port_sc, PORTS_BASE);

	/* UART TX */
	pic32_port_ansel(&port_sc, PORT_A, 0, 1);
	*(volatile uint32_t *)0xBF802590 = (1 << 0); /* rp1 TX */

	/* UART RX */
	pic32_port_ansel(&port_sc, PORT_A, 1, 1);
	pic32_port_tris(&port_sc, PORT_A, 1, PORT_INPUT);
	*(volatile uint32_t *)0xBF802520 = (2 << 16); /* rpinr9 RX RP2 */

	pic32_uart_init(&uart_sc, UART2_BASE, 19200, 8000000, 1);
	mdx_console_register(uart_putchar, (void *)&uart_sc);

	pic32_intc_init(&intc_sc, INTC_BASE);

	mips_wr_ebase(0x9d000000);

	mips_setup_intr(0, softintr, NULL);
	mips_setup_intr(1, softintr, NULL);
	mips_setup_intr(2, pic32_intc_intr, (void *)&intc_sc);
	mips_setup_intr(3, softintr, NULL);
	mips_setup_intr(4, softintr, NULL);
	mips_setup_intr(5, softintr, NULL);
	mips_setup_intr(6, softintr, NULL);
	mips_setup_intr(7, softintr, NULL);
	pic32_intc_install_intr_map(&intc_sc, intc_intr_map);

	status = mips_rd_status();
	status &= ~(MIPS_SR_IM_M);
	status |= MIPS_SR_IM_HARD(0);
	status |= MIPS_SR_IE;
	status &= ~MIPS_SR_BEV;
	status &= ~(MIPS_SR_EXL | MIPS_SR_ERL);
	mips_wr_status(status);

	pic32_intc_enable(&intc_sc, _CHANGE_NOTICE_B_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCP1_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCT1_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCP2_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCT2_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCP3_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _CCT3_VECTOR, 1);
	pic32_intc_enable(&intc_sc, _UART2_TX_VECTOR, 0);
	pic32_intc_enable(&intc_sc, _PERFORMANCE_COUNTER_VECTOR, 0);
}

static struct i2c_bitbang_ops i2c_ops = {
	.i2c_scl = &i2c_scl,
	.i2c_sda = &i2c_sda,
	.i2c_sda_val = &i2c_sda_val,
};

int
main(void)
{
	struct pic32_port_softc *sc;
	struct solder_softc *ssc;

	printf("Hello world\n");

	sc = &port_sc;
	ssc = &solder_sc;
	ssc->button = 0;
	ssc->enable = 0;

	pic32_ccp_init(&ccp_sc, CCP1_BASE, 122000);
	i2c_bitbang_init(&dev_bitbang, &i2c_ops);

	tweezers_configure(sc);
	tweezers(sc, ssc);

	/* NOT REACHED */

	panic("Reached unreachable place");

	return (0);
}
