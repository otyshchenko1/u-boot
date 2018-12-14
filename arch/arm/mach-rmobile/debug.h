// SPDX-License-Identifier: GPL-2.0+
/*
 * Contains functions used for PSCI debug.
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Based on arch/arm/mach-uniphier/debug.h
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <asm/io.h>

/* SCIFA definitions */
#define SCIFA_BASE		0xe6c40000

#define SCIFA_SCASSR	0x14	/* Serial status register */
#define SCIFA_SCAFTDR	0x20	/* Transmit FIFO data register */

/* SCIF definitions */
#define SCIF_BASE		0xe6e60000

#define SCIF_SCFSR		0x10	/* Serial status register */
#define SCIF_SCFTDR		0x0c	/* Transmit FIFO data register */

/* Common for both interfaces definitions */
#define SCFSR_TDFE		(1 << 5) /* Transmit FIFO Data Empty */
#define SCFSR_TEND		(1 << 6) /* Transmission End */

#ifdef CONFIG_SCIF_A
#define UART_BASE			SCIFA_BASE
#define UART_STATUS_REG		SCIFA_SCASSR
#define UART_TX_FIFO_REG	SCIFA_SCAFTDR
#else
#define UART_BASE			SCIF_BASE
#define UART_STATUS_REG		SCIF_SCFSR
#define UART_TX_FIFO_REG	SCIF_SCFTDR
#endif

/* All functions are inline so that they can be called from .secure section. */

#ifdef DEBUG
static inline void debug_putc(int c)
{
	void __iomem *base = (void __iomem *)UART_BASE;

	while (!(readw(base + UART_STATUS_REG) & SCFSR_TDFE))
		;

	writeb(c, base + UART_TX_FIFO_REG);
	writew(readw(base + UART_STATUS_REG) & ~(SCFSR_TEND | SCFSR_TDFE),
			base + UART_STATUS_REG);
}

static inline void debug_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			debug_putc('\r');

		debug_putc(*s++);
	}
}

static inline void debug_puth(unsigned long val)
{
	int i;
	unsigned char c;

	for (i = 8; i--; ) {
		c = ((val >> (i * 4)) & 0xf);
		c += (c >= 10) ? 'a' - 10 : '0';
		debug_putc(c);
	}
}
#else
static inline void debug_putc(int c)
{
}

static inline void debug_puts(const char *s)
{
}

static inline void debug_puth(unsigned long val)
{
}
#endif

#endif /* __DEBUG_H__ */
