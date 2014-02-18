/*
 * Copyright (C) 2013 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef __BBSI_H
#define __BBSI_H

#include <linux/netdevice.h>
#include <linux/spi/spi.h>

#ifndef KSEG1
#define KSEG1 0  // just to appease non-MIPS CPUs. Not really used.
#endif

#define BP_MOCA_MAX_NUM 1

/*
 * The exact values here don't matter, as they're translated into "real"
 * values before talking to mocad.  This is just for the device registration
 * tables.
 */
enum {
	BP_MOCA_TYPE_WAN,
	BP_MOCA_TYPE_LAN,
};

enum {
	BP_MOCA_RF_BAND_D_LOW,
	BP_MOCA_RF_BAND_D_HIGH,
	BP_MOCA_RF_BAND_EXT_D,
	BP_MOCA_RF_BAND_E,
	BP_MOCA_RF_BAND_F,
};

typedef struct BpMocaInfo {
	int type;
	int rfBand;
} BP_MOCA_INFO;

static void BpGetMocaInfo(BP_MOCA_INFO *chips, int *nchips) {
	if (*nchips >= 1) {
		*nchips = 1;
		chips[0].type = BP_MOCA_TYPE_LAN;
		chips[0].rfBand = BP_MOCA_RF_BAND_E;
	}
}

static uint32_t _spi_read32(struct spi_device *spi, uint32_t addr);


// TODO(apenwarr): don't make this global.
//   Or fix the driver to just only enable/disable interrupts at the right
//   times.
static int irq_disabled = 0;

static void kerSysMocaHostIntrEnable(struct spi_device *spi) {
	if (irq_disabled == 1) {
		irq_disabled = 0;
		enable_irq(spi->irq);
	}
}

static void kerSysMocaHostIntrDisable(struct spi_device *spi) {
	if (irq_disabled == 0) {
		disable_irq_nosync(spi->irq);
		irq_disabled = 1;
	}
}

static uint8_t __pollstatus(struct spi_device *spi) {
	uint8_t wclear[] = { 0x80, 0x06 };
	uint8_t rdata[1] = { 0 };
	struct spi_transfer t[2] = {
		{ .tx_buf = wclear, .len = sizeof(wclear) },
		{ .rx_buf = rdata, .len = sizeof(rdata) },
	};
	struct spi_message m;
	int i;

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	for (i = 0; i < 10; i++) {
		if (spi_sync_locked(spi, &m) < 0) {
			pr_warn("spi _pollstatus: SPI error\n");
			return 0x01; // error code
		}
		if (rdata[0] & 0x01) {
			pr_warn("spi _pollstatus: rbus error: %02X\n", rdata[0]);
			return 0x01; // error result; stop polling now
		}
		if (!(rdata[0] & 0x10)) return 0;   // transaction finished
	}
	// if we get here, the transaction still isn't finished: weird
	pr_warn("spi _pollstatus: still busy: %02X\n", rdata[0]);
	return rdata[0];
}

static uint32_t __spi_read32a(struct spi_device *spi, uint32_t addr,
				int speculative) {
	uint8_t waddr[] = {
		0x81, 0x07,
		0x01 | (speculative ? 0x02 : 0),
		0, 0, 0, 0 };
	struct spi_transfer addrt[1] = {
		{ .tx_buf = waddr, .len = sizeof(waddr) },
	};
	struct spi_message addrm;
	int j, st;

	spi_message_init(&addrm);
	spi_message_add_tail(&addrt[0], &addrm);

	__pollstatus(spi);
	for (j = 0; j < 10; j++) {
		// write address reg, which triggers the read
		writel(cpu_to_be32(addr), waddr + sizeof(waddr) - 4);
		if (spi_sync_locked(spi, &addrm) < 0) {
			pr_warn("spi_read_addr: error\n");
		}
		st = __pollstatus(spi);
		if (!st) break;
	}
	return st;
}

static uint32_t __spi_read32d_noswap(struct spi_device *spi) {
	uint8_t wdata[] = { 0x80, 0x0c };
	uint8_t rdata[4];
	struct spi_transfer datat[2] = {
		{ .tx_buf = wdata, .len = sizeof(wdata) },
		{ .rx_buf = rdata, .len = sizeof(rdata) },
	};
	struct spi_message datam;

	spi_message_init(&datam);
	spi_message_add_tail(&datat[0], &datam);
	spi_message_add_tail(&datat[1], &datam);

	// retrieve actual data bits
	if (spi_sync_locked(spi, &datam) < 0) {
		pr_warn("spi_read_data: error\n");
	}
	return readl(rdata);
}

static uint32_t _spi_read32(struct spi_device *spi, uint32_t addr) {
	int st;
	uint32_t retval;

	spi_bus_lock(spi->master);

	st = __spi_read32a(spi, addr, 0);
	if (st) {
		retval = 0x00000000; // error
	} else {
		retval = be32_to_cpu(__spi_read32d_noswap(spi));
	}
	spi_bus_unlock(spi->master);
	return retval;
}

static void __spi_write32a(struct spi_device *spi, uint32_t addr) {
	uint8_t waddr[] = { 0x81, 0x07, 0x00, 0, 0, 0, 0  };
	struct spi_transfer t[1] = {
		{ .tx_buf = waddr, .len = sizeof(waddr) },
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);

	// write address reg
	writel(cpu_to_be32(addr), waddr + sizeof(waddr) - 4);
	if (spi_sync_locked(spi, &m) < 0) {
		pr_warn("spi_write: error\n");
	}
}

static void __spi_write32d_noswap(struct spi_device *spi, uint32_t value) {
	uint8_t wdata[] = { 0x81, 0x0c, 0, 0, 0, 0 };
	struct spi_transfer t[1] = {
		{ .tx_buf = wdata, .len = sizeof(wdata) },
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);

	// write data reg
	writel(value, wdata + sizeof(wdata) - 4);
	if (spi_sync_locked(spi, &m) < 0) {
		pr_warn("spi_write: error\n");
	}
}


static void _spi_write32(struct spi_device *spi, uint32_t addr, uint32_t value) {
	spi_bus_lock(spi->master);
	__pollstatus(spi);
	__spi_write32a(spi, addr);
	__spi_write32d_noswap(spi, cpu_to_be32(value));
	__pollstatus(spi);
	spi_bus_unlock(spi->master);
}

static uint32_t kerSysBcmSpiSlaveReadReg32(struct spi_device *spi, uint32_t addr) {
	return _spi_read32(spi, addr);
}

static void kerSysBcmSpiSlaveWriteReg32(struct spi_device *spi, uint32_t addr, uint32_t value) {
	_spi_write32(spi, addr, value);
}

static void kerSysBcmSpiSlaveReadBuf(struct spi_device *spi, uint32_t addr, void *dst, int len, int wordsize) {
	int i;
	uint32_t *buf = dst;

	spi_bus_lock(spi->master);

	if (wordsize != 4) {
		pr_info("SPI readbuf: only word size == 4 bytes is supported!\n");
		return;
	}
	__spi_read32a(spi, addr, 1);
	for (i = 0; i < len; i += wordsize) {
		buf[i/4] = __spi_read32d_noswap(spi);
		__pollstatus(spi);
	}

	spi_bus_unlock(spi->master);
}

static void kerSysBcmSpiSlaveWriteBuf(struct spi_device *spi, uint32_t addr, const void *src, int len, int wordsize) {
	int i, nelems = len/4;
	const uint32_t *buf = src;
	uint8_t wdata[] = { 0x81, 0x0c };
	struct spi_transfer *t, *tp;
	struct spi_message m;

	if (len > 8192) {
		pr_warn("spi writebuf: buffer size %d is too large\n", len);
		return;
	}
	if (wordsize != 4) {
		pr_err("SPI writebuf: only word size == 4 bytes is supported!\n");
		return;
	}

	t = kmalloc(nelems * sizeof(struct spi_transfer) * 2, GFP_KERNEL);
	if (!t) {
		pr_warn("spi writebuf: out of memory\n");
		return;
	}

	memset(t, 0, nelems * sizeof(struct spi_transfer) * 2);
	spi_message_init(&m);

	for (i = 0, tp = t; i < nelems; i++) {
		tp->tx_buf = wdata;
		tp->len = sizeof(wdata);
		spi_message_add_tail(tp, &m);
		tp++;

		tp->tx_buf = &buf[i];
		tp->len = 4;
		tp->cs_change = 1;
		spi_message_add_tail(tp, &m);
		tp++;
	}

	spi_bus_lock(spi->master);

	__pollstatus(spi);
	writel(cpu_to_be32(addr), wdata + 2);
	__spi_write32a(spi, addr);
	spi_sync_locked(spi, &m);
	__pollstatus(spi);

	spi_bus_unlock(spi->master);
	kfree(t);
}

#endif // __BBSI_H
