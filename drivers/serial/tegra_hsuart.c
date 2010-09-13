/*
 * drivers/serial/tegra_hsuart.c
 *
 * High-speed serial driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*#define DEBUG           1*/
/*#define VERBOSE_DEBUG   1*/

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/termios.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <mach/dma.h>
#include <mach/pinmux.h>
#include <mach/serial.h>
#include "nvodm_uart.h"
#include <mach/clk.h>

#define TX_EMPTY_STATUS (UART_LSR_TEMT | UART_LSR_THRE)

/* Size of the rx buffer in continuous mode */
#define UART_RX_DMA_BUFFER_SIZE    (32*1024*2)

#define UART_LSR_FIFOE		0x80
#define UART_IER_EORD		0x20
#define UART_MCR_RTS_EN		0x40
#define UART_MCR_CTS_EN		0x20
#define UART_LSR_ANY		(UART_LSR_OE | UART_LSR_BI | \
				UART_LSR_PE | UART_LSR_FE)

#define TX_FORCE_PIO 0
#define RX_FORCE_PIO 0

const int dma_req_sel[] = {
	TEGRA_DMA_REQ_SEL_UARTA,
	TEGRA_DMA_REQ_SEL_UARTB,
	TEGRA_DMA_REQ_SEL_UARTC,
	TEGRA_DMA_REQ_SEL_UARTD,
	TEGRA_DMA_REQ_SEL_UARTE,
};

#define TEGRA_TX_PIO			1
#define TEGRA_TX_DMA			2
#define TEGRA_TX_RUN			3

/* Time taken by dma to update the status after reading fifo and
 * complete the current burst */
#define WAIT_TIME_FOR_DMA_STATUS_UPDATE_US   30
#define TEGRA_UART_MIN_DMA		     16
#define TEGRA_UART_FIFO_TX_PIO_TRIGGER_COUNT 8
#define TEGRA_UART_FIFO_TX_PIO_TRIGGER_LEVEL UART_FCR_T_TRIG_10

#define TEGRA_UART_CLOSED    0
#define TEGRA_UART_OPENED    1
#define TEGRA_UART_CLOCK_OFF 2
#define TEGRA_UART_SUSPEND   3

struct tegra_uart_port {
	struct uart_port        uport;
	char                    port_name[32];

	/* Module info */
	struct clk              *clk;
	unsigned int            baud;

	/* Register shadow */
	unsigned char           fcr_shadow;
	unsigned char           mcr_shadow;
	unsigned char           lcr_shadow;
	unsigned char           ier_shadow;
	bool                    use_cts_control;
	bool                    rts_active;

	/* Tx DMA info */
	void                   *tx_dma_virt;
	dma_addr_t              tx_dma_phys;
	int                     tx_dma_size;
	const struct tegra_pingroup_config *pinmux;
	int                     nr_pins;

	/* Rm DMA handles */
	struct tegra_dma_req      tx_dma_req;
	struct tegra_dma_channel *tx_dma;

	/* DMA requests */
	struct tegra_dma_req      rx_dma_req;
	struct tegra_dma_channel *rx_dma;

	bool                    use_rx_dma;
	bool                    use_tx_dma;

	int uart_state;
	NvOdmUartHandle         odm_uart_handle;

	int                     tx_in_progress;
	int                     rx_in_progress;
	struct work_struct      rx_work;
	struct workqueue_struct *rx_work_queue;
	struct work_struct      tx_work;
	struct workqueue_struct *tx_work_queue;
	int                     last_read_index;
	int                     already_read_bytecount;
	int                     last_transfer_count;

	spinlock_t              reg_access_lock;
};

#define uart_readb(t, reg) \
		readb(t->uport.membase + (reg << t->uport.regshift))
#define uart_writeb(val, t, reg) \
		writeb(val, t->uport.membase + (reg << t->uport.regshift))
#define uart_writel(val, t, reg) \
		writel(val, t->uport.membase + (reg << t->uport.regshift))

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud);
static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl);
static void set_rts(struct tegra_uart_port *t, bool active);
static void set_dtr(struct tegra_uart_port *t, bool active);
static void tegra_tx_dma_workqueue(struct work_struct *w)
{
	struct uart_port *u;
	struct tegra_uart_port *t;
	struct circ_buf *xmit;
	int to_end;
	unsigned long irq_flags;
	int pending;
	int data_size_to_dma;

	t = container_of(w, struct tegra_uart_port, tx_work);
	u = &t->uport;
	xmit = &u->state->xmit;

	if (!t->tx_in_progress) {
		dev_err(u->dev, "Tx is in stop condition\n");
		return;
	}

	if (t->tx_in_progress != TEGRA_TX_RUN) {
		dev_err(u->dev, "BUG(): Why tx state is not in run state %d\n",
			t->tx_in_progress);
		BUG();
		return;
	}

	if (uart_circ_empty(xmit)) {
		uart_write_wakeup(u);
		t->tx_in_progress = 0;
		return;
	}

	pending = uart_circ_chars_pending(xmit);
	if ((t->use_tx_dma) && (pending > TEGRA_UART_MIN_DMA)) {
		data_size_to_dma = (pending < t->tx_dma_size)?
						pending : t->tx_dma_size;
		data_size_to_dma &= ~3;
		dev_dbg(u->dev, "Tx DMA starting 0x%x size %d\n",
						xmit->tail, data_size_to_dma);

		to_end = CIRC_CNT_TO_END(xmit->head, xmit->tail,
							UART_XMIT_SIZE);
		if (to_end < data_size_to_dma) {
			char *tx_dma_virt_buff = (char *)t->tx_dma_virt;
			memcpy(tx_dma_virt_buff, xmit->buf + xmit->tail, to_end);
			memcpy(tx_dma_virt_buff + to_end, xmit->buf,
						data_size_to_dma - to_end);
		} else {
			memcpy(t->tx_dma_virt, xmit->buf + xmit->tail,
						data_size_to_dma);
		}
		dsb();
		outer_sync();

		t->tx_dma_req.source_addr = t->tx_dma_phys;
		t->tx_dma_req.size = data_size_to_dma;

		spin_lock_irqsave(&t->uport.lock, irq_flags);
		t->fcr_shadow &= ~UART_FCR_T_TRIG_11;
		t->fcr_shadow |= UART_FCR_T_TRIG_01;
		uart_writeb(t->fcr_shadow, t, UART_FCR);

		if (t->tx_in_progress) {
			t->tx_in_progress = TEGRA_TX_DMA;
			tegra_dma_enqueue_req(t->tx_dma, &t->tx_dma_req);
		} else {
			dev_err(u->dev, "Tx has been stopped\n");
		}
		spin_unlock_irqrestore(&t->uport.lock, irq_flags);
	} else {
		/* Transfer in PIO mode */
		spin_lock_irqsave(&t->uport.lock, irq_flags);
		t->fcr_shadow &= ~UART_FCR_T_TRIG_11;
		t->fcr_shadow |= TEGRA_UART_FIFO_TX_PIO_TRIGGER_LEVEL;
		uart_writeb(t->fcr_shadow, t, UART_FCR);

		t->tx_in_progress = TEGRA_TX_PIO;
		t->ier_shadow |= UART_IER_THRI;
		uart_writeb(t->ier_shadow, t, UART_IER);
		spin_unlock_irqrestore(&t->uport.lock, irq_flags);
	}
	return;
}

static void tegra_tx_dma_complete_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &t->uport.state->xmit;

	dev_dbg(t->uport.dev, "%s: %d\n", __func__, req->bytes_transferred);

	if (req->status == -TEGRA_DMA_REQ_ERROR_ABORTED)
		return;

	xmit->tail += t->tx_dma_req.size;
	xmit->tail &= (UART_XMIT_SIZE - 1);
	u->icount.tx += t->tx_dma_req.size;
	t->tx_dma_req.size = 0;
	t->tx_in_progress = TEGRA_TX_RUN;
	queue_work(t->tx_work_queue, &t->tx_work);
}

static void do_handle_tx_pio(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	struct circ_buf *xmit = &t->uport.state->xmit;
	int count = 0;
	int pending;

	if (!t->tx_in_progress) {
		/* Disable interrupts on FIFO empty and break */
		t->ier_shadow &= ~UART_IER_THRI;
		uart_writeb(t->ier_shadow, t, UART_IER);
		return;
	}

	/* As long as there is room in the FIFO write the buffer without
	 * polling fifo status register */
	pending = uart_circ_chars_pending(xmit);
	count = min(TEGRA_UART_FIFO_TX_PIO_TRIGGER_COUNT, pending);
	while (count) {
		uart_writeb(xmit->buf[xmit->tail], t, UART_TX);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		u->icount.tx++;
		count--;
	}

	pending = uart_circ_chars_pending(xmit);
	if ((t->use_tx_dma) && (pending > TEGRA_UART_MIN_DMA)) {
		t->ier_shadow &= ~UART_IER_THRI;
		uart_writeb(t->ier_shadow, t, UART_IER);
		t->tx_in_progress = TEGRA_TX_RUN;
		queue_work(t->tx_work_queue, &t->tx_work);
		return;
	} else {
		if (!pending) {
			t->ier_shadow &= ~UART_IER_THRI;
			uart_writeb(t->ier_shadow, t, UART_IER);
			t->tx_in_progress = 0;
		}
	}

	if (pending < WAKEUP_CHARS)
		uart_write_wakeup(u);
	return;
}

static void tegra_start_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);

	dev_vdbg(t->uport.dev, "+tegra_start_tx\n");

	if (t->tx_in_progress) {
		return;
	}
	t->tx_in_progress = TEGRA_TX_RUN;

	/* Enable Tx interrupt for the non dma mode transmit other schedule
	 * tx work queue*/
	if (!t->use_tx_dma) {
		t->tx_in_progress = TEGRA_TX_PIO;
		t->ier_shadow |= UART_IER_THRI;
		uart_writeb(t->ier_shadow, t, UART_IER);
	} else {
		queue_work(t->tx_work_queue, &t->tx_work);
	}
	dev_vdbg(t->uport.dev, "-tegra_start_tx\n");
}

static void tegra_stop_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;
	int tx_state;
	t = container_of(u, struct tegra_uart_port, uport);

	if (t->tx_in_progress) {
		tx_state = t->tx_in_progress;
		t->tx_in_progress = 0;
		if (tx_state == TEGRA_TX_PIO) {
			t->ier_shadow &= ~UART_IER_THRI;
			uart_writeb(t->ier_shadow, t, UART_IER);
		}
		if (tx_state == TEGRA_TX_DMA) {
			tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req);
			t->tx_dma_req.size = 0;
		}
	}
}


static  bool tegra_uart_isbreak(struct tegra_uart_port *t)
{
	unsigned char lsr;
	unsigned char rx_char;
	bool is_break_detected = false;

	lsr = uart_readb(t, UART_LSR);
	if (lsr & UART_LSR_BI) {
		is_break_detected = true;

		/* Again read the LSR and if there is fifo error with data then
		 * reset here */
		lsr = uart_readb(t, UART_LSR);
		if (lsr & UART_LSR_FIFOE) {
			if (lsr & UART_LSR_DR)
				return is_break_detected;

			/* Fifo error without rx data, read fifo till rx fifo
			 * empty: hw issue to detect next break */
			do {
				lsr = uart_readb(t, UART_LSR);
				if (!(lsr & UART_LSR_DR))
					break;
				rx_char = uart_readb(t, UART_TX);
			} while(1);
		}
	}
	return is_break_detected;
}

static char do_decode_rx_error(struct tegra_uart_port *t, u8 lsr)
{
	char flag = TTY_NORMAL;
	unsigned char rx_char;

	if (unlikely(lsr & UART_LSR_ANY)) {
		if (lsr & UART_LSR_OE) {
			/* Overrrun error  */
			flag |= TTY_OVERRUN;
			t->uport.icount.overrun++;
			dev_err(t->uport.dev, "Got overrun errors\n");
		} else if (lsr & UART_LSR_PE) {
			/* Parity error */
			flag |= TTY_PARITY;
			t->uport.icount.parity++;
			dev_err(t->uport.dev, "Got Parity errors\n");
		} else if (lsr & UART_LSR_FE) {
			flag |= TTY_FRAME;
			dev_err(t->uport.dev, "Got frame errors\n");
		} else if (lsr & UART_LSR_BI) {
			flag |= TTY_BREAK;
			dev_err(t->uport.dev, "Got Break\n");
			lsr = uart_readb(t, UART_LSR);
			if (lsr & UART_LSR_FIFOE) {
				if (lsr & UART_LSR_DR)
					return flag;
				/* Fifo error without rx data, read fifo till rx fifo
				 * empty: hw issue to detect next break */
				do {
					lsr = uart_readb(t, UART_LSR);
					if (!(lsr & UART_LSR_DR))
						break;
					rx_char = uart_readb(t, UART_TX);
				} while(1);
			}
		}
	}
	return flag;
}

static int copy_dma_buffer_to_tty_buffer(struct tegra_uart_port *t,
			int new_trans_count)
{
	int copied_count;
	unsigned char *dma_virt_buf = (char *)t->rx_dma_req.virt_addr;
	struct uart_port *u = &t->uport;
	int ret_copied;

	if (new_trans_count < t->last_transfer_count) {
		/* dma buffer roundoff */
		copied_count = UART_RX_DMA_BUFFER_SIZE - t->last_transfer_count;
		ret_copied = tty_insert_flip_string(u->state->port.tty,
					dma_virt_buf + t->last_read_index,
					copied_count);
		if (copied_count != ret_copied) {
			dev_err(u->dev, "dma_to_tty lost data(1): Trying"
					" %x got %x lost %x\n",
					copied_count,ret_copied,
					copied_count - ret_copied);
		}
		if (new_trans_count) {
			copied_count += new_trans_count;
			ret_copied = tty_insert_flip_string(u->state->port.tty,
					dma_virt_buf, new_trans_count);
			if (new_trans_count != ret_copied) {
				dev_err(u->dev, "dma_to_tty lost data(2):Trying"
						" %x got %x lost %x\n",
						copied_count,ret_copied,
						copied_count - ret_copied);
			}
		}
		t->uport.icount.rx += copied_count;
		t->last_read_index = new_trans_count;
		t->last_transfer_count = new_trans_count;
	} else {
		copied_count = new_trans_count - t->last_transfer_count;
		if (copied_count) {
			ret_copied = tty_insert_flip_string(u->state->port.tty,
					dma_virt_buf + t->last_read_index,
					copied_count);
			if (copied_count != ret_copied) {
				dev_err(u->dev, "Lost some data last data(3):"
						" Trying %x got %x lost %x\n",
					copied_count,ret_copied,
					copied_count - ret_copied);
			}
			t->uport.icount.rx += copied_count;
			t->last_read_index += copied_count;
			t->last_transfer_count = new_trans_count;
		}
	}
	dev_dbg(u->dev, "Received %d bytes\n", copied_count);
	return copied_count;
}

static int tegra_start_dma_rx(struct tegra_uart_port *t)
{
	t->rx_dma_req.bytes_transferred = 0;
	if (tegra_dma_enqueue_req(t->rx_dma, &t->rx_dma_req)) {
		dev_err(t->uport.dev, "Could not enqueue Rx DMA req\n");
		return -EINVAL;
	}
	return 0;
}

static int do_handle_rx_pio(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	struct tty_struct *tty = u->state->port.tty;
	int count = 0;

	do {
		char flag = TTY_NORMAL;
		unsigned char lsr = 0;
		unsigned char ch;

		lsr = uart_readb(t, UART_LSR);
		if (!(lsr & UART_LSR_DR))
			break;

		flag = do_decode_rx_error(t, lsr);
		ch = uart_readb(t, UART_TX);
		dev_vdbg(u->dev, "%c\n", ch);

		if (!uart_handle_sysrq_char(u, c)) {
			tty_insert_flip_char(tty, ch, flag);
			t->uport.icount.rx++;
			count++;
		}
	} while (1);

	dev_dbg(t->uport.dev, "PIO received %d bytes\n", count);

	return count;
}

static void tegra_rx_dma_workqueue(struct work_struct *w)
{
	struct tegra_uart_port *t;
	unsigned char lsr;
	bool is_dma_stopped = false;
	int dma_trans_count;
	int dma_read_count = 0;
	int pio_read_count = 0;
	int start_status;
	unsigned long irq_flags;
	struct uart_port *u;
	unsigned char ier;
	bool is_break_detected = false;

	t = container_of(w, struct tegra_uart_port, rx_work);
	u = &t->uport;

	if (!t->rx_in_progress) {
		dev_dbg(t->uport.dev, "The Rx is stopped\n");
		return;
	}

	is_break_detected = tegra_uart_isbreak(t);
	lsr = uart_readb(t, UART_LSR);
	if (lsr & UART_LSR_DR) {
		/* Data available in fifo */
		if (t->rts_active)
			set_rts(t, false);
		/* Wait for dma to update status on current burst */
		udelay(WAIT_TIME_FOR_DMA_STATUS_UPDATE_US);

		is_dma_stopped = true;
		dma_trans_count = tegra_dma_get_transfer_count(t->rx_dma,
					&t->rx_dma_req, true);
		if (dma_trans_count < 0) {
			goto end;
		}

		dma_read_count = copy_dma_buffer_to_tty_buffer(t, dma_trans_count);
		pio_read_count = do_handle_rx_pio(t);
		t->last_read_index = 0;
		t->already_read_bytecount = 0;
		t->last_transfer_count = 0;
		start_status = tegra_dma_start_dma(t->rx_dma, &t->rx_dma_req);
		if (start_status < 0) {
			goto end;
		}
		/* enable the rts now */
		if (t->rts_active)
			set_rts(t, true);
	} else {
		is_dma_stopped = false;
		dma_trans_count = tegra_dma_get_transfer_count(t->rx_dma,
					&t->rx_dma_req, false);
		if (dma_trans_count < 0) {
			goto end;
		}
		dma_read_count = copy_dma_buffer_to_tty_buffer(t, dma_trans_count);
	}

	if (is_break_detected){
		tty_insert_flip_char(u->state->port.tty, 0, TTY_BREAK);
		t->uport.icount.rx++;
	}

	if (dma_read_count || pio_read_count) {
		tty_flip_buffer_push(u->state->port.tty);
	}

	spin_lock_irqsave(&t->reg_access_lock, irq_flags);
	/* Enable the interrupts */
	if (t->rx_in_progress) {
		ier = t->ier_shadow;
		ier |= (UART_IER_RLSI | UART_IER_RTOIE | UART_IER_EORD);
		t->ier_shadow = ier;
		uart_writeb(ier, t, UART_IER);
	}
	spin_unlock_irqrestore(&t->reg_access_lock, irq_flags);

end:
	return;
}

static void tegra_rx_dma_threshold_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	queue_work(t->rx_work_queue, &t->rx_work);
}

/* It is expected that the callers take the UART lock when this API is called.
 *
 * There are 2 contexts when this function is called:
 *
 * 1. DMA ISR - DMA ISR triggers the threshold complete calback, which calls the
 * dequue API which in-turn calls this callback. UART lock is taken during
 * the call to the threshold callback.
 *
 * 2. UART ISR - UART calls the dequue API which in-turn will call this API.
 * In this case, UART ISR takes the UART lock.
 * */
static void tegra_rx_dma_complete_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	struct uart_port *u = &t->uport;
	int dma_read_count = 0;
	int pio_read_count = 0;

	/* If we are here, DMA is stopped */

	dma_read_count = copy_dma_buffer_to_tty_buffer(t, req->bytes_transferred);
	pio_read_count = do_handle_rx_pio(t);

	if ((dma_read_count > 0) || pio_read_count) {
		tty_flip_buffer_push(u->state->port.tty);
	}
}

static void tegra_stop_rx(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned char ier;

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->rts_active)
		set_rts(t, false);

	if (t->rx_in_progress) {
		/* Enable the interrupts */
		ier = t->ier_shadow;
		ier |= UART_IER_RDI;
		uart_writeb(ier, t, UART_IER);

		ier &= ~(UART_IER_RDI | UART_IER_RLSI | UART_IER_RTOIE | UART_IER_EORD);
		t->ier_shadow = ier;
		uart_writeb(ier, t, UART_IER);
		t->rx_in_progress = 0;
	}

	if (likely(t->use_rx_dma))
		tegra_dma_dequeue(t->rx_dma);
	return;
}

static void do_handle_modem_signal(struct uart_port *u)
{
	bool is_change = false;
	unsigned char msr;
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);

	msr = uart_readb(t, UART_MSR);

	if (msr & UART_MSR_CTS) {
		/* status = (msr & NV_DRF_DEF(UART, MSR, CTS, ENABLE))?TIOCM_CTS:0;
		   uart_handle_cts_change(u, status);*/
		u->icount.cts++;
		is_change = true;
		dev_dbg(u->dev, "CTS changed\n");
	}

	if (msr & UART_MSR_DSR) {
		u->icount.dsr++;
		is_change = true;
		dev_dbg(u->dev, "DSR changed\n");
	}

	if (msr & UART_MSR_DCD) {
		u->icount.dcd++;
		is_change = true;
		dev_dbg(u->dev, "CD changed\n");
	}

	if (msr & UART_MSR_RI) {
		u->icount.rng++;
		is_change = true;
		dev_dbg(u->dev, "RI changed\n");
	}

	if (is_change)
		wake_up_interruptible(&u->state->port.delta_msr_wait);
	return;
}

static irqreturn_t tegra_uart_isr(int irq, void *data)
{
	struct tegra_uart_port *t = (struct tegra_uart_port *)data;
	struct uart_port *u = &t->uport;
	unsigned char iir;
	unsigned char ier;
	unsigned long flags;
	bool is_rx_int = false;

	spin_lock_irqsave(&u->lock, flags);
	while (1) {
		iir = uart_readb(t, UART_FCR);
		if (iir & UART_IIR_NO_INT) {
			if (is_rx_int) {
				if (likely(t->use_rx_dma))
					queue_work(t->rx_work_queue,
							&t->rx_work);
			}
			spin_unlock_irqrestore(&u->lock, flags);
			return IRQ_HANDLED;
		}

		dev_dbg(u->dev, "tegra_uart_isr iir = 0x%x\n", iir);
		switch ((iir >> 1) & 0x7) {
		case 0: /* Modem signal change interrupt */
			do_handle_modem_signal(u);
			break;
		case 1: /* Transmit interrupt only triggered when using PIO */
			do_handle_tx_pio(t);
			break;
		case 4: /* End of data */
		case 6: /* Rx timeout */
		case 2: /* Receive */
			if (likely(t->use_rx_dma)) {
				if (!is_rx_int) {
					is_rx_int = true;
					/* Disable interrups */
					ier = t->ier_shadow;
					/* Sw workaround to disable the
					 * rx timeout interrupt.*/
					ier |= UART_IER_RDI;
					uart_writeb(ier, t, UART_IER);

					ier &= ~(UART_IER_RDI | UART_IER_RLSI |
						UART_IER_RTOIE | UART_IER_EORD);
					t->ier_shadow = ier;
					uart_writeb(ier, t, UART_IER);
				}
			} else {
				do_handle_rx_pio(t);
				spin_unlock_irqrestore(&u->lock, flags);
				tty_flip_buffer_push(u->state->port.tty);
				spin_lock_irqsave(&u->lock, flags);
			}
			break;

		case 3: /* Receive error */
			do_decode_rx_error(t, uart_readb(t, UART_LSR));
			break;
		case 5: /* break nothing to handle */
		case 7: /* break nothing to handle */
			break;
		}
	}
}

static void tegra_uart_hw_deinit(struct tegra_uart_port *t)
{
	unsigned char fcr;
	unsigned long flag;

	spin_lock_irqsave(&t->reg_access_lock, flag);

	/* Disable interrupts */
	uart_writeb(0, t, UART_IER);

	/* TBD: why this delay is needed */
	udelay(200);

	/* Reset the Rx and Tx FIFOs */
	fcr = t->fcr_shadow;
	fcr |= UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR;
	uart_writeb(fcr, t, UART_FCR);

	tegra_periph_reset_assert(t->clk);
	udelay(100);
	tegra_periph_reset_deassert(t->clk);
	udelay(100);

	clk_disable(t->clk);
	spin_unlock_irqrestore(&t->reg_access_lock, flag);
	t->baud = 0;
	if (t->pinmux)
		tegra_pinmux_config_tristate_table(t->pinmux, t->nr_pins, TEGRA_TRI_TRISTATE);
	t->baud = 0;
	t->uart_state = TEGRA_UART_CLOSED;
}

static void tegra_uart_free_rx_dma(struct tegra_uart_port *t)
{
	if (!t->use_rx_dma)
		return;

	tegra_dma_free_channel(t->rx_dma);

	if (likely(t->rx_dma_req.dest_addr))
		dma_free_coherent(t->uport.dev, t->rx_dma_req.size,
			t->rx_dma_req.virt_addr, t->rx_dma_req.dest_addr);

	t->use_rx_dma = false;
}

static void tegra_uart_free_tx_dma(struct tegra_uart_port *t)
{
	if (!t->use_tx_dma)
		return;

	tegra_dma_free_channel(t->tx_dma);

	if (likely(t->tx_dma_req.source_addr))
		dma_free_coherent(t->uport.dev, UART_XMIT_SIZE,
			t->tx_dma_req.virt_addr, t->tx_dma_phys);

	t->use_tx_dma = false;
}

static int tegra_uart_hw_init(struct tegra_uart_port *t)
{
	unsigned char fcr;
	unsigned char ier;

	dev_vdbg(t->uport.dev, "+tegra_uart_hw_init\n");

	t->fcr_shadow = 0;
	t->mcr_shadow = 0;
	t->lcr_shadow = 0;
	t->ier_shadow = 0;
	t->baud = 0;
	if (t->pinmux)
		tegra_pinmux_config_tristate_table(t->pinmux, t->nr_pins, TEGRA_TRI_NORMAL);

	clk_enable(t->clk);
	msleep(10);

	/* Reset the UART controller to clear all previous status.*/
	tegra_periph_reset_assert(t->clk);
	udelay(100);
	tegra_periph_reset_deassert(t->clk);
	udelay(100);

	spin_lock(&t->reg_access_lock);

	t->rx_in_progress = 0;
	t->tx_in_progress = 0;

	/* Reset the FIFO twice with some delay to make sure that the FIFOs are
	 * really flushed. Wait is needed as the clearing needs to cross
	 * multiple clock domains.
	 * */
	t->fcr_shadow = UART_FCR_ENABLE_FIFO;

	fcr = t->fcr_shadow;
	fcr |= UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR;
	uart_writeb(fcr, t, UART_FCR);

	udelay(100);
	uart_writeb(fcr, t, UART_FCR);
	udelay(100);

	/* Set the trigger level
	 *
	 * For PIO mode:
	 *
	 * For receive, this will interrupt the CPU after that many number of
	 * bytes are received, for the remaining bytes the receive timeout
	 * interrupt is received.
	 *
	 *  Rx high watermark is set to 4.
	 *
	 * For transmit, if the trasnmit interrupt is enabled, this will
	 * interrupt the CPU when the number of entries in the FIFO reaches the
	 * low watermark.
	 *
	 *  Tx low watermark is set to 8.
	 *
	 *  For DMA mode:
	 *
	 *  Set the Tx trigger to 4. This should match the DMA burst size that
	 *  programmed in the DMA registers.
	 * */
	t->fcr_shadow |= UART_FCR_R_TRIG_01;

	if (t->use_tx_dma) {
		t->fcr_shadow |= UART_FCR_T_TRIG_01;
	} else {
		t->fcr_shadow |= TEGRA_UART_FIFO_TX_PIO_TRIGGER_LEVEL;
	}

	uart_writeb(t->fcr_shadow, t, UART_FCR);
	t->last_read_index = 0;
	t->already_read_bytecount = 0;
	t->last_transfer_count = 0;

	if (t->use_rx_dma) {
		t->fcr_shadow |= UART_FCR_DMA_SELECT;
		uart_writeb(t->fcr_shadow, t, UART_FCR);

		/* Initialize the uart for some default configi as it is
		 * going to start dma */
		t->lcr_shadow = 3; /* no parity,1 stop nit, 8 data bit */
		tegra_set_baudrate(t, 9600);

		if (tegra_start_dma_rx(t)) {
			dev_err(t->uport.dev, "Could not enqueue Rx DMA req\n");
			tegra_uart_free_rx_dma(t);
			t->use_rx_dma = false;
			t->fcr_shadow &= ~UART_FCR_DMA_SELECT;
			uart_writeb(t->fcr_shadow, t, UART_FCR);
		}
	} else {
		t->fcr_shadow &= ~UART_FCR_DMA_SELECT;
		uart_writeb(t->fcr_shadow, t, UART_FCR);
	}

	t->rx_in_progress = 1;

	/*
	 *  Enable IE_RXS for the receive status interrupts like line errros.
	 *  Enable IE_RX_TIMEOUT to get the bytes which cannot be DMA'd.
	 *
	 *  If using DMA mode, enable EORD instead of receive interrupt which
	 *  will interrupt after the UART is done with the receive instead of
	 *  the interrupt when the FIFO "threshold" is reached.
	 *
	 *  EORD is different interrupt than RX_TIMEOUT - RX_TIMEOUT occurs when
	 *  the DATA is sitting in the FIFO and couldn't be transferred to the
	 *  DMA as the DMA size alignment(4 bytes) is not met. EORD will be
	 *  triggered when there is a pause of the incomming data stream for 4
	 *  characters long.
	 *
	 *  For pauses in the data which is not aligned to 4 bytes, we get
	 *  both the EORD as well as RX_TIMEOUT - SW sees RX_TIMEOUT first
	 *  then the EORD.
	 *
	 *  Don't get confused, believe in the magic of nvidia hw...:-)
	 */
	ier = 0;
	ier |= UART_IER_RLSI | UART_IER_RTOIE;
	if (t->use_rx_dma)
		ier |= UART_IER_EORD;
	else
		ier |= UART_IER_RDI;
	t->ier_shadow = ier;
	uart_writeb(ier, t, UART_IER);
	spin_unlock(&t->reg_access_lock);

	t->uart_state = TEGRA_UART_OPENED;
	dev_vdbg(t->uport.dev, "-tegra_uart_hw_init\n");
	return 0;

}

static int tegra_uart_init_rx_dma(struct tegra_uart_port *t)
{
	dma_addr_t rx_dma_phys;
	void *rx_dma_virt;

	t->rx_dma = tegra_dma_allocate_channel(
					TEGRA_DMA_MODE_CONTINUOUS_SAME_BUFFER);
	if (IS_ERR_OR_NULL(t->rx_dma))
		return -ENODEV;

	memset(&t->rx_dma_req, 0, sizeof(t->rx_dma_req));

	t->rx_dma_req.size = UART_RX_DMA_BUFFER_SIZE;
	rx_dma_virt = dma_alloc_coherent(t->uport.dev,
		t->rx_dma_req.size, &rx_dma_phys, GFP_KERNEL);
	if (rx_dma_virt) {
		t->rx_dma_req.dest_addr = rx_dma_phys;
		t->rx_dma_req.virt_addr = rx_dma_virt;

		t->rx_dma_req.source_addr = t->uport.mapbase;

		t->rx_dma_req.source_wrap = 4;
		t->rx_dma_req.dest_wrap = 0;
		t->rx_dma_req.to_memory = 1;
		t->rx_dma_req.source_bus_width = 8;
		t->rx_dma_req.dest_bus_width = 32;
		t->rx_dma_req.req_sel = dma_req_sel[t->uport.line];

		t->rx_dma_req.complete = tegra_rx_dma_complete_callback;
		t->rx_dma_req.threshold = tegra_rx_dma_threshold_callback;
		t->rx_dma_req.dev = t;
		return 0;
	} else {
		tegra_dma_free_channel(t->rx_dma);
		return -ENODEV;
	}
}

static int tegra_uart_init_tx_dma(struct tegra_uart_port *t)
{
	t->tx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
	if (IS_ERR_OR_NULL(t->tx_dma))
		return -ENODEV;

	memset(&t->tx_dma_req, 0, sizeof(t->tx_dma_req));

	t->tx_dma_virt = dma_alloc_coherent(t->uport.dev,
			UART_XMIT_SIZE, &t->tx_dma_phys, GFP_KERNEL);
	if (t->tx_dma_virt) {
		t->tx_dma_size = UART_XMIT_SIZE;

		t->tx_dma_req.virt_addr = t->tx_dma_virt;

		/* Setup the DMA Tx request structure
		 * which doesn't change */
		t->tx_dma_req.instance = t->uport.line;
		t->tx_dma_req.to_memory = 0;
		t->tx_dma_req.complete = tegra_tx_dma_complete_callback;

		t->tx_dma_req.dest_addr = t->uport.mapbase;
		t->tx_dma_req.dest_wrap = 4;
		t->tx_dma_req.dest_bus_width = 8;

		t->tx_dma_req.source_bus_width = 32;
		t->tx_dma_req.source_wrap = 0;
		t->tx_dma_req.req_sel = dma_req_sel[t->uport.line];


		t->tx_dma_req.dev = t;
		t->tx_dma_req.size = 0;
	} else {
		tegra_dma_free_channel(t->tx_dma);
		t->use_tx_dma = false;
		return -ENODEV;
	}
	return 0;
}

static int tegra_startup(struct uart_port *u)
{
	struct tegra_uart_port *t;
	int ret = 0;

	t = container_of(u, struct tegra_uart_port, uport);
	sprintf(t->port_name, "tegra_uart_%d", u->line);

	BUG_ON(u->irq == (NvU32)(-1));

	t->use_tx_dma = false;
	if (!TX_FORCE_PIO) {
		if (!tegra_uart_init_tx_dma(t))
			t->use_tx_dma = true;
	}
	t->tx_in_progress = 0;

	t->use_rx_dma = false;
	if (!RX_FORCE_PIO) {
		if (!tegra_uart_init_rx_dma(t))
			t->use_rx_dma = true;
	}
	t->rx_in_progress = 0;

	ret = tegra_uart_hw_init(t);
	if (ret)
		goto fail;

	ret = request_irq(u->irq, tegra_uart_isr, IRQF_DISABLED, t->port_name, t);
	if (ret) {
		dev_err(u->dev, "Failed to register ISR for IRQ %d\n", u->irq);
		goto fail;
	}

	/* Set the irq flags to irq valid, which is the default linux behaviour.
	 * For irqs used by Nv* APIs, IRQF_NOAUTOEN is also set */
	set_irq_flags(u->irq, IRQF_VALID);
	dev_info(u->dev,"Started UART port %d\n", u->line);

	return 0;
fail:
	dev_err(u->dev, "Tegra UART startup failed\n");
	return ret;
}

static void tegra_shutdown(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_shutdown\n");

	tegra_uart_hw_deinit(t);
	spin_unlock_irqrestore(&u->lock, flags);

	tegra_uart_free_rx_dma(t);
	tegra_uart_free_tx_dma(t);

	free_irq(u->irq, u);
	dev_vdbg(u->dev, "-tegra_shutdown\n");
}

static unsigned int tegra_get_mctrl(struct uart_port *u)
{
	/* RI - Ring detector is active
	 * CD/DCD/CAR - Carrier detect is always active. For some reason
	 *			  linux has different names for carrier detect.
	 * DSR - Data Set ready is active as the hardware doesn't support it.
	 *	   Don't know if the linux support this yet?
	 * CTS - Clear to send. Always set to active, as the hardware handles
	 *	   CTS automatically.
	 * */
	return TIOCM_RI | TIOCM_CD | TIOCM_DSR | TIOCM_CTS;
}

static void set_rts(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr |= UART_MCR_RTS_EN;
	else
		mcr &= ~UART_MCR_RTS_EN;

	if (mcr != t->mcr_shadow) {
		uart_writeb(mcr, t, UART_MCR);
		t->mcr_shadow = mcr;
	}
	return;
}

static void set_dtr(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr |= UART_MCR_DTR;
	else
		mcr &= UART_MCR_DTR;
	if (mcr != t->mcr_shadow) {
		uart_writeb(mcr, t, UART_MCR);
		t->mcr_shadow = mcr;
	}
	return;
}

static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl)
{
	struct tegra_uart_port *t;

	dev_dbg(u->dev, "tegra_set_mctrl called with %d\n", mctrl);
	t = container_of(u, struct tegra_uart_port, uport);

	if (mctrl & TIOCM_RTS) {
		t->rts_active = true;
		set_rts(t, true);
	} else {
		t->rts_active = false;
		set_rts(t, false);
	}

	if (mctrl & TIOCM_DTR)
		set_dtr(t, true);
	else
		set_dtr(t, false);
	return;
}

static void tegra_break_ctl(struct uart_port *u, int break_ctl)
{
	struct tegra_uart_port *t;
	unsigned char lcr;

	t = container_of(u, struct tegra_uart_port, uport);
	lcr = t->lcr_shadow;
	if (break_ctl)
		lcr |= UART_LCR_SBC;
	else
		lcr &= ~UART_LCR_SBC;
	uart_writeb(lcr, t, UART_LCR);
	t->lcr_shadow = lcr;
}

static int tegra_request_port(struct uart_port *u)
{
	return 0;
}

static void tegra_release_port(struct uart_port *u)
{

}

static unsigned int tegra_tx_empty(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned int ret = 0;
	unsigned long flags;
	unsigned char lsr;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_tx_empty\n");

	spin_lock_irqsave(&u->lock, flags);
	if (!t->tx_in_progress) {
		lsr = uart_readb(t, UART_LSR);
		if ((lsr & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
			ret = TIOCSER_TEMT;
	}
	spin_unlock_irqrestore(&u->lock, flags);

	dev_vdbg(u->dev, "-tegra_tx_empty\n");
	return ret;
}

static void tegra_enable_ms(struct uart_port *u)
{
}

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud)
{
	unsigned long rate;
	unsigned int divisor;
	unsigned char lcr;

	if (t->baud == baud)
		return;

	rate = baud * 16;
	clk_set_rate(t->clk, rate);
	rate = clk_get_rate(t->clk);

	divisor = rate;
	do_div(divisor, 16);
	divisor += baud/2;
	do_div(divisor, baud);

	lcr = t->lcr_shadow;
	lcr |= UART_LCR_DLAB;
	uart_writeb(lcr, t, UART_LCR);

	uart_writel(divisor & 0xFF, t, UART_DLL);
	uart_writel(((divisor >> 8) & 0xFF), t, UART_DLM);

	lcr &= ~UART_LCR_DLAB;
	uart_writeb(lcr, t, UART_LCR);

	t->baud = baud;
	dev_vdbg(t->uport.dev, "Baud %u clock freq %lu and divisor of %u\n",
				baud, rate, divisor);
}

static void tegra_set_termios(struct uart_port *u, struct ktermios *termios,
					struct ktermios *oldtermios)
{
	struct tegra_uart_port *t;
	unsigned int baud;
	unsigned long flags;
	unsigned int lcr;
	unsigned int c_cflag = termios->c_cflag;
	char debug_string[50];
	unsigned char mcr;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(t->uport.dev, "+tegra_set_termios\n");
	debug_string[0] = 0;
	strlcat(debug_string, "uart port setting: ", 50);

	spin_lock_irqsave(&u->lock, flags);

	/* Changing configuration, it is safe to stop any rx now */
	if (t->rts_active)
		set_rts(t, false);

	/* Baud rate */
	baud = uart_get_baud_rate(u, termios, oldtermios, 200, 4000000);
	tegra_set_baudrate(t, baud);

	/* Parity */
	lcr = t->lcr_shadow;
	lcr &= ~UART_LCR_PARITY;
	if (PARENB == (c_cflag & PARENB)) {
		if (CMSPAR == (c_cflag & CMSPAR)) {
			strlcat(debug_string, "space parity ", 50);
			dev_err(t->uport.dev, "Space parity is not supported\n");
			BUG();
		} else if (c_cflag & PARODD) {
			strlcat(debug_string, "ODD parity ", 50);
			lcr |= UART_LCR_PARITY;
			lcr &= ~UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		} else {
			strlcat(debug_string, "Even parity ", 50);
			lcr |= UART_LCR_PARITY;
			lcr |= UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		}
	}

	lcr &= ~UART_LCR_WLEN8;
	switch (c_cflag & CSIZE) {
	case CS5:
		lcr |= UART_LCR_WLEN5;
		strlcat(debug_string, "5", 50);
		break;
	case CS6:
		lcr |= UART_LCR_WLEN6;
		strlcat(debug_string, "6", 50);
		break;
	case CS7:
		lcr |= UART_LCR_WLEN7;
		strlcat(debug_string, "7", 50);
		break;
	default:
		lcr |= UART_LCR_WLEN8;
		strlcat(debug_string, "8", 50);
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB) {
		lcr |= UART_LCR_STOP;
		strlcat(debug_string, "n2", 50);
	} else {
		lcr &= ~UART_LCR_STOP;
		strlcat(debug_string, "n1", 50);
	}

	uart_writeb(lcr, t, UART_LCR);
	t->lcr_shadow = lcr;

	/* Flow control */
	if (termios->c_cflag & CRTSCTS) {
		mcr = t->mcr_shadow;
		mcr |= UART_MCR_CTS_EN;
		mcr &= ~UART_MCR_RTS_EN;
		t->mcr_shadow = mcr;
		uart_writeb(mcr, t, UART_MCR);
		t->use_cts_control = true;

		/* if top layer have asked to set rts to active then do here */
		if (t->rts_active)
			set_rts(t, true);
	} else {
		mcr = t->mcr_shadow;
		mcr &= ~UART_MCR_CTS_EN;
		mcr &= ~UART_MCR_RTS_EN;
		t->mcr_shadow = mcr;
		uart_writeb(mcr, t, UART_MCR);
		t->use_cts_control = false;
	}

	/* update the port timeout based on new settings */
	uart_update_timeout(u, termios->c_cflag, baud);

	spin_unlock_irqrestore(&u->lock, flags);
	dev_dbg(u->dev, "%s\n", debug_string);
	dev_vdbg(t->uport.dev, "-tegra_set_termios\n");
	return;
}

/*
 * Flush any TX data submitted for DMA. Called when the TX circular
 * buffer is reset.
 */
static void tegra_flush_buffer(struct uart_port *u)
{
	struct tegra_uart_port *t;

	dev_vdbg(u->dev, "tegra_flush_buffer called");

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->use_tx_dma) {
		tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req);
		t->tx_dma_req.size = 0;
	}
	return;
}


static void tegra_pm(struct uart_port *u, unsigned int state,
	unsigned int oldstate)
{

}

static const char *tegra_type(struct uart_port *u)
{
	return 0;
}

static struct uart_ops tegra_uart_ops = {
	.tx_empty       = tegra_tx_empty,
	.set_mctrl      = tegra_set_mctrl,
	.get_mctrl      = tegra_get_mctrl,
	.stop_tx        = tegra_stop_tx,
	.start_tx       = tegra_start_tx,
	.stop_rx        = tegra_stop_rx,
	.flush_buffer   = tegra_flush_buffer,
	.enable_ms      = tegra_enable_ms,
	.break_ctl      = tegra_break_ctl,
	.startup        = tegra_startup,
	.shutdown       = tegra_shutdown,
	.set_termios    = tegra_set_termios,
	.pm             = tegra_pm,
	.type           = tegra_type,
	.request_port   = tegra_request_port,
	.release_port   = tegra_release_port,
};

static int __init tegra_uart_probe(struct platform_device *pdev);
static int __devexit tegra_uart_remove(struct platform_device *pdev);
static int tegra_uart_suspend(struct platform_device *pdev, pm_message_t state);
static int tegra_uart_resume(struct platform_device *pdev);

static struct platform_driver tegra_uart_platform_driver = {
	.remove         = tegra_uart_remove,
	.probe          = tegra_uart_probe,
	.suspend        = tegra_uart_suspend,
	.resume         = tegra_uart_resume,
	.driver         = {
		.name   = "tegra_uart"
	}
};

static struct uart_driver tegra_uart_driver =
{
	.owner          = THIS_MODULE,
	.driver_name    = "tegra_uart",
	.dev_name       = "ttyHS",
	.cons           = 0,
	.nr             = 5,
};

static int tegra_uart_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		pr_err("Invalid Uart instance (%d) \n", pdev->id);
		return -ENODEV;
	}

	dev_info(t->uport.dev, "tegra_uart_suspend called \n");

	if (t->odm_uart_handle)
		NvOdmUartSuspend(t->odm_uart_handle);

	u = &t->uport;

	/* Enable clock before calling suspend so that controller
	   register can be accessible */
	if (t->uart_state == TEGRA_UART_CLOCK_OFF) {
		clk_enable(t->clk);
		t->uart_state = TEGRA_UART_OPENED;
	}

	uart_suspend_port(&tegra_uart_driver, u);
	if (t->pinmux)
		tegra_pinmux_config_tristate_table(t->pinmux, t->nr_pins, TEGRA_TRI_TRISTATE);
	t->uart_state = TEGRA_UART_SUSPEND;
	return 0;
}

static int tegra_uart_resume(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		pr_err("Invalid Uart instance (%d) \n", pdev->id);
		return -ENODEV;
	}

	u = &t->uport;
	dev_info(t->uport.dev, "tegra_uart_resume called \n");
	if (t->pinmux)
		tegra_pinmux_config_tristate_table(t->pinmux, t->nr_pins, TEGRA_TRI_NORMAL);

	if (t->uart_state == TEGRA_UART_SUSPEND) {
		uart_resume_port(&tegra_uart_driver, u);
		t->uart_state = TEGRA_UART_OPENED;
	}
	if (t->odm_uart_handle)
		NvOdmUartResume(t->odm_uart_handle);
	return 0;
}

static int __devexit tegra_uart_remove(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		pr_err("Invalid Uart instance (%d) \n", pdev->id);
		return -ENODEV;
	}

	u = &t->uport;
	uart_remove_one_port(&tegra_uart_driver, u);
	if (t->odm_uart_handle)
		NvOdmUartClose(t->odm_uart_handle);
	destroy_workqueue(t->rx_work_queue);
	destroy_workqueue(t->tx_work_queue);

	platform_set_drvdata(pdev, NULL);

	dev_err(u->dev, "Unregistered UART port %s%d\n",
		tegra_uart_driver.dev_name, u->line);
	kfree(t);
	return 0;
}

static int __init tegra_uart_probe(struct platform_device *pdev)
{
	struct tegra_uart_port *t;
	struct tegra_serial_platform_data *pdata = pdev->dev.platform_data;
	struct uart_port *u;
	int ret;
	char name[64];
	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		dev_err(&pdev->dev, "Invalid Uart instance (%d) \n", pdev->id);
		return -ENODEV;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "Platform data is not available\n");
		return -ENODEV;
	}

	if (!pdata->p.membase) {
		dev_err(&pdev->dev, "Uart register mapping is not available\n");
		return -ENODEV;
	}

	t = kzalloc(sizeof(struct tegra_uart_port), GFP_KERNEL);
	if (!t) {
		dev_err(&pdev->dev,"%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	u = &t->uport;
	u->dev = &pdev->dev;
	platform_set_drvdata(pdev, u);
	u->line = pdev->id;
	u->ops = &tegra_uart_ops;
	u->type = ~PORT_UNKNOWN;
	u->fifosize = 32;
	u->mapbase = pdata->p.mapbase;
	u->membase = pdata->p.membase;
	u->irq = pdata->p.irq;
	t->rts_active = false;
	u->regshift = 2;
	t->pinmux = pdata->pinmux;
	t->nr_pins = pdata->nr_pins;
	t->odm_uart_handle = NvOdmUartOpen(pdev->id);

	t->clk = clk_get(&pdev->dev, NULL);
	if (!t->clk) {
		dev_err(&pdev->dev, "Couldn't get the clock\n");
		ret = -ENODEV;
		goto clk_fail;
	}


	ret = uart_add_one_port(&tegra_uart_driver, u);
	if (ret) {
		dev_err(u->dev, "%s: Failed(%d) to add uart port %s%d\n",
			__func__, ret, tegra_uart_driver.dev_name, u->line);
		goto clk_fail;
	}

	/* Create the workqueue for the Rx Path */
	snprintf(name, sizeof(name), "tegra_hsuart_rx_%d", u->line);
	t->rx_work_queue = create_singlethread_workqueue(name);
	if (t->rx_work_queue == NULL) {
		dev_err(u->dev, "Failed to create work queue\n");
		ret = -ENODEV;
		goto rx_workq_fail;
	}
	INIT_WORK(&t->rx_work, tegra_rx_dma_workqueue);

	/* Create the workqueue for the Tx Path */
	snprintf(name, sizeof(name), "tegra_hsuart_tx_%d", u->line);
	t->tx_work_queue = create_singlethread_workqueue(name);
	if (t->tx_work_queue == NULL) {
		dev_err(u->dev, "Failed to create work queue\n");
		ret = -ENODEV;
		goto tx_workq_fail;
	}

	INIT_WORK(&t->tx_work, tegra_tx_dma_workqueue);

	spin_lock_init(&t->reg_access_lock);

	dev_info(u->dev, "Registered UART port %s%d\n",
					tegra_uart_driver.dev_name, u->line);

	t->uart_state = TEGRA_UART_CLOSED;
	return ret;

tx_workq_fail:
	destroy_workqueue(t->rx_work_queue);

rx_workq_fail:
	uart_remove_one_port(&tegra_uart_driver, u);

clk_fail:
	if (t->odm_uart_handle)
		NvOdmUartClose(t->odm_uart_handle);
	kfree(t);
	platform_set_drvdata(pdev, NULL);
	return -ENODEV;
}

/* Switch off the clock of the uart controller. */
void tegra_uart_request_clock_off(struct uart_port *uport)
{
	unsigned long flags;
	struct tegra_uart_port *t;

	dev_vdbg(uport->dev, "tegra_uart_request_clock_off");

	t = container_of(uport, struct tegra_uart_port, uport);
	spin_lock_irqsave(&uport->lock, flags);
	if (t->uart_state == TEGRA_UART_OPENED) {
		clk_disable(t->clk);
		t->uart_state = TEGRA_UART_CLOCK_OFF;
	}
	spin_unlock_irqrestore(&uport->lock, flags);
	return;
}

/* Switch on the clock of the uart controller */
void tegra_uart_request_clock_on(struct uart_port *uport)
{
	unsigned long flags;
	struct tegra_uart_port *t;

	t = container_of(uport, struct tegra_uart_port, uport);
	spin_lock_irqsave(&uport->lock, flags);
	if (t->uart_state == TEGRA_UART_CLOCK_OFF) {
		clk_enable(t->clk);
		t->uart_state = TEGRA_UART_OPENED;
	}
	spin_unlock_irqrestore(&uport->lock, flags);
	return;
}

/* Set the modem control signals state of uart controller. */
void tegra_uart_set_mctrl(struct uart_port *uport, unsigned int mctrl)
{
	unsigned long flags;
	unsigned char mcr;
	struct tegra_uart_port *t;

	t = container_of(uport, struct tegra_uart_port, uport);
	spin_lock_irqsave(&uport->lock, flags);
	mcr = t->mcr_shadow;
	if (mctrl & TIOCM_RTS) {
		t->rts_active = true;
		set_rts(t, true);
	} else {
		t->rts_active = false;
		set_rts(t, false);
	}

	if (mctrl & TIOCM_DTR)
		set_dtr(t, true);
	else
		set_dtr(t, false);
	spin_unlock_irqrestore(&uport->lock, flags);
	return;
}

/* Return the status of the transmit fifo whether empty or not.
 * Return 0 if tx fifo is not empty.
 * Return TIOCSER_TEMT if tx fifo is empty.
 */
int tegra_uart_is_tx_empty(struct uart_port *uport)
{
	return tegra_tx_empty(uport);
}

static int __init tegra_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&tegra_uart_driver);
	if (unlikely(ret)) {
		pr_err("Could not register %s driver\n",
			tegra_uart_driver.driver_name);
		return ret;
	}

	ret = platform_driver_register(&tegra_uart_platform_driver);
	if (unlikely(ret)) {
		pr_err("Could not register the UART platfrom "
			"driver\n");
		uart_unregister_driver(&tegra_uart_driver);
		return ret;
	}

	pr_info("Initialized tegra uart driver\n");
	return 0;
}

static void __exit tegra_uart_exit(void)
{
	pr_info("Unloading tegra uart driver\n");
	platform_driver_unregister(&tegra_uart_platform_driver);
	uart_unregister_driver(&tegra_uart_driver);
}

module_init(tegra_uart_init);
module_exit(tegra_uart_exit);
MODULE_DESCRIPTION("High speed UART driver for tegra chipset");
