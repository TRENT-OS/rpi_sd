//
// sdhost.cpp
//
// BCM2835 SD host driver.
//
// Author:	Phil Elwell <phil@raspberrypi.org>
//		Copyright (C) 2015-2016 Raspberry Pi (Trading) Ltd.
//
// Ported to Circle by R. Stange
//
// Based on
//  mmc-bcm2835.c by Gellert Weisz
// which is, in turn, based on
//  sdhci-bcm2708.c by Broadcom
//  sdhci-bcm2835.c by Stephen Warren and Oleksandr Tymoshenko
//  sdhci.c and sdhci-pci.c by Pierre Ossman
//
// This program is free software; you can redistribute it and/or modify it
// under the terms and conditions of the GNU General Public License,
// version 2, as published by the Free Software Foundation.
//
// This program is distributed in the hope it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <circleenv/sysconfig.h>
#ifdef USE_SDHOST

#include <circle/bcm2837_sdhost.h>
#include <circle/bcm2837_gpio.h>

// #include <circle/mmcerror.h>
// #include <circle/bcmpropertytags.h>
#include <circleenv/machineinfo.h>
#include <circleenv/synchronize.h>
#include <circleenv/spinlock.h>
#include <circleenv/bcm2835int.h>
// #include <circle/logger.h>
#include <circleenv/macros.h>
#include <circleenv/util.h>
#include <assert.h>
#include <circleos.h>

#define SDHOST_DEBUG		0

#define FIFO_READ_THRESHOLD     4
#define FIFO_WRITE_THRESHOLD    4
#define ALLOW_CMD23_READ        0
#define ALLOW_CMD23_WRITE       0
#define SDDATA_FIFO_PIO_BURST   8
#define CMD_DALLY_US            1

#define DRIVER_NAME "sdhost-bcm2835"

#define SDCMD  0x00 /* Command to SD card              - 16 R/W */
#define SDARG  0x04 /* Argument to SD card             - 32 R/W */
#define SDTOUT 0x08 /* Start value for timeout counter - 32 R/W */
#define SDCDIV 0x0c /* Start value for clock divider   - 11 R/W */
#define SDRSP0 0x10 /* SD card response (31:0)         - 32 R   */
#define SDRSP1 0x14 /* SD card response (63:32)        - 32 R   */
#define SDRSP2 0x18 /* SD card response (95:64)        - 32 R   */
#define SDRSP3 0x1c /* SD card response (127:96)       - 32 R   */
#define SDHSTS 0x20 /* SD host status                  - 11 R   */
#define SDVDD  0x30 /* SD card power control           -  1 R/W */
#define SDEDM  0x34 /* Emergency Debug Mode            - 13 R/W */
#define SDHCFG 0x38 /* Host configuration              -  2 R/W */
#define SDHBCT 0x3c /* Host byte count (debug)         - 32 R/W */
#define SDDATA 0x40 /* Data to/from SD card            - 32 R/W */
#define SDHBLC 0x50 /* Host block count (SDIO/SDHC)    -  9 R/W */

#define SDCMD_NEW_FLAG                  0x8000
#define SDCMD_FAIL_FLAG                 0x4000
#define SDCMD_BUSYWAIT                  0x800
#define SDCMD_NO_RESPONSE               0x400
#define SDCMD_LONG_RESPONSE             0x200
#define SDCMD_WRITE_CMD                 0x80
#define SDCMD_READ_CMD                  0x40
#define SDCMD_CMD_MASK                  0x3f

#define SDCDIV_MAX_CDIV                 0x7ff

#define SDHSTS_BUSY_IRPT                0x400
#define SDHSTS_BLOCK_IRPT               0x200
#define SDHSTS_SDIO_IRPT                0x100
#define SDHSTS_REW_TIME_OUT             0x80
#define SDHSTS_CMD_TIME_OUT             0x40
#define SDHSTS_CRC16_ERROR              0x20
#define SDHSTS_CRC7_ERROR               0x10
#define SDHSTS_FIFO_ERROR               0x08
/* Reserved */
/* Reserved */
#define SDHSTS_DATA_FLAG                0x01

#define SDHSTS_TRANSFER_ERROR_MASK      (SDHSTS_CRC7_ERROR|SDHSTS_CRC16_ERROR| \
					 SDHSTS_REW_TIME_OUT|SDHSTS_FIFO_ERROR)
#define SDHSTS_ERROR_MASK               (SDHSTS_CMD_TIME_OUT|SDHSTS_TRANSFER_ERROR_MASK)

#define SDHCFG_BUSY_IRPT_EN     (1<<10)
#define SDHCFG_BLOCK_IRPT_EN    (1<<8)
#define SDHCFG_SDIO_IRPT_EN     (1<<5)
#define SDHCFG_DATA_IRPT_EN     (1<<4)
#define SDHCFG_SLOW_CARD        (1<<3)
#define SDHCFG_WIDE_EXT_BUS     (1<<2)
#define SDHCFG_WIDE_INT_BUS     (1<<1)
#define SDHCFG_REL_CMD_LINE     (1<<0)

#define SDEDM_FORCE_DATA_MODE   (1<<19)
#define SDEDM_CLOCK_PULSE       (1<<20)
#define SDEDM_BYPASS            (1<<21)

#define SDEDM_WRITE_THRESHOLD_SHIFT 9
#define SDEDM_READ_THRESHOLD_SHIFT 14
#define SDEDM_THRESHOLD_MASK     0x1f

#define SDEDM_FSM_MASK           0xf
#define SDEDM_FSM_IDENTMODE      0x0
#define SDEDM_FSM_DATAMODE       0x1
#define SDEDM_FSM_READDATA       0x2
#define SDEDM_FSM_WRITEDATA      0x3
#define SDEDM_FSM_READWAIT       0x4
#define SDEDM_FSM_READCRC        0x5
#define SDEDM_FSM_WRITECRC       0x6
#define SDEDM_FSM_WRITEWAIT1     0x7
#define SDEDM_FSM_POWERDOWN      0x8
#define SDEDM_FSM_POWERUP        0x9
#define SDEDM_FSM_WRITESTART1    0xa
#define SDEDM_FSM_WRITESTART2    0xb
#define SDEDM_FSM_GENPULSES      0xc
#define SDEDM_FSM_WRITEWAIT2     0xd
#define SDEDM_FSM_STARTPOWDOWN   0xf

#define SDDATA_FIFO_WORDS        16

#define USE_CMD23_FLAGS          ((ALLOW_CMD23_READ * MMC_DATA_READ) | \
				  (ALLOW_CMD23_WRITE * MMC_DATA_WRITE))

// #define MHZ			1000000

#define mmc_hostname(mmc)	"emmc1"

#define BUG_ON(cond)		assert (!(cond))
#define WARN_ON(cond)		assert (!(cond))

// #define pr_err(...)		CLogger::Get ()->Write (From, LogError,  ##__VA_ARGS__)
// #define pr_warn(...)		CLogger::Get ()->Write (From, LogWarning,  ##__VA_ARGS__)
// #define pr_info(...)		CLogger::Get ()->Write (From, LogNotice,  ##__VA_ARGS__)

#define pr_err(...)			LogWrite (From, USPI_LOG_ERROR,  ##__VA_ARGS__)
#define pr_warn(...)		LogWrite (From, USPI_LOG_WARNING,  ##__VA_ARGS__)
#define pr_info(...)		LogWrite (From, USPI_LOG_NOTICE,  ##__VA_ARGS__)
#define pr_debug(...)		if(SDHOST_DEBUG) LogWrite (From, USPI_LOG_DEBUG,  ##__VA_ARGS__)
// #if SDHOST_DEBUG
// 	#define pr_debug(...)	CLogger::Get ()->Write (From, LogDebug,  ##__VA_ARGS__)
// #else
// 	#define pr_debug(...)	((void) 0)
// #endif

#define cpu_relax()		DataSyncBarrier()
#define mmiowb()		DataSyncBarrier()

#define mdelay(ms)		MsDelay (ms)
#define udelay(us)		usDelay (us)
#define ndelay(ns)		nsDelay(ns)
// #define ndelay(ns)		usDelay ((ns) / 1000)

#define min(a, b)		((a) < (b) ? (a) : (b))
#define max(a, b)		((a) > (b) ? (a) : (b))

#define is_power_of_2(n)	(!((n) & ((n)-1)))

#define tasklet_schedule(tasklet) tasklet_finish()

static const char From[] = "sdhost";

#define CLOCK_ID_CORE		4

TSpinLock    m_pSpinLock;
TSpinLock 	*const spinlock = &m_pSpinLock;

// TInterruptSystem m_pInterrupt;
// TInterruptSystem *const interrupt = &m_pInterrupt;
	// CGPIOPin m_GPIO34_39[6];	// WiFi
	// CGPIOPin m_GPIO48_53[6];	// SD card

bcm2835_host m_Host;
bcm2835_host *const host = &m_Host;

void SDHostDevice (void *vaddr)
{
	MMCHost();
	SpinLock(spinlock,IRQ_LEVEL);

    // bcm2837_gpio_init(vaddr);
	bcm2837_gpio_init(gpioBaseReg);

	for (unsigned i = 0; i <= 5; i++)
	{
		// bcm2837_gpio_fsel(RPI_GPIO_P34 + i,BCM2837_GPIO_FSEL_INPT);
		// m_GPIO34_39[i].AssignPin (34+i);
		// m_GPIO34_39[i].SetMode (GPIOModeInput, FALSE);

		//SD card pins according to https://elinux.org/RPi_BCM2835_GPIOs
		bcm2837_gpio_fsel(RPI_GPIO_P48 + i,BCM2837_GPIO_FSEL_ALT0);

		// m_GPIO48_53[i].AssignPin (48+i);
		// m_GPIO48_53[i].SetMode (GPIOModeAlternateFunction0, FALSE);
		// m_GPIO48_53[i].SetPullMode (i == 0 ? GPIOPullModeOff : GPIOPullModeUp);
	}
}

void _SDHOSTDevice (void)
{
	PeripheralEntry ();

	sdhost_remove ();

	PeripheralExit ();
}

boolean sdhost_Initialize (void)
{
	PeripheralEntry ();

	int ret = probe ();

	PeripheralExit ();

	if (ret != 0)
	{
		return FALSE;
	}

	return TRUE;
}

void sdhost_Reset (void)
{
	PeripheralEntry ();

	reset (GetMMCHost ());

	PeripheralExit ();
}

void sdhost_SetIOS (mmc_ios_t *pIOS)
{
	PeripheralEntry ();

	assert (pIOS != 0);
	set_ios (GetMMCHost (), pIOS);

	PeripheralExit ();
}

void sdhost_Request (mmc_request_t *pRequest)
{
	PeripheralEntry ();

	assert (pRequest != 0);
	request (GetMMCHost (), pRequest);

	PeripheralExit ();
}

void dumpcmd (mmc_command_t *cmd, const char *label)
{
	if (cmd)
		pr_info("%s:%c%s op %d arg 0x%x flags 0x%x - resp %08x %08x %08x %08x, err %d",
			mmc_hostname(host->mmc),
			(cmd == host->cmd) ? '>' : ' ',
			label, cmd->opcode, cmd->arg, cmd->flags,
			cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
			cmd->error);
}

void dumpregs (void)
{
	if (host->mrq)
	{
		dumpcmd(host->mrq->sbc, "sbc");
		dumpcmd(host->mrq->cmd, "cmd");
		if (host->mrq->data)
			pr_info("%s: data blocks %x blksz %x - err %d",
				mmc_hostname(host->mmc),
				host->mrq->data->blocks,
				host->mrq->data->blksz,
				host->mrq->data->error);
		dumpcmd(host->mrq->stop, "stop");
	}

	pr_info("%s: =========== REGISTER DUMP ===========", mmc_hostname(host->mmc));

	pr_info("%s: SDCMD  0x%08x", mmc_hostname(host->mmc), read(SDCMD));
	pr_info("%s: SDARG  0x%08x", mmc_hostname(host->mmc), read(SDARG));
	pr_info("%s: SDTOUT 0x%08x", mmc_hostname(host->mmc), read(SDTOUT));
	pr_info("%s: SDCDIV 0x%08x", mmc_hostname(host->mmc), read(SDCDIV));
	pr_info("%s: SDRSP0 0x%08x", mmc_hostname(host->mmc), read(SDRSP0));
	pr_info("%s: SDRSP1 0x%08x", mmc_hostname(host->mmc), read(SDRSP1));
	pr_info("%s: SDRSP2 0x%08x", mmc_hostname(host->mmc), read(SDRSP2));
	pr_info("%s: SDRSP3 0x%08x", mmc_hostname(host->mmc), read(SDRSP3));
	pr_info("%s: SDHSTS 0x%08x", mmc_hostname(host->mmc), read(SDHSTS));
	pr_info("%s: SDVDD  0x%08x", mmc_hostname(host->mmc), read(SDVDD));
	pr_info("%s: SDEDM  0x%08x", mmc_hostname(host->mmc), read(SDEDM));
	pr_info("%s: SDHCFG 0x%08x", mmc_hostname(host->mmc), read(SDHCFG));
	pr_info("%s: SDHBCT 0x%08x", mmc_hostname(host->mmc), read(SDHBCT));
	pr_info("%s: SDHBLC 0x%08x", mmc_hostname(host->mmc), read(SDHBLC));

	pr_info("%s: =====================================", mmc_hostname(host->mmc));
}

void set_power (boolean on)
{
	write(on ? 1 : 0, SDVDD);
}

void reset_internal (void)
{
	if (host->debug)
		pr_info("%s: reset", mmc_hostname(host->mmc));

	set_power(false);

	write(0, SDCMD);
	write(0, SDARG);
	write(0xf00000, SDTOUT);
	write(0, SDCDIV);
	write(0x7f8, SDHSTS); /* Write 1s to clear */
	write(0, SDHCFG);
	write(0, SDHBCT);
	write(0, SDHBLC);

	/* Limit fifo usage due to silicon bug */
	u32 temp = read(SDEDM);
	temp &= ~((SDEDM_THRESHOLD_MASK<<SDEDM_READ_THRESHOLD_SHIFT) |
		  (SDEDM_THRESHOLD_MASK<<SDEDM_WRITE_THRESHOLD_SHIFT));
	temp |= (FIFO_READ_THRESHOLD << SDEDM_READ_THRESHOLD_SHIFT) |
		(FIFO_WRITE_THRESHOLD << SDEDM_WRITE_THRESHOLD_SHIFT);
	write(temp, SDEDM);
	mdelay(10);
	set_power(true);
	mdelay(10);
	host->clock = 0;
// 	host->sectors = 0;
	write(host->hcfg, SDHCFG);
	write(SDCDIV_MAX_CDIV, SDCDIV);
	mmiowb();
}

void reset (mmc_host_t *mmc)
{
	if (0 != sd_mutex_lock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
	}
	// SpinLockAcquire(spinlock);

	reset_internal ();

	if (0 != sd_mutex_unlock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
	}
	// SpinLockRelease(spinlock);
}

void init (int soft)
{
	pr_debug("init(%d)", soft);

	/* Set interrupt enables */
	host->hcfg = SDHCFG_BUSY_IRPT_EN;

	reset_internal();

	if (soft) {
		/* force clock reconfiguration */
		host->clock = 0;
		set_ios(host->mmc, &host->mmc->ios);
	}
}

void wait_transfer_complete(void)
{
	int timediff;
	u32 alternate_idle;
	u32 edm;

	alternate_idle = (host->mrq->data->flags & MMC_DATA_READ) ?
		SDEDM_FSM_READWAIT : SDEDM_FSM_WRITESTART1;

	edm = read(SDEDM);

	timediff = 0;

	while (1) {
		u32 fsm = edm & SDEDM_FSM_MASK;
		if ((fsm == SDEDM_FSM_IDENTMODE) ||
		    (fsm == SDEDM_FSM_DATAMODE))
			break;
		if (fsm == alternate_idle) {
			write(edm | SDEDM_FORCE_DATA_MODE, SDEDM);
			break;
		}

		timediff++;
		if (timediff == 100000) {
			pr_err("%s: wait_transfer_complete - still waiting after %d retries",
			       mmc_hostname(host->mmc),
			       timediff);
			dumpregs();
			host->mrq->data->error = -ETIMEDOUT;
			return;
		}
		cpu_relax();
		edm = read(SDEDM);
	}
}

void read_block_pio(void)
{
	size_t blksize = host->data->blksz;

	// unsigned start = m_pTimer->GetClockTicks();
	unsigned start = GetClockTicks();

	while (blksize) {
		int copy_words;
		u32 hsts = 0;

		if (!sg_miter_next(&host->sg_miter)) {
			host->data->error = -EINVAL;
			break;
		}

		size_t len = min(host->sg_miter.length, blksize);
		if (len % 4) {
			host->data->error = -EINVAL;
			break;
		}

		blksize -= len;
		host->sg_miter.consumed = len;

		u32 *buf = (u32 *)host->sg_miter.addr;

		copy_words = len/4;

		while (copy_words) {
			int burst_words, words;
			u32 edm;

			burst_words = SDDATA_FIFO_PIO_BURST;
			if (burst_words > copy_words)
				burst_words = copy_words;
			edm = read(SDEDM);
			words = ((edm >> 4) & 0x1f);

			if (words < burst_words) {
				int fsm_state = (edm & SDEDM_FSM_MASK);
				if ((fsm_state != SDEDM_FSM_READDATA) &&
				    (fsm_state != SDEDM_FSM_READWAIT) &&
				    (fsm_state != SDEDM_FSM_READCRC)) {
					hsts = read(SDHSTS);
					pr_info("%s: fsm %x, hsts %x",
					       mmc_hostname(host->mmc),
					       fsm_state, hsts);
					if (hsts & SDHSTS_ERROR_MASK)
						break;
				}

				// if (m_pTimer->GetClockTicks() - start > host->pio_timeout) {
				if (GetClockTicks() - start > host->pio_timeout) {
					pr_err("%s: PIO read timeout - EDM %x",
					       mmc_hostname(host->mmc),
					       edm);
					hsts = SDHSTS_REW_TIME_OUT;
					break;
				}
				udelay((burst_words - words) *
				       host->ns_per_fifo_word);
				// ndelay((burst_words - words) *
				//        host->ns_per_fifo_word);
				continue;
			} else if (words > copy_words) {
				words = copy_words;
			}

			copy_words -= words;

			while (words) {
				*(buf++) = read(SDDATA);
				words--;
			}
		}

		if (hsts & SDHSTS_ERROR_MASK)
			break;
	}

	sg_miter_stop(&host->sg_miter);
}

void write_block_pio (void)
{
	size_t blksize = host->data->blksz;

	// unsigned start = m_pTimer->GetClockTicks ();
	unsigned start = GetClockTicks ();

	while (blksize) {
		int copy_words;
		u32 hsts = 0;

		if (!sg_miter_next(&host->sg_miter)) {
			host->data->error = -EINVAL;
			break;
		}

		size_t len = min(host->sg_miter.length, blksize);
		if (len % 4) {
			host->data->error = -EINVAL;
			break;
		}

		blksize -= len;
		host->sg_miter.consumed = len;

		u32 *buf = (u32 *)host->sg_miter.addr;

		copy_words = len/4;

		while (copy_words) {
			int burst_words, words;
			u32 edm;

			burst_words = SDDATA_FIFO_PIO_BURST;
			if (burst_words > copy_words)
				burst_words = copy_words;
			edm = read(SDEDM);
			words = SDDATA_FIFO_WORDS - ((edm >> 4) & 0x1f);

			if (words < burst_words) {
				int fsm_state = (edm & SDEDM_FSM_MASK);
				if ((fsm_state != SDEDM_FSM_WRITEDATA) &&
				    (fsm_state != SDEDM_FSM_WRITESTART1) &&
				    (fsm_state != SDEDM_FSM_WRITESTART2)) {
					hsts = read(SDHSTS);
					pr_info("%s: fsm %x, hsts %x",
					       mmc_hostname(host->mmc),
					       fsm_state, hsts);
					if (hsts & SDHSTS_ERROR_MASK)
						break;
				}

				if (GetClockTicks () - start > host->pio_timeout) {
					pr_err("%s: PIO write timeout - EDM %x",
					       mmc_hostname(host->mmc),
					       edm);
					hsts = SDHSTS_REW_TIME_OUT;
					break;
				}
				udelay((burst_words - words) *
				       host->ns_per_fifo_word);
				// ndelay((burst_words - words) *
				//        host->ns_per_fifo_word);
				continue;
			} else if (words > copy_words) {
				words = copy_words;
			}

			copy_words -= words;

			while (words) {
				write(*(buf++), SDDATA);
				words--;
			}
		}

		if (hsts & SDHSTS_ERROR_MASK)
			break;
	}

	sg_miter_stop(&host->sg_miter);
}

void transfer_pio (void)
{
	BUG_ON(!host->data);

	boolean is_read = (host->data->flags & MMC_DATA_READ) != 0;
	if (is_read)
		read_block_pio();
	else
		write_block_pio();

	u32 sdhsts = read(SDHSTS);
	if (sdhsts & (SDHSTS_CRC16_ERROR |
		      SDHSTS_CRC7_ERROR |
		      SDHSTS_FIFO_ERROR)) {
		pr_err("%s: %s transfer error - HSTS %x",
		       mmc_hostname(host->mmc),
		       is_read ? "read" : "write",
		       sdhsts);
		host->data->error = -EILSEQ;
	} else if ((sdhsts & (SDHSTS_CMD_TIME_OUT |
			      SDHSTS_REW_TIME_OUT))) {
		pr_err("%s: %s timeout error - HSTS %x",
		       mmc_hostname(host->mmc),
		       is_read ? "read" : "write",
		       sdhsts);
		host->data->error = -ETIMEDOUT;
	}
}

void set_transfer_irqs (void)
{
	u32 all_irqs = SDHCFG_DATA_IRPT_EN | SDHCFG_BLOCK_IRPT_EN | SDHCFG_BUSY_IRPT_EN;

	host->hcfg = (host->hcfg & ~all_irqs) | SDHCFG_DATA_IRPT_EN | SDHCFG_BUSY_IRPT_EN;

	write(host->hcfg, SDHCFG);
}

void prepare_data (mmc_command_t *cmd)
{
	mmc_data_t *data = cmd->data;

	WARN_ON(host->data);

	host->data = data;
	if (!data)
		return;

	/* Sanity checks */
	BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > 65535);

	host->data_complete = 0;
	host->data->bytes_xfered = 0;

// 	if (!host->sectors && host->mmc->card) {
// 		struct mmc_card *card = host->mmc->card;
// 		if (!mmc_card_sd(card) && mmc_card_blockaddr(card)) {
// 			/*
// 			 * The EXT_CSD sector count is in number of 512 byte
// 			 * sectors.
// 			 */
// 			host->sectors = card->ext_csd.sectors;
// 		} else {
// 			/*
// 			 * The CSD capacity field is in units of read_blkbits.
// 			 * set_capacity takes units of 512 bytes.
// 			 */
// 			host->sectors = card->csd.capacity <<
// 				(card->csd.read_blkbits - 9);
// 		}
// 	}

	int flags = SG_MITER_ATOMIC;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;
	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
	host->blocks = data->blocks;

	set_transfer_irqs();

	write(data->blksz, SDHBCT);
	write(data->blocks, SDHBLC);

	BUG_ON(!host->data);
}

boolean send_command (mmc_command_t *cmd)
{
// 	WARN_ON(host->cmd);

	if (cmd->data) {
		pr_debug("%s: send_command %d 0x%x "
			 "(flags 0x%x) - %s %d*%d",
			 mmc_hostname(host->mmc),
			 cmd->opcode, cmd->arg, cmd->flags,
			 (cmd->data->flags & MMC_DATA_READ) ?
			 "read" : "write", cmd->data->blocks,
			 cmd->data->blksz);
	} else
		pr_debug("%s: send_command %d 0x%x (flags 0x%x)",
			 mmc_hostname(host->mmc),
			 cmd->opcode, cmd->arg, cmd->flags);

	/* Wait max 100 ms */
	unsigned long timeout = 10000;

	while (read(SDCMD) & SDCMD_NEW_FLAG) {
		if (timeout == 0) {
			pr_warn("%s: previous command never completed.",
				mmc_hostname(host->mmc));
			if (host->debug)
				dumpregs();
			tasklet_schedule(&host->finish_tasklet);
			return false;
		}
		timeout--;
		udelay(10);
	}

	int delay = (10000 - timeout)/100;
	if (delay > host->max_delay) {
		host->max_delay = delay;
		pr_warn("%s: controller hung for %d ms",
			   mmc_hostname(host->mmc),
			   host->max_delay);
	}

// 	timeout = jiffies;
// 	if (!cmd->data && cmd->busy_timeout > 9000)
// 		timeout += DIV_ROUND_UP(cmd->busy_timeout, 1000) * HZ + HZ;
// 	else
// 		timeout += 10 * HZ;
// 	mod_timer(&host->timer, timeout);

	host->cmd = cmd;

	/* Clear any error flags */
	u32 sdhsts = read(SDHSTS);
	if (sdhsts & SDHSTS_ERROR_MASK)
		write(sdhsts, SDHSTS);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		pr_err("%s: unsupported response type!",
			mmc_hostname(host->mmc));
		cmd->error = -EINVAL;
		tasklet_schedule(&host->finish_tasklet);
		return false;
	}

	prepare_data(cmd);

	write(cmd->arg, SDARG);

	u32 sdcmd = cmd->opcode & SDCMD_CMD_MASK;

	host->use_busy = 0;
	if (!(cmd->flags & MMC_RSP_PRESENT)) {
		sdcmd |= SDCMD_NO_RESPONSE;
	} else {
		if (cmd->flags & MMC_RSP_136)
			sdcmd |= SDCMD_LONG_RESPONSE;
		if (cmd->flags & MMC_RSP_BUSY) {
			sdcmd |= SDCMD_BUSYWAIT;
			host->use_busy = 1;
		}
	}

	if (cmd->data) {
		if (host->delay_after_this_stop) {
			// unsigned now = m_pTimer->GetClockTicks () * (1000000U / CLOCKHZ);
			unsigned now = GetClockTicks () * (1000000U / CLOCKHZ);
			unsigned time_since_stop = now - host->stop_time;
			if (time_since_stop < host->delay_after_this_stop)
				udelay(host->delay_after_this_stop - time_since_stop);
		}

		host->delay_after_this_stop = host->delay_after_stop;
// 		if ((cmd->data->flags & MMC_DATA_READ) && !host->use_sbc) {
// 			/* See if read crosses one of the hazardous sectors */
// 			u32 first_blk, last_blk;
//
// 			/* Intentionally include the following sector because
// 			   without CMD23/SBC the read may run on. */
// 			first_blk = host->mrq->cmd->arg;
// 			last_blk = first_blk + cmd->data->blocks;
//
// 			if (((last_blk >= (host->sectors - 64)) &&
// 			     (first_blk <= (host->sectors - 64))) ||
// 			    ((last_blk >= (host->sectors - 32)) &&
// 			     (first_blk <= (host->sectors - 32)))) {
// 				host->delay_after_this_stop =
// 					max(250u, host->delay_after_stop);
// 			}
// 		}

		if (cmd->data->flags & MMC_DATA_WRITE)
			sdcmd |= SDCMD_WRITE_CMD;
		if (cmd->data->flags & MMC_DATA_READ)
			sdcmd |= SDCMD_READ_CMD;
	}

	write(sdcmd | SDCMD_NEW_FLAG, SDCMD);

	return true;
}

void finish_data (void)
{
	mmc_data_t *data = host->data;
	BUG_ON(!data);

	pr_debug("finish_data(error %d, stop %d, sbc %d)",
	       data->error, data->stop ? 1 : 0,
	       host->mrq->sbc ? 1 : 0);

	host->hcfg &= ~(SDHCFG_DATA_IRPT_EN | SDHCFG_BLOCK_IRPT_EN);
	write(host->hcfg, SDHCFG);

	data->bytes_xfered = data->error ? 0 : (data->blksz * data->blocks);

	host->data_complete = 1;

	if (host->cmd) {
		/*
		 * Data managed to finish before the
		 * command completed. Make sure we do
		 * things in the proper order.
		 */
		pr_debug("Finished early - HSTS %x", read(SDHSTS));
	}
	else
		transfer_complete();
}

void transfer_complete(void)
{
	BUG_ON(host->cmd);
	BUG_ON(!host->data);
	BUG_ON(!host->data_complete);

	mmc_data_t *data = host->data;
	host->data = 0;

	pr_debug("transfer_complete(error %d, stop %d)",
	       data->error, data->stop ? 1 : 0);

	/*
	 * Need to send CMD12 if -
	 * a) open-ended multiblock transfer (no CMD23)
	 * b) error in multiblock transfer
	 */
	if (host->mrq->stop && (data->error || !host->use_sbc)) {
		if (send_command(host->mrq->stop)) {
			/* No busy, so poll for completion */
			if (!host->use_busy)
				finish_command();

			if (host->delay_after_this_stop)
				host->stop_time = GetClockTicks () * (1000000U / CLOCKHZ);
				// host->stop_time = m_pTimer->GetClockTicks () * (1000000U / CLOCKHZ);
		}
	} else {
		wait_transfer_complete();
		tasklet_schedule(&host->finish_tasklet);
	}
}

/* If irq_flags is valid, the caller is in a thread context and is allowed
   to sleep */
void finish_command (void)
{
	u32 sdcmd;

	pr_debug("finish_command(0x%x)", read(SDCMD));

	BUG_ON(!host->cmd || !host->mrq);

	/* Poll quickly at first */

	u32 retries = host->cmd_quick_poll_retries;
	if (!retries) {
		/* Work out how many polls take 1us by timing 10us */
		int us_diff;

		retries = 1;
		do {
			retries *= 2;

			// unsigned start = m_pTimer->GetClockTicks ();
			unsigned start = GetClockTicks ();

			for (unsigned i = 0; i < retries; i++) {
				cpu_relax();
				sdcmd = read(SDCMD);
			}

			// unsigned now = m_pTimer->GetClockTicks ();
			unsigned now = GetClockTicks ();
			us_diff = (now - start) * (1000000 / CLOCKHZ);
		} while (us_diff < 10);

		host->cmd_quick_poll_retries = ((retries * us_diff + 9)*CMD_DALLY_US)/10 + 1;
		retries = 1; // We've already waited long enough this time
	}

	for (sdcmd = read(SDCMD);
	     (sdcmd & SDCMD_NEW_FLAG) && retries;
	     retries--) {
		cpu_relax();
		sdcmd = read(SDCMD);
	}

	if (!retries) {
// 		if (!irq_flags) {
// 			/* Schedule the work */
// 			schedule_work(&host->cmd_wait_wq);
// 			return;
// 		}

		/* Wait max 100 ms */
		unsigned start = GetClockTicks ();
		while (GetClockTicks () - start < CLOCKHZ/10) {
			if (0 != sd_mutex_unlock())
			{
				LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
			}
			// SpinLockRelease(spinlock);
			udelay(10);
			if (0 != sd_mutex_lock())
			{
				LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
			}
			// SpinLockAcquire(spinlock);
			sdcmd = read(SDCMD);
			if (!(sdcmd & SDCMD_NEW_FLAG))
				break;
		}
	}

	/* Check for errors */
	if (sdcmd & SDCMD_NEW_FLAG) {
		if (host->debug) {
			pr_err("%s: command %d never completed.",
			       mmc_hostname(host->mmc), host->cmd->opcode);
			dumpregs();
		}
		host->cmd->error = -EILSEQ;
		tasklet_schedule(&host->finish_tasklet);
		return;
	} else if (sdcmd & SDCMD_FAIL_FLAG) {
		u32 sdhsts = read(SDHSTS);

		/* Clear the errors */
		write(SDHSTS_ERROR_MASK, SDHSTS);

		if (host->debug)
			pr_info("%s: error detected - CMD %x, HSTS %03x, EDM %x",
				mmc_hostname(host->mmc), sdcmd, sdhsts,
				read(SDEDM));

		if ((sdhsts & SDHSTS_CRC7_ERROR) &&
		    (host->cmd->opcode == 1)) {
			if (host->debug)
				pr_info("%s: ignoring CRC7 error for CMD1",
					mmc_hostname(host->mmc));
		} else {
			u32 edm, fsm;

			if (sdhsts & SDHSTS_CMD_TIME_OUT) {
				if (host->debug)
					pr_warn("%s: command %d timeout",
					       mmc_hostname(host->mmc),
					       host->cmd->opcode);
				host->cmd->error = -ETIMEDOUT;
			} else {
				pr_warn("%s: unexpected command %d error",
				       mmc_hostname(host->mmc),
				       host->cmd->opcode);
				host->cmd->error = -EILSEQ;
			}

			edm = read(SDEDM);
			fsm = edm & SDEDM_FSM_MASK;
			if (fsm == SDEDM_FSM_READWAIT ||
			    fsm == SDEDM_FSM_WRITESTART1)
				write(edm | SDEDM_FORCE_DATA_MODE, SDEDM);
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
	}

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			int i;
			for (i = 0; i < 4; i++)
				host->cmd->resp[3 - i] = read(SDRSP0 + i*4);
			pr_debug("%s: finish_command %08x %08x %08x %08x",
				 mmc_hostname(host->mmc),
				 host->cmd->resp[0],
				 host->cmd->resp[1],
				 host->cmd->resp[2],
				 host->cmd->resp[3]);
		} else {
			host->cmd->resp[0] = read(SDRSP0);
			pr_debug("%s: finish_command %08x",
				 mmc_hostname(host->mmc),
				 host->cmd->resp[0]);
		}
	}

	if (host->cmd == host->mrq->sbc) {
		/* Finished CMD23, now send actual command. */
		host->cmd = 0;
		if (send_command(host->mrq->cmd)) {
			/* PIO starts after irq */
			if (!host->use_busy)
				finish_command();
		}
	} else if (host->cmd == host->mrq->stop) {
		/* Finished CMD12 */
		tasklet_schedule(&host->finish_tasklet);
	} else {
		/* Processed actual command. */
		host->cmd = 0;
		if (!host->data)
			tasklet_schedule(&host->finish_tasklet);
		else if (host->data_complete)
			transfer_complete();
		// else
		// 	tasklet_schedule(&host->finish_tasklet);
	}
}

void busy_irq (u32 intmask)
{
	if (!host->cmd) {
		pr_err("%s: got command busy interrupt 0x%08x even "
			"though no command operation was in progress.",
			mmc_hostname(host->mmc), (unsigned)intmask);
		dumpregs();
		return;
	}

	if (!host->use_busy) {
		pr_err("%s: got command busy interrupt 0x%08x even "
			"though not expecting one.",
			mmc_hostname(host->mmc), (unsigned)intmask);
		dumpregs();
		return;
	}
	host->use_busy = 0;

	if (intmask & SDHSTS_ERROR_MASK)
	{
		pr_err("sdhost_busy_irq: intmask %x, data %p", intmask, host->mrq->data);
		if (intmask & SDHSTS_CRC7_ERROR)
			host->cmd->error = -EILSEQ;
		else if (intmask & (SDHSTS_CRC16_ERROR |
				    SDHSTS_FIFO_ERROR)) {
			if (host->mrq->data)
				host->mrq->data->error = -EILSEQ;
			else
				host->cmd->error = -EILSEQ;
		} else if (intmask & SDHSTS_REW_TIME_OUT) {
			if (host->mrq->data)
				host->mrq->data->error = -ETIMEDOUT;
			else
				host->cmd->error = -ETIMEDOUT;
		} else if (intmask & SDHSTS_CMD_TIME_OUT)
			host->cmd->error = -ETIMEDOUT;

		if (host->debug) {
			dumpregs();
		}
	}
	else
		finish_command();

}

void data_irq (u32 intmask)
{
	/* There are no dedicated data/space available interrupt
	   status bits, so it is necessary to use the single shared
	   data/space available FIFO status bits. It is therefore not
	   an error to get here when there is no data transfer in
	   progress. */
	if (!host->data)
		return;

	if (intmask & (SDHSTS_CRC16_ERROR |
		       SDHSTS_FIFO_ERROR |
		       SDHSTS_REW_TIME_OUT)) {
		if (intmask & (SDHSTS_CRC16_ERROR |
			       SDHSTS_FIFO_ERROR))
			host->data->error = -EILSEQ;
		else
			host->data->error = -ETIMEDOUT;

		if (host->debug) {
			dumpregs();
		}
	}

	if (host->data->error) {
		finish_data();
	} else if (host->data->flags & MMC_DATA_WRITE) {
		/* Use the block interrupt for writes after the first block */
		host->hcfg &= ~(SDHCFG_DATA_IRPT_EN);
		host->hcfg |= SDHCFG_BLOCK_IRPT_EN;
		write(host->hcfg, SDHCFG);
		transfer_pio();
	} else {
		transfer_pio();
		host->blocks--;
		if ((host->blocks == 0) || host->data->error)
			finish_data();
	}
}

void block_irq (u32 intmask)
{
	if (!host->data) {
		pr_err("%s: got block interrupt 0x%08x even "
			"though no data operation was in progress.",
			mmc_hostname(host->mmc), (unsigned)intmask);
		dumpregs();
		return;
	}

	if (intmask & (SDHSTS_CRC16_ERROR |
		       SDHSTS_FIFO_ERROR |
		       SDHSTS_REW_TIME_OUT)) {
		if (intmask & (SDHSTS_CRC16_ERROR |
			       SDHSTS_FIFO_ERROR))
			host->data->error = -EILSEQ;
		else
			host->data->error = -ETIMEDOUT;

		if (host->debug) {
			dumpregs();
		}
	}

	BUG_ON(!host->blocks);
	if (host->data->error || (--host->blocks == 0)) {
		finish_data();
	} else {
		transfer_pio();
	}
}

void irq_handler (void)
{
	if (0 != sd_mutex_lock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
	}
	// SpinLockAcquire(spinlock);

	u32 intmask = read(SDHSTS);

	write(  SDHSTS_BUSY_IRPT | SDHSTS_BLOCK_IRPT
	      | SDHSTS_SDIO_IRPT | SDHSTS_DATA_FLAG, SDHSTS);

	if (intmask & SDHSTS_BLOCK_IRPT) {
		block_irq(intmask);
	}

	if (intmask & SDHSTS_BUSY_IRPT) {
		busy_irq(intmask);
	}

	/* There is no true data interrupt status bit, so it is
	   necessary to qualify the data flag with the interrupt
	   enable bit */
	if ((intmask & SDHSTS_DATA_FLAG) &&
	    (host->hcfg & SDHCFG_DATA_IRPT_EN)) {
		data_irq(intmask);
	}

	mmiowb();

	if (0 != sd_mutex_unlock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
	}
	// SpinLockRelease(spinlock);
}

void irq_stub (void *param)
{
	PeripheralEntry ();

	irq_handler ();

	PeripheralExit ();
}

void set_clock (unsigned int clock)
{
	unsigned int input_clock = clock;

	if (host->debug)
		pr_info("%s: set_clock(%d)", mmc_hostname(host->mmc), clock);

	if (host->overclock_50 && (clock == 50*MHZ))
		clock = host->overclock_50 * MHZ + (MHZ - 1);

	/* The SDCDIV register has 11 bits, and holds (div - 2).
	   But in data mode the max is 50MHz wihout a minimum, and only the
	   bottom 3 bits are used. Since the switch over is automatic (unless
	   we have marked the card as slow...), chosen values have to make
	   sense in both modes.
	   Ident mode must be 100-400KHz, so can range check the requested
	   clock. CMD15 must be used to return to data mode, so this can be
	   monitored.

	   clock 250MHz -> 0->125MHz, 1->83.3MHz, 2->62.5MHz, 3->50.0MHz
                           4->41.7MHz, 5->35.7MHz, 6->31.3MHz, 7->27.8MHz

			 623->400KHz/27.8MHz
			 reset value (507)->491159/50MHz

	   BUT, the 3-bit clock divisor in data mode is too small if the
	   core clock is higher than 250MHz, so instead use the SLOW_CARD
	   configuration bit to force the use of the ident clock divisor
	   at all times.
	*/

	host->mmc->actual_clock = 0;

	if (host->firmware_sets_cdiv) {
		u32 msg[3] = { clock, 0, 0 };

		SetSDHostClock(msg,3);
		// set_sdhost_clock (msg);

		clock = max(msg[1], msg[2]);

		if (0 != sd_mutex_lock())
		{
			LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
		}
		// SpinLockAcquire(spinlock);
	} else {
		if (0 != sd_mutex_lock())
		{
			LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
		}
		// SpinLockAcquire(spinlock);

		if (clock < 100000) {
			/* Can't stop the clock, but make it as slow as
			 * possible to show willing
			 */
			host->cdiv = SDCDIV_MAX_CDIV;
			write(host->cdiv, SDCDIV);
			mmiowb();
			if (0 != sd_mutex_unlock())
			{
				LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
			}
			// SpinLockRelease(spinlock);
			return;
		}

		int div = host->max_clk / clock;
		if (div < 2)
			div = 2;
		if ((host->max_clk / div) > clock)
			div++;
		div -= 2;

		if (div > SDCDIV_MAX_CDIV)
			div = SDCDIV_MAX_CDIV;

		clock = host->max_clk / (div + 2);

		host->cdiv = div;
		write(host->cdiv, SDCDIV);

		if (host->debug)
			pr_info("%s: clock=%d -> max_clk=%d, cdiv=%x "
				"(actual clock %d)",
				mmc_hostname(host->mmc), input_clock,
				host->max_clk, host->cdiv,
				clock);
	}

	/* Calibrate some delays */

	host->ns_per_fifo_word = (1000000000/clock) *
		((host->mmc->caps & MMC_CAP_4_BIT_DATA) ? 8 : 32);

	if (input_clock == 50 * MHZ) {
		if (clock > input_clock) {
			/* Save the closest value, to make it easier
			   to reduce in the event of error */
			host->overclock_50 = (clock/MHZ);

			if (clock != host->overclock) {
				pr_info("%s: overclocking to %dHz",
					mmc_hostname(host->mmc), clock);
				host->overclock = clock;
			}
		} else if (host->overclock) {
			host->overclock = 0;
			if (clock == 50 * MHZ)
				pr_warn("%s: cancelling overclock",
					mmc_hostname(host->mmc));
		}
	} else if (input_clock == 0) {
		/* Reset the preferred overclock when the clock is stopped.
		 * This always happens during initialisation. */
		host->overclock_50 = host->user_overclock_50;
		host->overclock = 0;
	}

	/* Set the timeout to 500ms */
	write(clock/2, SDTOUT);

	host->mmc->actual_clock = clock;
	host->clock = input_clock;
	host->reset_clock = 0;

	mmiowb();
	if (0 != sd_mutex_unlock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
	}
	// SpinLockRelease(spinlock);
}

void request (mmc_host_t *mmc, mmc_request_t *mrq)
{
	if (host->debug) {
		mmc_command_t *cmd = mrq->cmd;
		BUG_ON(!cmd);
		if (cmd->data)
			pr_info("%s: cmd %d 0x%x (flags 0x%x) - %s %d*%d",
				mmc_hostname(mmc),
				cmd->opcode, cmd->arg, cmd->flags,
				(cmd->data->flags & MMC_DATA_READ) ?
				"read" : "write", cmd->data->blocks,
				cmd->data->blksz);
		else
			pr_info("%s: cmd %d 0x%x (flags 0x%x)",
				mmc_hostname(mmc),
				cmd->opcode, cmd->arg, cmd->flags);
	}

	/* Reset the error statuses in case this is a retry */
	if (mrq->sbc)
		mrq->sbc->error = 0;
	if (mrq->cmd)
		mrq->cmd->error = 0;
	if (mrq->data)
		mrq->data->error = 0;
	if (mrq->stop)
		mrq->stop->error = 0;

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		pr_err("%s: unsupported block size (%d bytes)",
		       mmc_hostname(mmc), mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (host->reset_clock)
	    set_clock(host->clock);

	if (0 != sd_mutex_lock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
	}
	// SpinLockAcquire(spinlock);

	WARN_ON(host->mrq != 0);
	host->mrq = mrq;

	u32 edm = read(SDEDM);
	u32 fsm = edm & SDEDM_FSM_MASK;

	if ((fsm != SDEDM_FSM_IDENTMODE) &&
	    (fsm != SDEDM_FSM_DATAMODE)) {
		if (host->debug) {
			pr_warn("%s: previous command (%d) not complete (EDM %x)",
			       mmc_hostname(host->mmc),
			       read(SDCMD) & SDCMD_CMD_MASK,
			       edm);
			dumpregs();
		}
		mrq->cmd->error = -EILSEQ;
		tasklet_schedule(&host->finish_tasklet);
		mmiowb();
		if (0 != sd_mutex_unlock())
		{
			LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
		}
		// SpinLockRelease(spinlock);
		return;
	}

	host->use_sbc = !!mrq->sbc &&
		(host->mrq->data->flags & USE_CMD23_FLAGS);
	if (host->use_sbc) {
		if (send_command(mrq->sbc)) {
			if (!host->use_busy)
				finish_command();
		}
	} else if (send_command(mrq->cmd)) {
		/* PIO starts after irq */

		if (!host->use_busy)
		{
			finish_command();
		}
		// else{ //TODO: was added just to get further in the program flow
		// 	finish_command();
		// 	print_mmc_host_info();
		// }
	}

	mmiowb();

	if (0 != sd_mutex_unlock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
	}
	// SpinLockRelease(spinlock);
}

void set_ios (mmc_host_t *mmc, mmc_ios_t *ios)
{
	if (host->debug)
		pr_info("%s: ios clock %d, pwr %d, bus_width %d, "
			"timing %d, vdd %d, drv_type %d",
			mmc_hostname(mmc),
			ios->clock, ios->power_mode, ios->bus_width,
			ios->timing, ios->signal_voltage, ios->drv_type);

	if (0 != sd_mutex_lock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to lock mutex!", __func__);
	}
	// SpinLockAcquire(spinlock);

	/* set bus width */
	host->hcfg &= ~SDHCFG_WIDE_EXT_BUS;
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		host->hcfg |= SDHCFG_WIDE_EXT_BUS;

	host->hcfg |= SDHCFG_WIDE_INT_BUS;

	/* Disable clever clock switching, to cope with fast core clocks */
	host->hcfg |= SDHCFG_SLOW_CARD;

	write(host->hcfg, SDHCFG);

	mmiowb();

	if (0 != sd_mutex_unlock())
	{
		LogWrite(From,USPI_LOG_ERROR,"%s: failed to unlock mutex!", __func__);
	}
	// SpinLockRelease(spinlock);

	if (!ios->clock || ios->clock != host->clock)
		set_clock(ios->clock);
}

void tasklet_finish (void)
{
	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq) {
		return;
	}

// 	del_timer(&host->timer);

	mmc_request_t *mrq = host->mrq;

	/* Drop the overclock after any data corruption, or after any
	 * error while overclocked. Ignore errors for status commands,
	 * as they are likely when a card is ejected. */
	if (host->overclock) {
		if ((mrq->cmd && mrq->cmd->error &&
		     (mrq->cmd->opcode != MMC_SEND_STATUS)) ||
		    (mrq->data && mrq->data->error) ||
		    (mrq->stop && mrq->stop->error) ||
		    (mrq->sbc && mrq->sbc->error)) {
			host->overclock_50--;
			pr_warn("%s: reducing overclock due to errors",
				mmc_hostname(host->mmc));
			host->reset_clock = 1;
			mrq->cmd->error = -ETIMEDOUT;
			mrq->cmd->retries = 1;
		}
	}

	host->mrq = 0;
	host->cmd = 0;
	host->data = 0;

	mmiowb();

	/* The SDHOST block doesn't report any errors for a disconnected
	   interface. All cards and SDIO devices should report some supported
	   voltage range, so a zero response to SEND_OP_COND, IO_SEND_OP_COND
	   or APP_SEND_OP_COND can be treated as an error. */
	if (((mrq->cmd->opcode == MMC_SEND_OP_COND) ||
	     (mrq->cmd->opcode == SD_IO_SEND_OP_COND) ||
	     (mrq->cmd->opcode == SD_APP_OP_COND)) &&
	    (mrq->cmd->error == 0) &&
	    (mrq->cmd->resp[0] == 0)) {
		mrq->cmd->error = -ETIMEDOUT;
		if (host->debug)
			pr_info("%s: faking timeout due to zero OCR",
				mmc_hostname(host->mmc));
	}

	mmc_request_done(host->mmc, mrq);
}

int add_host (void)
{
	mmc_host_t *mmc = host->mmc;

	if (!mmc->f_max || mmc->f_max > host->max_clk)
		mmc->f_max = host->max_clk;
	mmc->f_min = host->max_clk / SDCDIV_MAX_CDIV;

	mmc->max_busy_timeout = (~(unsigned int)0)/(mmc->f_max/1000);

	pr_debug("f_max %d, f_min %d, max_busy_timeout %d",
		 mmc->f_max, mmc->f_min, mmc->max_busy_timeout);

	/* host controller capabilities */
	mmc->caps |=
		MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED |
		MMC_CAP_NEEDS_POLL | MMC_CAP_HW_RESET | MMC_CAP_ERASE |
		((ALLOW_CMD23_READ|ALLOW_CMD23_WRITE) * MMC_CAP_CMD23);

	mmc->max_segs = 128;
	mmc->max_req_size = 524288;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_size = 512;
	mmc->max_blk_count =  65535;

	/* report supported voltage ranges */
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

// 	tasklet_init(&host->finish_tasklet,
// 		bcm2835_sdhost_tasklet_finish, (unsigned long)host);
//
// 	INIT_WORK(&host->cmd_wait_wq, bcm2835_sdhost_cmd_wait_work);
//
// 	timer_setup(&host->timer, bcm2835_sdhost_timeout, 0);

	init(0);

	ConnectInterrupt(host->irq,irq_stub,NULL);
	// m_pInterruptSystem->ConnectIRQ (host->irq, irq_stub, this);

	mmiowb();

	pr_info("%s: %s loaded", mmc_hostname(mmc), DRIVER_NAME);

	return 0;

// untasklet:
// 	tasklet_kill(&host->finish_tasklet);
//
// 	return ret;
}

int probe (void)
{
	int ret = 0;

	memset (host, 0, sizeof *host);
	host->debug = SDHOST_DEBUG;

	pr_debug("probe");

	mmc_host_t *mmc = GetMMCHost ();

	host->mmc = mmc;
	host->pio_timeout = 500*CLOCKHZ/1000;
	host->max_delay = 1; /* Warn if over 1ms */

	/* Read any custom properties */
	host->delay_after_stop = 0;
	host->user_overclock_50 = 0;

// 	clk = devm_clk_get(dev, NULL);
// 	if (IS_ERR(clk)) {
// 		ret = PTR_ERR(clk);
// 		if (ret == -EPROBE_DEFER)
// 			dev_info(dev, "could not get clk, deferring probe\n");
// 		else
// 			dev_err(dev, "could not get clk\n");
// 		goto err;
// 	}
	host->max_clk = GetClockRate (CLOCK_ID_CORE);
	host->irq = ARM_IRQ_SDIO;

	pr_debug(" - max_clk %lu, irq %d",
		 (unsigned long)host->max_clk,
		 (int)host->irq);

	mmc->caps |= MMC_CAP_4_BIT_DATA;

	u32 msg[3];
	msg[0] = 0U;
	msg[1] = ~0U;
	msg[2] = ~0U;
	// ret = set_sdhost_clock (msg);
	ret = SetSDHostClock(msg,3);
	if (!ret) {
		pr_err ("SetSDHostClock() failed");
		goto err;
	}

	host->firmware_sets_cdiv = (msg[1] != ~0U);
	if (host->firmware_sets_cdiv)
		pr_debug("firmware sets clock divider");

	ret = add_host();
	if (ret)
		goto err;

	pr_debug("probe -> OK");

	return 0;

err:
	pr_debug("probe -> err %d", ret);

	return ret;
}

int sdhost_remove (void)
{
	pr_debug("bcm2835_sdhost_remove");

	set_power(false);

	DisconnectInterrupt(host->irq);
	// m_pInterruptSystem->DisconnectIRQ (host->irq);


// 	del_timer_sync(&host->timer);
// 	tasklet_kill(&host->finish_tasklet);

	pr_debug("bcm2835_sdhost_remove - OK");

	return 0;
}

#endif	// #ifdef USE_SDHOST
