/*
 * slcan.c - serial line CAN interface driverfor multiple busses per serial 
 *
 * This file is derived from linux/drivers/can/slcan.c
 *
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 * slcan_multimode.c Author: Tristan Timmermans <>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/can.h>
#include <linux/can/skb.h>
#include <linux/can/can-ml.h>

MODULE_ALIAS_LDISC(N_SLCAN);
MODULE_DESCRIPTION("serial line CAN interface driver for multiple busses per serial line");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tristan Timmermans");

#define SLCAN_MAGIC 0x53CA // same as SLCAN

static int maxdev = 1;		/*  MAX number of SLCAN channels;
				                This can be overridden with
				                insmod slcan_multimode.ko maxdev=nnn	*/
static int maxbus = 3;
static int binary_mode = 0;
module_param(maxdev, int, 0);
module_param(maxbus, int, 0);
module_param(binary_mode, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of slcan interfaces");
MODULE_PARM_DESC(maxbus, "Maximum number of busses per slcan serial interface");
MODULE_PARM_DESC(binary_mode, "Use binary or normal slcan mode");

/* maximum rx buffer len: extended CAN frame with timestamp */
#define SLC_MTU (sizeof("B0T1111222281122334455667788EA5F\r")+1)
#define SLC_B_MTU 15
/*
 * 2 preamble
 * 4 id + bus |+ type
 * 1 length
 * 8 binary
 */

#define SLC_CMD_LEN 1
#define SLC_BUS_LEN 2
#define SLC_SFF_ID_LEN 3
#define SLC_EFF_ID_LEN 8

struct slcan {
	int			magic;

	/* Various fields. */
	struct tty_struct	*tty;		/* ptr to TTY structure	     */
	struct net_device	**dev;		/* note: multiple devices    */
    int                 devcount;
    int                 active;

	struct work_struct	tx_work;	/* Flushes transmit buffer   */

	spinlock_t		    lock;       // global lock

	/* These are pointers to the malloc()ed frame buffers. */
	unsigned char		rbuff[SLC_MTU];	/* receiver buffer	     */
	int			        rcount;         /* received chars counter    */
	unsigned char		xbuff[SLC_MTU];	/* transmitter buffer	     */
	unsigned char		*xhead;         /* pointer to next XMIT byte */
	int			        xleft;          /* bytes left in XMIT queue  */

	unsigned long		flags;		/* Flag values/ mode etc     */
#define SLF_INUSE		0		/* Channel in use            */
#define SLF_ERROR		1               /* Parity, etc. error        */
};

struct slcan_bus {
	struct net_device	*dev;		/* note: single device    */
    struct slcan        *sl;
    int id;
};

struct slcan_binary {
    uint16_t preamble;
    struct {
        uint32_t bus : 2;
        uint32_t rtr : 1;
        uint32_t id : 29;
    };
    uint8_t len;
    uint8_t data[8];
};


static struct net_device **slcan_net;
static struct slcan **slcan_devs;

 /************************************************************************
  *			SLCAN ENCAPSULATION FORMAT			 *
  ************************************************************************/

/*
 * A CAN frame has a can_id (11 bit standard frame format OR 29 bit extended
 * frame format) a data length code (len) which can be from 0 to 8
 * and up to <len> data bytes as payload.
 * Additionally a CAN frame may become a remote transmission frame if the
 * RTR-bit is set. This causes another ECU to send a CAN frame with the
 * given can_id.
 *
 * The SLCAN ASCII representation of these different frame types is:
 * <type> <id> <dlc> <data>*
 *
 * Extended frames (29 bit) are defined by capital characters in the type.
 * RTR frames are defined as 'r' types - normal frames have 't' type:
 * t => 11 bit data frame
 * r => 11 bit RTR frame
 * T => 29 bit data frame
 * R => 29 bit RTR frame
 *
 * The <id> is 3 (standard) or 8 (extended) bytes in ASCII Hex (base64).
 * The <dlc> is a one byte ASCII number ('0' - '8')
 * The <data> section has at much ASCII Hex bytes as defined by the <dlc>
 *
 * Examples:
 *
 * t1230 : can_id 0x123, len 0, no data
 * t4563112233 : can_id 0x456, len 3, data 0x11 0x22 0x33
 * T12ABCDEF2AA55 : extended can_id 0x12ABCDEF, len 2, data 0xAA 0x55
 * r1230 : can_id 0x123, len 0, no data, remote transmission request
 *
 */

 /************************************************************************
  *			STANDARD SLCAN DECAPSULATION			 *
  ************************************************************************/

/* Send one completely decapsulated can_frame to the network layer */
static void slc_bump(struct slcan *sl)
{
	struct sk_buff *skb;
	struct can_frame cf;
	int i, tmp, bus, busoffset;
	u32 tmpid;
	char *cmd = sl->rbuff;
    struct slcan_binary slb;

	memset(&cf, 0, sizeof(cf));

    if (!binary_mode) {

        switch (*cmd) {
            case 'B':
                bus = (*(cmd+1)) - 0x30;
                busoffset = SLC_BUS_LEN;
                cmd += SLC_BUS_LEN;
                break;
            default:
                bus = 0;
                busoffset = 0;
                break; // behave like a normal SLCAN frame and send that
        }

        if (bus > sl->devcount) {
            pr_err("slcan mm: Bus requested is higher than available busses: %d vs %d\n", bus, sl->devcount);
            return;
        }

        pr_info("slcan mm: received bus %d type %c\n", bus, *cmd);
        switch (*cmd) {
            case 'r':
                cf.can_id = CAN_RTR_FLAG;
                fallthrough;
            case 't':
                /* store dlc ASCII value and terminate SFF CAN ID string */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
                cf.can_dlc = sl->rbuff[busoffset + SLC_CMD_LEN + SLC_SFF_ID_LEN];
#else
                cf.len = sl->rbuff[busoffset + SLC_CMD_LEN + SLC_SFF_ID_LEN];
#endif
                sl->rbuff[busoffset + SLC_CMD_LEN + SLC_SFF_ID_LEN] = 0;
                /* point to payload data behind the dlc */
                cmd += SLC_CMD_LEN + SLC_SFF_ID_LEN + 1;
                break;
            case 'R':
                cf.can_id = CAN_RTR_FLAG;
                fallthrough;
            case 'T':
                cf.can_id |= CAN_EFF_FLAG;
                /* store dlc ASCII value and terminate EFF CAN ID string */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
                cf.can_dlc = sl->rbuff[busoffset + SLC_CMD_LEN + SLC_EFF_ID_LEN];
#else
                cf.len = sl->rbuff[busoffset + SLC_CMD_LEN + SLC_EFF_ID_LEN];
#endif
                sl->rbuff[SLC_CMD_LEN + SLC_EFF_ID_LEN] = 0;
                /* point to payload data behind the dlc */
                cmd += SLC_CMD_LEN + SLC_EFF_ID_LEN + 1;
                break;
            default:
                return;
        }

        if (kstrtou32(sl->rbuff + SLC_CMD_LEN + busoffset, 16, &tmpid))
            return;

        cf.can_id |= tmpid;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
        if (cf.can_dlc >= '0' && cf.can_dlc < '9')
            cf.can_dlc -= '0';
        else
            return;
#else
        /* get len from sanitized ASCII value */
        if (cf.len >= '0' && cf.len < '9')
            cf.len -= '0';
        else
            return;
#endif

        /* RTR frames may have a dlc > 0 but they never have any data bytes */
        if (!(cf.can_id & CAN_RTR_FLAG)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
            for (i = 0; i < cf.can_dlc; i++) {
#else
            for (i = 0; i < cf.len; i++) {
#endif
                tmp = hex_to_bin(*cmd++);
                if (tmp < 0)
                    return;
                cf.data[i] = (tmp << 4);
                tmp = hex_to_bin(*cmd++);
                if (tmp < 0)
                    return;
                cf.data[i] |= tmp;
            }
        }
    } else {
        // binary mode
        memcpy(&slb, cmd, SLC_B_MTU);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
        cf.can_dlc = slb.len;
#else
        cf.len = slb.len;
#endif
        bus = (slb.id & 0xC0000000) >> 29;
        if ((slb.id & 0x1FFFFFFF) > 0x7FF) cf.can_id |= CAN_EFF_FLAG;
        if (slb.id & 0x20000000) cf.can_id = CAN_RTR_FLAG;
        cf.can_id |= slb.id;
        memcpy(cf.data, slb.data, slb.len);
    }

	skb = dev_alloc_skb(sizeof(struct can_frame) +
			    sizeof(struct can_skb_priv));
	if (!skb)
		return;

	skb->dev = sl->dev[bus];
	skb->protocol = htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = sl->dev[bus]->ifindex;
	can_skb_prv(skb)->skbcnt = 0;

	skb_put_data(skb, &cf, sizeof(struct can_frame));

	sl->dev[bus]->stats.rx_packets++;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
	sl->dev[bus]->stats.rx_bytes += cf.can_dlc;
#else
	sl->dev[bus]->stats.rx_bytes += cf.len;
#endif
	netif_rx_ni(skb);
}

/* parse tty input stream */
static void slcan_unesc(struct slcan *sl, unsigned char s)
{
    int i;
    if (!binary_mode) {
        if ((s == '\r') || (s == '\a')) { /* CR or BEL ends the pdu */
            if (!test_and_clear_bit(SLF_ERROR, &sl->flags) &&
                (sl->rcount > 4))  {
                slc_bump(sl);
            }
            sl->rcount = 0;
        } else {
            if (!test_bit(SLF_ERROR, &sl->flags))  {
                if (sl->rcount < SLC_MTU)  {
                    sl->rbuff[sl->rcount++] = s;
                    return;
                } else {
                    for (i = 0; i < sl->devcount; i++) sl->dev[i]->stats.rx_over_errors++;
                    set_bit(SLF_ERROR, &sl->flags);
                }
            }
        }
    } else {
        // preamble detected, pushing
        if (sl->rcount < SLC_MTU && sl->rbuff[0] == 0xAA && sl->rbuff[1] == 0x55) {
            sl->rbuff[sl->rcount++] = s;
        } else if (sl->rbuff[0] != 0xAA) {
            // clear buffer
            sl->rcount = 0;
            if (s == 0xAA) {
                sl->rbuff[sl->rcount++] = s;
            }
        } else if (sl->rbuff[1] != 0x55) {
            if (s == 0x55) {
                sl->rbuff[sl->rcount++] = s;
            } else {
                sl->rcount = 0;
            }
        } else if (sl->rcount == SLC_B_MTU) {
            if (!test_and_clear_bit(SLF_ERROR, &sl->flags)) {
                slc_bump(sl);
            }
            sl->rcount = 0;
        } else {
            // overflow
            for (i = 0; i < sl->devcount; i++) sl->dev[i]->stats.rx_over_errors++;
            set_bit(SLF_ERROR, &sl->flags);
        }
    }
}

 /************************************************************************
  *			STANDARD SLCAN ENCAPSULATION			 *
  ************************************************************************/

/* Encapsulate one can_frame and stuff into a TTY queue. */
static void slc_encaps(struct slcan *sl, struct can_frame *cf, struct net_device *nd)
{
	int actual, i;
	unsigned char *pos;
	unsigned char *endpos;
	canid_t id = cf->can_id;

	pos = sl->xbuff;

	if (cf->can_id & CAN_RTR_FLAG)
		*pos = 'R'; /* becomes 'r' in standard frame format (SFF) */
	else
		*pos = 'T'; /* becomes 't' in standard frame format (SSF) */

	/* determine number of chars for the CAN-identifier */
	if (cf->can_id & CAN_EFF_FLAG) {
		id &= CAN_EFF_MASK;
		endpos = pos + SLC_EFF_ID_LEN;
	} else {
		*pos |= 0x20; /* convert R/T to lower case for SFF */
		id &= CAN_SFF_MASK;
		endpos = pos + SLC_SFF_ID_LEN;
	}

	/* build 3 (SFF) or 8 (EFF) digit CAN identifier */
	pos++;
	while (endpos >= pos) {
		*endpos-- = hex_asc_upper[id & 0xf];
		id >>= 4;
	}

	pos += (cf->can_id & CAN_EFF_FLAG) ? SLC_EFF_ID_LEN : SLC_SFF_ID_LEN;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
	*pos++ = cf->can_dlc + '0';
#else
	*pos++ = cf->len + '0';
#endif

	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
		for (i = 0; i < cf->can_dlc; i++)
#else
		for (i = 0; i < cf->len; i++)
#endif
			pos = hex_byte_pack_upper(pos, cf->data[i]);
	}

	*pos++ = '\r';

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	actual = sl->tty->ops->write(sl->tty, sl->xbuff, pos - sl->xbuff);
	sl->xleft = (pos - sl->xbuff) - actual;
	sl->xhead = sl->xbuff + actual;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,11,0)
	nd->stats.tx_bytes += cf->can_dlc;
#else
	nd->stats.tx_bytes += cf->len;
#endif
}


static void slc_open_bus(struct slcan *sl, int bus,struct net_device *nd)
{
	int actual;
	unsigned char *pos;

	pos = sl->xbuff;

    *pos++ = 'D';
    *pos++ = 0x30 + binary_mode;
    *pos++ = '\r';
    *pos++ = 'B';
    *pos++ = 0x30 + bus;
    *pos++ = 'O';
	*pos++ = '\r';

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	actual = sl->tty->ops->write(sl->tty, sl->xbuff, pos - sl->xbuff);
	sl->xleft = (pos - sl->xbuff) - actual;
	sl->xhead = sl->xbuff + actual;
	nd->stats.tx_bytes += 4;
}

static void slc_close_bus(struct slcan *sl, int bus, struct net_device *nd)
{
	int actual;
	unsigned char *pos;

	pos = sl->xbuff;

    *pos++ = 'B';
    *pos++ = 0x30 + bus;
    *pos++ = 'C';
	*pos++ = '\r';

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	actual = sl->tty->ops->write(sl->tty, sl->xbuff, pos - sl->xbuff);
	sl->xleft = (pos - sl->xbuff) - actual;
	sl->xhead = sl->xbuff + actual;
	nd->stats.tx_bytes += 4;
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void slcan_transmit(struct work_struct *work)
{
    int netdevrun = 0;
    int i;
	struct slcan *sl = container_of(work, struct slcan, tx_work);
	int actual;

	spin_lock_bh(&sl->lock);

    for (i = 0; i < sl->devcount; i++) netdevrun |= netif_running(sl->dev[i]);

	/* First make sure we're connected. */
	if (!sl->tty || sl->magic != SLCAN_MAGIC || !netdevrun) {
        spin_unlock_bh(&sl->lock);
        pr_info("slcan mm: not connected\n");
		return;
	}

	if (sl->xleft <= 0)  {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet */
        sl->dev[sl->active]->stats.tx_packets++;
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
		spin_unlock_bh(&sl->lock);
        netif_wake_queue(sl->dev[sl->active]);
        pr_info("slcan mm: Waking up netqueue\n");
		return;
	}

	actual = sl->tty->ops->write(sl->tty, sl->xhead, sl->xleft);
	sl->xleft -= actual;
	sl->xhead += actual;
	spin_unlock_bh(&sl->lock);
    pr_info("slcan mm: Transmitted data to tty\n");
}

/*
 * Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void slcan_write_wakeup(struct tty_struct *tty)
{
	struct slcan *sl;

	rcu_read_lock();
	sl = rcu_dereference(tty->disc_data);
	if (sl)
		schedule_work(&sl->tx_work);
	rcu_read_unlock();
}

/* Send a can_frame to a TTY queue. */
static netdev_tx_t slc_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct slcan_bus *bus = netdev_priv(dev);

	if (skb->len != CAN_MTU)
		goto out;

	spin_lock(&bus->sl->lock);
	if (!netif_running(dev))  {
		spin_unlock(&bus->sl->lock);
		printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
		goto out;
	}
	if (bus->sl->tty == NULL) {
		spin_unlock(&bus->sl->lock);
		goto out;
	}
	netif_stop_queue(bus->dev);
    bus->sl->active = bus->id;
	slc_encaps(bus->sl, (struct can_frame *) skb->data, dev); /* encaps & send */
	spin_unlock(&bus->sl->lock);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */
static int slc_close(struct net_device *dev)
{
	struct slcan_bus *bus = netdev_priv(dev);

	// close first
    slc_close_bus(bus->sl, bus->id, dev);

	spin_lock_bh(&bus->sl->lock);

	if (bus->sl->tty) {
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &bus->sl->tty->flags);
	}
    // send close command

	netif_stop_queue(dev);
	bus->sl->rcount   = 0;
	bus->sl->xleft    = 0;
	spin_unlock_bh(&bus->sl->lock);
    pr_info("slcan mm: netdev %s went down\n", dev->name);
	return 0;
}

/* Netdevice DOWN -> UP routine */
static int slc_open(struct net_device *dev)
{
	struct slcan_bus *bus = netdev_priv(dev);

	if (bus->sl->tty == NULL)
		return -ENODEV;

	bus->sl->flags &= (1 << SLF_INUSE);
	spin_lock_bh(&bus->sl->lock);
    slc_open_bus(bus->sl, bus->id, dev);
	spin_unlock_bh(&bus->sl->lock);
	netif_start_queue(dev);
    pr_info("slcan mm: netdev %s went up\n", dev->name);
	return 0;
}

/* Hook the destructor so we can free slcan devs at the right point in time */
static void slc_free_netdev(struct net_device *dev)
{
	int i = dev->base_addr;

	slcan_net[i] = NULL;
}

static int slcan_change_mtu(struct net_device *dev, int new_mtu)
{
	return -EINVAL;
}

static const struct net_device_ops slc_netdev_ops = {
	.ndo_open               = slc_open,
	.ndo_stop               = slc_close,
	.ndo_start_xmit         = slc_xmit,
	.ndo_change_mtu         = slcan_change_mtu,
};

static void slc_setup(struct net_device *dev)
{
	dev->netdev_ops		= &slc_netdev_ops;
	dev->needs_free_netdev	= true;
	dev->priv_destructor	= slc_free_netdev;

	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;

	dev->mtu		= CAN_MTU;
	dev->type		= ARPHRD_CAN;

	/* New-style flags. */
	dev->flags		= IFF_NOARP;
	dev->features           = NETIF_F_HW_CSUM;
}

/******************************************
  Routines looking at TTY side.
 ******************************************/

/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of SLCAN data has been received, which can now be decapsulated
 * and sent on to some IP layer for further processing. This will not
 * be re-entered while running but other ldisc functions may be called
 * in parallel
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static void slcan_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, char *fp,
			      int count)
#else
static void slcan_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, const char *fp,
			      int count)
#endif
{
    int netdefrun = 0, i = 0;
	struct slcan *sl = (struct slcan *) tty->disc_data;


    for (i = 0; i < sl->devcount; i++) netdefrun |= netif_running(sl->dev[i]);

	if (!sl || sl->magic != SLCAN_MAGIC || !netdefrun)
		return;

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) {
			if (!test_and_set_bit(SLF_ERROR, &sl->flags)) {
				for (i = 0; i < sl->devcount; i++) sl->dev[i]->stats.rx_errors++;
            }
			cp++;
			continue;
		}
		slcan_unesc(sl, *cp++);
	}
}

/************************************
 *  slcan_open helper routines.
 ************************************/

/* Collect hanged up channels */
static void slc_sync(void)
{
	int i;
	struct net_device *dev;
	struct slcan_bus  *bus;

	for (i = 0; i < maxdev * maxbus; i++) {
		dev = slcan_net[i];
		if (dev == NULL)
			break;

		bus = netdev_priv(dev);
		if (bus->sl->tty)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}

/* Find a free SLCAN channel, and link in this `tty' line. */
static struct slcan *slc_alloc(void)
{
	int i, j;
	char name[IFNAMSIZ];
	struct net_device *dev = NULL;
	struct can_ml_priv *can_ml;
	struct slcan        *sl;
    struct slcan_bus    *bus;
	int size;

    pr_info("slcan mm: allocating new busses\n");
	for (i = 0; i < maxdev; i++) {
		sl = slcan_devs[i];
		if (sl == NULL)
			break;
    }

    if (i > maxdev) {
        pr_err("slcan mm: No slcan slots available\n");
        return NULL;
    }

    // assign slcan data
    sl = kcalloc(1, sizeof(struct slcan), GFP_KERNEL);

    if (NULL == sl) return NULL;

    sl->dev = kcalloc(maxbus, sizeof(struct net_device *), GFP_KERNEL);

    if (NULL == sl->dev) {
        kfree(sl);
        return NULL;
    }


	for (i = 0; i < maxdev * maxbus; i++) {
		dev = slcan_net[i];
		if (dev != NULL)
			break;

	/* Sorry, too many, all slots in use */
        if (i > (maxdev -1) * maxbus) {
            pr_err("slcan mm: Not enough netslots available\n");
            return NULL;
        }

        sl->magic = SLCAN_MAGIC;
        spin_lock_init(&sl->lock);
        INIT_WORK(&sl->tx_work, slcan_transmit);

        for (j = 0; j < maxbus; j++) {
            sprintf(name, "slcan%dd%d", i, j);
            size = ALIGN(sizeof(*bus), NETDEV_ALIGN) + sizeof(struct can_ml_priv);
            dev = alloc_netdev(size, name, NET_NAME_UNKNOWN, slc_setup);
            if (!dev)
                return NULL;
            dev->base_addr  = i+j;
            bus = netdev_priv(dev);
            can_ml = (void *)bus + ALIGN(sizeof(*bus), NETDEV_ALIGN);
            can_set_ml_priv(dev, can_ml);

            /* Initialize channel control data */
            bus->sl = sl;
            bus->id = j;
            sl->dev[j]	= dev;
            sl->devcount++;
            slcan_net[i] = dev;
        }
        i += j;

    }

	return sl;
}

/*
 * Open the high-level part of the SLCAN channel.
 * This function is called by the TTY module when the
 * SLCAN line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free SLCAN channel...
 *
 * Called in process context serialized from other ldisc calls.
 */

static int slcan_open(struct tty_struct *tty)
{
	struct slcan *sl;
	int err;
    int i;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	pr_info("slcan mm: opening busses\n");
	/* RTnetlink lock is misused here to serialize concurrent
	   opens of slcan channels. There are better ways, but it is
	   the simplest one.
	 */
	rtnl_lock();

	/* Collect hanged up channels. */
	slc_sync();

	sl = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (sl && sl->magic == SLCAN_MAGIC)
		goto err_exit;

	/* OK.  Find a free SLCAN channel to use. */
	err = -ENFILE;
	sl = slc_alloc();
	if (sl == NULL)
		goto err_exit;

	sl->tty = tty;
	tty->disc_data = sl;

	if (!test_bit(SLF_INUSE, &sl->flags)) {
		/* Perform the low-level SLCAN initialization. */
		sl->rcount   = 0;
		sl->xleft    = 0;

		set_bit(SLF_INUSE, &sl->flags);

		for (i = 0; i < sl->devcount; i++) {
            pr_info("slcan mm: registering %8s\n", sl->dev[i]->name);
            err = register_netdevice(sl->dev[i]);
            if (err)
                goto err_free_chan;
            pr_info("slcan mm: registered %8s\n", sl->dev[i]->name);
        }
	}

	/* Done.  We have linked the TTY line to a channel. */
	rtnl_unlock();
	tty->receive_room = 65536;	/* We don't flow control */

	/* TTY layer expects 0 on success */
	return 0;

err_free_chan:
	sl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &sl->flags);
	for (i = 0; i < sl->devcount; i++) slc_free_netdev(sl->dev[i]);
	/* do not call free_netdev before rtnl_unlock */
	rtnl_unlock();
	for (i = 0; i < sl->devcount; i++) free_netdev(sl->dev[i]);
	return err;

err_exit:
	rtnl_unlock();

	/* Count references from TTY module */
	return err;
}

/*
 * Close down a SLCAN channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions.
 *
 * We also use this method for a hangup event.
 */

static void slcan_close(struct tty_struct *tty)
{
    int i;
	struct slcan *sl = (struct slcan *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLCAN_MAGIC || sl->tty != tty)
		return;

	spin_lock_bh(&sl->lock);
	rcu_assign_pointer(tty->disc_data, NULL);
	sl->tty = NULL;
	spin_unlock_bh(&sl->lock);

	synchronize_rcu();
    flush_work(&sl->tx_work);

	pr_info("slcan mm: closing busses\n");
	/* Flush network side */
	for (i = 0; i < sl->devcount; i++) unregister_netdev(sl->dev[i]);
	/* This will complete via sl_free_netdev */
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,0)
static int slcan_hangup(struct tty_struct *tty)
#else
static void slcan_hangup(struct tty_struct *tty)
#endif
{
	slcan_close(tty);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,0)
    return 0;
#endif
}

/* Perform I/O control on an active SLCAN channel. */
static int slcan_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
    int i = 0;
    char *buffer = NULL;
	struct slcan *sl = (struct slcan *) tty->disc_data;
	unsigned int tmp, tmp2;

	/* First make sure we're connected. */
	if (!sl || sl->magic != SLCAN_MAGIC)
		return -EINVAL;

	switch (cmd) {
        case SIOCGIFNAME:
            tmp = 0;
            for (i = 0; i < sl->devcount; i++) {
                tmp += strlen(sl->dev[i]->name) + 1;
            }
            
            buffer = kcalloc(tmp + 1, sizeof(char), GFP_KERNEL);

            if (NULL == buffer) {
                return -ENOMEM;
            }

            tmp2 = 0;
            for (i = 0; i < sl->devcount; i++) {
                tmp2 += snprintf(&buffer[tmp2], tmp, "%s,", sl->dev[i]->name);
            }
            buffer[tmp2-1] = '\0';

            pr_info("Found %s netdevices\n", buffer);
            if (copy_to_user((void __user *)arg, buffer, tmp2 - 1)) {
                kfree(buffer);
                return -EFAULT;
            }
            kfree(buffer);
            return 0;

	case SIOCSIFHWADDR:
		return -EINVAL;

	default:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,16,0)
		return tty_mode_ioctl(tty, file, cmd, arg);
#else
		return tty_mode_ioctl(tty, cmd, arg);
#endif
	}
}

static struct tty_ldisc_ops slc_ldisc = {
	.owner		= THIS_MODULE,
	.num		= N_SLCAN,
	.name		= "slcan",
	.open		= slcan_open,
	.close		= slcan_close,
	.hangup		= slcan_hangup,
	.ioctl		= slcan_ioctl,
	.receive_buf	= slcan_receive_buf,
	.write_wakeup	= slcan_write_wakeup,
};

static int __init slcan_init(void)
{
	int status;

    if (maxbus < 3) {
        maxbus = 3;
    }

	if (maxdev < 1) {
		maxdev = 1; /* Sanity */
    }

	pr_info("slcan mm: serial line CAN interface driver\n");
	pr_info("slcan mm: %d dynamic interface channels with %d busses each.\n", maxdev, maxbus);

	slcan_net = kcalloc(maxdev * maxbus, sizeof(struct net_device *), GFP_KERNEL);
	if (!slcan_net)
		return -ENOMEM;

	slcan_devs = kcalloc(maxdev, sizeof(struct slcan *), GFP_KERNEL);
	if (!slcan_devs) {
        kfree(slcan_net);
		return -ENOMEM;
    }

	/* Fill in our line protocol discipline, and register it */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,14,0)
	status = tty_register_ldisc(N_SLCAN, &slc_ldisc);
#else
	status = tty_register_ldisc(&slc_ldisc);
#endif
	if (status)  {
		printk(KERN_ERR "slcan: can't register line discipline\n");
		kfree(slcan_net);
        kfree(slcan_devs);
	}
	return status;
}

static void __exit slcan_exit(void)
{
	int i, j;
	struct net_device *dev;
	struct slcan_bus *bus;
	unsigned long timeout = jiffies + HZ;
	int busy = 0;
    struct slcan *sl;

	if (slcan_net == NULL)
		return;

	/* First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < maxdev; i++) {
			sl = slcan_devs[i];
			if (!sl)
				continue;
			spin_lock_bh(&sl->lock);
			if (sl->tty) {
				busy++;
				tty_hangup(sl->tty);
			}
			spin_unlock_bh(&sl->lock);
		}
	} while (busy && time_before(jiffies, timeout));

	/* FIXME: hangup is async so we should wait when doing this second
	   phase */

    // check like in slcan
    for (i = 0; i < maxdev; i++) {
        sl = slcan_devs[i];
        if (!sl) continue;
		if (sl->tty) {
            for (j = 0; i < sl->devcount; j++) {
    			printk(KERN_ERR "%s: tty discipline still running\n",
    			       sl->dev[j]->name);
            }
		}
        kfree(sl); // free anyway
    }

	for (i = 0; i < maxdev * maxbus; i++) {
		dev = slcan_net[i];
		if (!dev)
			continue;
        pr_info("slcan mm: closing %8s\n", dev->name);
		slcan_net[i] = NULL;

		bus = netdev_priv(dev);

		unregister_netdev(dev);
	}

	kfree(slcan_net);
    kfree(slcan_devs);
	slcan_net = NULL;

    pr_info("slcan mm: unregistering ldisc\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,16,0)
	tty_unregister_ldisc(N_SLCAN);
#else
	tty_unregister_ldisc(&slc_ldisc);
#endif
}

module_init(slcan_init);
module_exit(slcan_exit);
