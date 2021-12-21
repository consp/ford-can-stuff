#include <linux/init.h>
#include <linux/slab.h> 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>  
#include <linux/tty.h>
#include <linux/version.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tristan Timmermans");
MODULE_DESCRIPTION("SLCAN MultiMode Single TTY to multiple TTY");
MODULE_VERSION("0.01");

#define SLCAN_MM_MAGIC 0x53CB
#define SLC_MM_TTY_MAJOR 599

/* maximum rx buffer len: extended CAN frame with timestamp */
#define SLC_MTU (sizeof("B0T1111222281122334455667788EA5F\r")+1)

#define SLC_CMD_LEN 1
#define SLC_SFF_ID_LEN 3
#define SLC_EFF_ID_LEN 8

static int maxdev = 3;

struct slc_mm_can_port {
    struct tty_struct   *tty;
    struct tty_port     port;
    int                 open_count; 
    int                 initialized;
    struct mutex        mutex;
    uint8_t             readbuf[SLC_MTU];
    uint8_t             writebuf[SLC_MTU];
};

static struct slc_mm_can_port *slc_mm_can_ports;


module_param(maxdev, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of busses to register as tty");


static int slc_mm_open(struct tty_struct *tty, struct file *file) {
    struct slc_mm_can_port *port;
    int index = 0;
    int rval;
    
    printk(KERN_INFO "opening");
    tty->driver_data = NULL;
    
    index = tty->index;
    printk(KERN_WARNING "opening %d", index);
    port = &slc_mm_can_ports[index];
    
    if (!port->initialized) {
        /* first time accessing this device, let's create it */

        printk(KERN_WARNING "initializing %d", index);
        mutex_init(&port->mutex);
        port->open_count = 0;       
        port->initialized = 1;
    }

    mutex_lock(&port->mutex);

    tty->driver_data = port;
    port->tty = tty;
    
    mutex_unlock(&port->mutex);
    return 0;
}

static void slc_mm_close(struct tty_struct *tty, struct file *file) {
    struct slc_mm_can_port *port = tty->driver_data;
    
    printk(KERN_INFO "closing");

    if (port) {
        mutex_lock(&port->mutex);
        
        if (!port->open_count) {
            mutex_unlock(&port->mutex);
            return;
        }
        
        port->open_count--;
        if (port->open_count <=0) {
            printk(KERN_INFO "Closing port %d", tty->index);
        }
        mutex_unlock(&port->mutex);
    }
}

static int slc_mm_write(struct tty_struct *tty, const unsigned char *buffer, int count) {
    return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)) 
static int slc_mm_write_room(struct tty_struct *tty) {
#else
static unsigned int slc_mm_write_room(struct tty_struct *tty) {
#endif
    return 0;
}

static struct tty_operations serial_ops = {
    .open = slc_mm_open,
    .close = slc_mm_close,
    .write = slc_mm_write,
    .write_room = slc_mm_write_room,
    //.set_termios = slc_mm_set_termios,
};

static const struct tty_port_operations slc_mm_port_ops = {
	.activate = NULL, //slc_mm_port_activate,
	.shutdown = NULL, //slc_mm_port_shutdown,
};

static struct tty_driver *slc_tty_slave;

static int __init slc_mm_init(void) {
    struct tty_driver *driver;
    int retval;
    int i;
    struct device	*tty_dev;

    printk(KERN_INFO "Starting SLCAN MultiMode Driver");

    
    //devfs_mk_dir("ttySLCMM");
    
    driver = tty_alloc_driver(maxdev, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
    
    if (IS_ERR(driver))
		return PTR_ERR(driver);
    
    slc_tty_slave = driver;
    slc_tty_slave->owner = THIS_MODULE;
    slc_tty_slave->driver_name = "slc_mm_tty";
    slc_tty_slave->name = "ttyCAN";
    //slc_tty_slave->devfs_name = "ttySLCMM/CAN%d";
    //slc_tty_slave->major = SLC_MM_TTY_MAJOR,
    slc_tty_slave->type = TTY_DRIVER_TYPE_SERIAL,
    slc_tty_slave->subtype = SERIAL_TYPE_NORMAL,
    slc_tty_slave->init_termios = tty_std_termios;
    slc_tty_slave->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
    tty_set_operations(slc_tty_slave, &serial_ops);
    
    // alloc slc_mm ports
    
    slc_mm_can_ports = kcalloc(maxdev, sizeof(struct slc_mm_can_port), GFP_KERNEL);
    
    if (IS_ERR(slc_mm_can_ports)) {
        return PTR_ERR(driver);
    }
    
    for (i = 0; i < maxdev; i++) {
        tty_port_init(&slc_mm_can_ports[i].port);
		tty_port_link_device(&slc_mm_can_ports[i].port, slc_tty_slave, i);
    }
    
    retval = tty_register_driver(slc_tty_slave);
    
    if (retval) {
        printk(KERN_ERR "failed to register SLC_MM tty driver");
        put_tty_driver(slc_tty_slave);
        return retval;
    }
    
    for (i = 0; i < maxdev; i++) {
        tty_dev = tty_register_device(slc_tty_slave, i, NULL);
        if (IS_ERR(tty_dev)) {
            printk(KERN_INFO "Unregistering device attempt");
            tty_unregister_device(slc_tty_slave, i);
        }
    }
    
    printk(KERN_INFO "Starting SLCAN MultiMode Driver initialized");

    return 0;
}

static void __exit slc_mm_exit(void) {
    int i;
    //unregister_chrdev(major_num, DEVICE_NAME);
    for (i = 0; i < maxdev; i++) tty_unregister_device(slc_tty_slave, i);
    tty_unregister_driver(slc_tty_slave);
    kfree(slc_mm_can_ports);
    printk(KERN_INFO "Ending SLCAN MultiMode Driver");
}

module_init(slc_mm_init);
module_exit(slc_mm_exit);
