#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x24d702c7, "module_layout" },
	{ 0x5d87ad84, "usb_serial_generic_tiocmiwait" },
	{ 0xcc12f33e, "usb_serial_deregister_drivers" },
	{ 0x3e4003b1, "usb_serial_register_drivers" },
	{ 0x3bf8f0d, "usb_serial_generic_open" },
	{ 0x30e74134, "tty_termios_copy_hw" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0x6c257ac0, "tty_termios_hw_change" },
	{ 0xaca5b517, "tty_put_char" },
	{ 0x54496b4, "schedule_timeout_interruptible" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x845d17cc, "_dev_info" },
	{ 0x9200756d, "usb_serial_generic_resume" },
	{ 0xf9873a52, "kmem_cache_alloc_trace" },
	{ 0x9f703e67, "kmalloc_caches" },
	{ 0xa2645617, "usb_control_msg" },
	{ 0xc42c119c, "_dev_err" },
	{ 0x33d60ab6, "tty_kref_put" },
	{ 0x1b8b2a7f, "usb_serial_handle_dcd_change" },
	{ 0xc9e78710, "tty_port_tty_get" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xb3142616, "usb_submit_urb" },
	{ 0x6811a622, "usb_kill_urb" },
	{ 0x14bb3b9d, "usb_serial_generic_close" },
	{ 0x37a0cba, "kfree" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x1fdc7df2, "_mcount" },
};

MODULE_INFO(depends, "usbserial");

MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p7522d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2184p0057d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v4348p5523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v9986p7523d*dc*dsc*dp*ic*isc*ip*in*");
