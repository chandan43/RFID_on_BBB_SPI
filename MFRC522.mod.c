#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x87a14630, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xe6909653, __VMLINUX_SYMBOL_STR(driver_unregister) },
	{ 0x4fe282c1, __VMLINUX_SYMBOL_STR(__spi_register_driver) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xa671b895, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x48a4dba3, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0xef2551c6, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xc86d49df, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0x303a0eae, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xd4a9c8dc, __VMLINUX_SYMBOL_STR(cdev_alloc) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0x59a8012b, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0x79c5a9f0, __VMLINUX_SYMBOL_STR(ioremap) },
	{ 0xa9795959, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x6275482, __VMLINUX_SYMBOL_STR(spi_setup) },
	{ 0x27790104, __VMLINUX_SYMBOL_STR(spi_write_then_read) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0x4d2e8e9, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0xe0c82ef0, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x6fd7d068, __VMLINUX_SYMBOL_STR(device_destroy) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "E4B8A6FD8DEEF62F5A63978");
