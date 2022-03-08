#
# Makefile for the touchscreen drivers.
#
subdir-ccflags-$(CONFIG_TOUCHPANEL_MTK_PLATFORM) += -I$(srctree)/drivers/misc/mediatek/include/

obj-$(CONFIG_TOUCHPANEL_ONEPLUS) += util_interface/
obj-$(CONFIG_TOUCHPANEL_ONEPLUS) += touchpanel_common_driver.o
obj-$(CONFIG_TOUCHPANEL_SYNAPTICS) += Synaptics/
obj-$(CONFIG_TOUCHPANEL_FOCAL) += Focal/
obj-$(CONFIG_TOUCHPANEL_GOODIX) += Goodix/
obj-$(CONFIG_TOUCHPANEL_NOVA) += Novatek/
obj-$(CONFIG_TOUCHPANEL_HIMAX) += Himax/
obj-$(CONFIG_TOUCHPANEL_SAMSUNG) += Samsung/
obj-$(CONFIG_TOUCHPANEL_ST) += St/

