#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.
ifeq ($(OP_TARGET_DEVICE),MSM_17001)
obj-$(CONFIG_TOUCHPANEL_SYNAPTICS_S3320) += synaptics_drivers_s3320_17001.o
else ifeq ($(OP_TARGET_DEVICE),MSM_17151)
obj-$(CONFIG_TOUCHPANEL_SYNAPTICS_S3320) += synaptics_drivers_s3320_17001.o
else
obj-$(CONFIG_TOUCHPANEL_SYNAPTICS_S3320) += synaptics_drivers_s3320.o
endif
#endif
