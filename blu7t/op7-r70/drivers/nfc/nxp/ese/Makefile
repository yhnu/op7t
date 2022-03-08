#
# Makefile for ese devices
#
p73-objs = ese.o sn1xx.o pn8xt.o
obj-$(CONFIG_NXP_NFC_ESE_DEVICE)    += p73.o
ccflags-$(CONFIG_NXP_ESE_SN1XX)     := -DNFC_PLATFORM=sn1xx
ccflags-$(CONFIG_NXP_ESE_PN8XT)     := -DNFC_PLATFORM=pn8xt

# obj-y += p73.o
# ccflags-y  := -DNFC_PLATFORM=pn8xt