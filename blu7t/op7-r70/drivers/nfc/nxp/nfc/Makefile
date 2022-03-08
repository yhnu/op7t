#
# Makefile for nfc devices
#
pn553-objs = nfc.o sn1xx.o pn8xt.o
obj-$(CONFIG_NXP_NFC_ESE_DEVICE) += pn553.o
ccflags-$(CONFIG_NXP_NFC_SN1XX)  := -DNFC_PLATFORM=sn1xx
ccflags-$(CONFIG_NXP_NFC_PN8XT)  := -DNFC_PLATFORM=pn8xt

# obj-y += pn553.o
# ccflags-y  := -DNFC_PLATFORM=pn8xt