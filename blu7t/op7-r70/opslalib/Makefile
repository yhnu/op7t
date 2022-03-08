SLA_LIB_PATH = $(KBUILD_SRC)/opslalib/slalib
ifeq ($(SLA_LIB_PATH),$(wildcard $(SLA_LIB_PATH)))
obj-$(subst y,$(CONFIG_SLA),$(CONFIG_SLA_ALGO)) += slalib/op_sla_help_lib.o
endif
