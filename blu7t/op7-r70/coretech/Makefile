obj-$(CONFIG_OPCHAIN) += uxcore/
CORE_PATH = $(KBUILD_SRC)/coretech/uxcore/core
ifeq ($(CORE_PATH),$(wildcard $(CORE_PATH)))
obj-$(CONFIG_OPCHAIN) += uxcore/core/
endif

obj-$(CONFIG_MEMPLUS) += memplus/
CORE_PATH = $(KBUILD_SRC)/coretech/memplus/core
ifeq ($(CORE_PATH),$(wildcard $(CORE_PATH)))
obj-$(CONFIG_MEMPLUS) += memplus/core/
endif

obj-$(CONFIG_SMART_BOOST) += smartboost/
CORE_PATH = $(KBUILD_SRC)/coretech/smartboost/core
ifeq ($(CORE_PATH),$(wildcard $(CORE_PATH)))
obj-$(CONFIG_SMART_BOOST) += smartboost/core/
endif

obj-$(CONFIG_DEFRAG) += defrag/
CORE_PATH = $(KBUILD_SRC)/coretech/defrag/core
ifeq ($(CORE_PATH),$(wildcard $(CORE_PATH)))
obj-$(CONFIG_DEFRAG) += defrag/core/
endif

obj-$(CONFIG_FSC) += fsc/
obj-$(CONFIG_HOUSTON) += houston/
obj-$(CONFIG_CONTROL_CENTER) += control_center/
obj-$(CONFIG_AIGOV) += aigov/
