occ-hwmon-objs := common.o

ifeq ($(CONFIG_SENSORS_OCC_P9_SBE), y)
occ-hwmon-objs += p9.o p9_sbe.o
endif

obj-$(CONFIG_SENSORS_OCC) += occ-hwmon.o
