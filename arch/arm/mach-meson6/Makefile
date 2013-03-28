#
# Makefile for the linux kernel.
#

# Common support
obj-y := cpu.o clock.o gpio.o time.o usbclock.o usbsetting.o power_gate.o mod_gate.o
obj-m :=
obj-n :=
obj-  :=

# Power Management
obj-$(CONFIG_SUSPEND)		+= pm.o

ifeq ($(CONFIG_MESON_PLATFORM_API),y)
plat-api=$(wildcard $(wildcard $(srctree)/$(src)/boards/*.c))
plat-api:=$(plat-api:$(srctree)/$(src)/%.c=%.o)
obj-y +=$(plat-api)
else
obj-$(CONFIG_MACH_MESON6_REF)	+= board-meson6-ref.o
obj-$(CONFIG_MACH_MESON6_REF)	+= board-m6ref-panel.o
obj-$(CONFIG_MACH_MESON6_SKT)	+= board-meson6-skt.o
obj-$(CONFIG_MACH_MESON6_SKT)	+= board-m6skt-panel.o
endif
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o


obj-$(CONFIG_CARDREADER)	+= sd_pad.o
#obj-$(CONFIG_CARDREADER)	+= gpio.o

obj-$(CONFIG_MESON_SUSPEND) += power_suspend.o
obj-$(CONFIG_SUSPEND) += sleep.o

obj-$(CONFIG_MESON_CS_DCDC_REGULATOR) += voltage.o
