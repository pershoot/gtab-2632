# Switch Class Driver
obj-$(CONFIG_SWITCH)		+= switch_class.o
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio.o

ifeq ($(CONFIG_7265C_V20),y)
use_h2w_dock_underline :=y
endif
ifeq ($(CONFIG_7564C_V10),y)
use_h2w_dock_underline :=y
endif
ifeq ($(CONFIG_7323C_V21),y)
use_h2w_dock_underline :=y
endif

ifeq ($(use_h2w_dock_underline),y)
obj-$(CONFIG_SWITCH_DOCK)	+= switch_dock_.o
obj-$(CONFIG_SWITCH_H2W)	+= switch_h2w_.o
else
obj-$(CONFIG_SWITCH_DOCK)	+= switch_dock.o
obj-$(CONFIG_SWITCH_H2W)	+= switch_h2w.o
endif

obj-$(CONFIG_SWITCH_HDMI)	+= switch_hdmi.o

