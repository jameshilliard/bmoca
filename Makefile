override EXTRA_CFLAGS += -I$M/../include -DCONFIG_BCM_6802_MoCA=1 -DDSL_MOCA=1

all: modules

obj-m += bmoca.o
