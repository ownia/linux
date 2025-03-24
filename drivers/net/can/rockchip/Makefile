# SPDX-License-Identifier: GPL-2.0

obj-m += rockchip_canfd.o

rockchip_canfd-objs :=
rockchip_canfd-objs += rockchip_canfd-core.o
rockchip_canfd-objs += rockchip_canfd-ethtool.o
rockchip_canfd-objs += rockchip_canfd-rx.o
rockchip_canfd-objs += rockchip_canfd-timestamp.o
rockchip_canfd-objs += rockchip_canfd-tx.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
