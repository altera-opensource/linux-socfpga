obj-y += clk.o
ifeq ($(CONFIG_ARCH_SOCFPGA),y)
obj-y += clk-gate.o
obj-y += clk-pll.o
obj-y += clk-periph.o
obj-y += clk-pll-a10.o clk-periph-a10.o clk-gate-a10.o
else
obj-y += clk-pll-s10.o clk-periph-s10.o clk-gate-s10.o
endif
