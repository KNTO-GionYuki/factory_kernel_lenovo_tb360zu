#
# Makefile for the Novatek NT36xxx touchscreen driver.
#

# Each configuration option enables a list of files.

#obj-y += nt36xxx.o nt36xxx_fw_update.o nt36xxx_ext_proc.o nt36xxx_mp_ctrlram.o
#linden code for JLINDEN-222 by yangjinmin at 20221130 start
obj-$(CONFIG_TOUCHSCREEN_NT36523N) += nvt_36523n.o

nvt_36523n-y := nt36xxx.o nt36xxx_fw_update.o nt36xxx_ext_proc.o nt36xxx_mp_ctrlram.o
#linden code for JLINDEN-222 by yangjinmin at 20221130 end
