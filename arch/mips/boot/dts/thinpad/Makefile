dtb-$(CONFIG_THINPAD_SPANTAN6)	+= naivemips_thinpad.dtb
dtb-$(CONFIG_THINPAD_NG_ARTIX7)	+= naivemips_thinpad.dtb
dtb-$(CONFIG_DE2I_CYCLONE4)	+= naivemips_de2i.dtb
dtb-$(CONFIG_FPGA_A7_NSCSCC)	+= naivemips_nscscc.dtb
dtb-$(CONFIG_FPGA_K7_EES)	+= naivemips_ees.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files	:= *.dtb *.dtb.S
