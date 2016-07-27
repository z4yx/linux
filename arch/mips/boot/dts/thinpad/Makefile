dtb-$(CONFIG_THINPAD_SPANTAN6)	+= naivemips_thinpad.dtb
dtb-$(CONFIG_DE2I_CYCLONE4)	+= naivemips_de2i.dtb
dtb-$(CONFIG_DE1SOC_CYCLONE5)	+= naivemips_de1soc.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files	:= *.dtb *.dtb.S
