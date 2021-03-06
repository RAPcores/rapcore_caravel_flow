PROJ_ROOT = ../../../../..
FIRMWARE_PATH = ../..
VERILOG_PATH = ../../../..
RTL_PATH = ../../../../rtl
IP_PATH = ../../../../ip
BEHAVIOURAL_MODELS = ../../
RAPCORES_PATH = ../../../../../rapcores
GL_PATH = $(PROJ_ROOT)/openlane/rapcores/runs/rapcores/results/lvs/




GCC_PATH?=//opt/riscv32/bin
GCC_PREFIX?=riscv32-unknown-elf
PDK_PATH?=$(PDK_ROOT)/sky130A

SIM?=GL

.SUFFIXES:

PATTERN = io_ports

all:  ${PATTERN:=.vcd}

hex:  ${PATTERN:=.hex}

%.vvp: %_tb.v %.hex
ifndef PDK_ROOT
	$(error "!!! export PDK_ROOT= !!!!")
endif
ifeq ($(SIM),RTL)
	iverilog -DFUNCTIONAL -DSIM -I $(BEHAVIOURAL_MODELS) \
	-I $(PDK_PATH) -I $(IP_PATH) -I $(RTL_PATH) -I $(RAPCORES_PATH)/.. -I $(RAPCORES_PATH)/src -I $(RAPCORES_PATH)/testbench -I $(RAPCORES_PATH)/boards \
	-o $@ $<
else
	iverilog -DPROJ_GL -DFUNCTIONAL -DSIM -I $(BEHAVIOURAL_MODELS) \
	-I $(PDK_PATH) -I $(IP_PATH) -I $(RTL_PATH) -I $(RAPCORES_PATH)/.. -I $(RAPCORES_PATH)/src -I $(RAPCORES_PATH)/testbench -I $(RAPCORES_PATH)/boards \
	-o $@ $<
endif


%.vcd: %.vvp
	vvp $< -fst

%.elf: %.c $(FIRMWARE_PATH)/sections.lds $(FIRMWARE_PATH)/start.s
	${GCC_PATH}/${GCC_PREFIX}-gcc -march=rv32imc -mabi=ilp32 -Wl,-Bstatic,-T,$(FIRMWARE_PATH)/sections.lds,--strip-debug -ffreestanding -nostdlib -o $@ $(FIRMWARE_PATH)/start.s $<

%.hex: %.elf
	${GCC_PATH}/${GCC_PREFIX}-objcopy -O verilog $< $@
	# to fix flash base address
	sed -i 's/@10000000/@00000000/g' $@

%.bin: %.elf
	${GCC_PATH}/${GCC_PREFIX}-objcopy -O binary $< /dev/stdout | tail -c +1048577 > $@

# ---- Clean ----

clean:
	rm -f *.elf *.hex *.bin *.vvp *.vcd *.log

.PHONY: clean hex all
