# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

# Base Configurations. Don't Touch
# section begin
set script_dir [file dirname [file normalize [info script]]]
set ::env(DESIGN_NAME) user_project_wrapper
#section end

#set ::env(DESIGN_IS_CORE) 1
#set ::env(SYNTH_READ_BLACKBOX_LIB) 1

set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_PERIOD) "15"


set ::env(VERILOG_FILES) "\
    $script_dir/../../verilog/rtl/defines.v \
    $script_dir/../../verilog/rtl/user_project_wrapper.v"

set ::env(VERILOG_FILES_BLACKBOX) "\
    $script_dir/../../verilog/rtl/defines.v \
    $script_dir/../../verilog/rtl/rapcores.v"

set ::env(EXTRA_LEFS) "\
	$script_dir/../../lef/rapcores.lef"

set ::env(EXTRA_GDS_FILES) "\
	$script_dir/../../gds/rapcores.gds"

## Internal Macros
### Macro Placement
set ::env(MACRO_PLACEMENT_CFG) $script_dir/macro.cfg

set ::env(PL_OPENPHYSYN_OPTIMIZATIONS) 0
set ::env(DIODE_INSERTION_STRATEGY) 3

set ::env(ROUTING_CORES) 6

# Need to fix a FastRoute bug for this to work, but it's good
# for a sense of "isolation"
set ::env(MAGIC_ZEROIZE_ORIGIN) 0
set ::env(MAGIC_WRITE_FULL_LEF) 0

# The following is because there are no std cells in the example wrapper project.
set ::env(SYNTH_TOP_LEVEL) 1
set ::env(PL_RANDOM_GLB_PLACEMENT) 1
set ::env(PL_OPENPHYSYN_OPTIMIZATIONS) 0
set ::env(DIODE_INSERTION_STRATEGY) 0
set ::env(FILL_INSERTION) 0
set ::env(TAP_DECAP_INSERTION) 0
set ::env(CLOCK_TREE_SYNTH) 0

set ::env(MAGIC_GENERATE_LEF) 0
set ::env(RUN_SPEF_EXTRACTION) 0
set ::env(GLB_RT_OBS) "li1 0 0 2920 3520"
set ::env(GLB_RT_MINLAYER) 2

# Area Configurations. DON'T TOUCH.
set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 2920 3520"

# Power & Pin Configurations. DON'T TOUCH.
set ::env(FP_PDN_CORE_RING) 1
set ::env(FP_PDN_CORE_RING_VWIDTH) 3
set ::env(FP_PDN_CORE_RING_HWIDTH) $::env(FP_PDN_CORE_RING_VWIDTH)
set ::env(FP_PDN_CORE_RING_VOFFSET) 14
set ::env(FP_PDN_CORE_RING_HOFFSET) $::env(FP_PDN_CORE_RING_VOFFSET)
set ::env(FP_PDN_CORE_RING_VSPACING) 1.7
set ::env(FP_PDN_CORE_RING_HSPACING) $::env(FP_PDN_CORE_RING_VSPACING)

set ::env(FP_PDN_VWIDTH) 3
set ::env(FP_PDN_HWIDTH) 3
set ::env(FP_PDN_VOFFSET) 0
set ::env(FP_PDN_HOFFSET) $::env(FP_PDN_VOFFSET)
set ::env(FP_PDN_VPITCH) 180
set ::env(FP_PDN_HPITCH) $::env(FP_PDN_VPITCH)
set ::env(FP_PDN_VSPACING) [expr 5*$::env(FP_PDN_CORE_RING_VWIDTH)]
set ::env(FP_PDN_HSPACING) [expr 5*$::env(FP_PDN_CORE_RING_HWIDTH)]

#set ::env(VDD_NETS) [list {vccd1} {vccd2} {vdda1} {vdda2}]
#set ::env(GND_NETS) [list {vssd1} {vssd2} {vssa1} {vssa2}]
set ::env(VDD_NETS) vccd1
set ::env(GND_NETS) vssd1
set ::env(VDD_PIN) vccd1
set ::env(GND_PIN) vssd1

set ::env(SYNTH_USE_PG_PINS_DEFINES) "USE_POWER_PINS"

set ::env(RUN_CVC) 0

# Pin Configurations. DON'T TOUCH
set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg
#set ::env(FP_DEF_TEMPLATE) $script_dir/../../def/user_project_wrapper_empty.def
set ::unit 2.4
set ::env(FP_IO_VEXTEND) [expr 2*$::unit]
set ::env(FP_IO_HEXTEND) [expr 2*$::unit]
set ::env(FP_IO_VLENGTH) $::unit
set ::env(FP_IO_HLENGTH) $::unit

set ::env(FP_IO_VTHICKNESS_MULT) 4
set ::env(FP_IO_HTHICKNESS_MULT) 4
