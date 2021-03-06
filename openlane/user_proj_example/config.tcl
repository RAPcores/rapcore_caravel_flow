set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) user_proj_example

set ::env(VERILOG_FILES) "\
        $script_dir/../../verilog/rtl/defines.v \
        $script_dir/../../rapcores/src/constants.v \
        $script_dir/../../rapcore_caravel_defines.v \
        $script_dir/../../rapcores/src/macro_params.v \
        $script_dir/../../rapcores/src/top.v \
        $script_dir/../../verilog/rtl/user_proj_example.v \
        $script_dir/../../rapcores/src/stepper.v \
        $script_dir/../../rapcores/src/spi.v \
        $script_dir/../../rapcores/src/quad_enc.v \
        $script_dir/../../rapcores/src/pwm.v \
        $script_dir/../../rapcores/src/microstepper/microstepper_top.v \
        $script_dir/../../rapcores/src/microstepper/microstep_counter.v \
        $script_dir/../../rapcores/src/microstepper/cosine.v \
        $script_dir/../../rapcores/src/microstepper/analog_out.v \
        $script_dir/../../rapcores/src/microstepper/chargepump.v \
        $script_dir/../../rapcores/src/microstepper/mytimer.v"

set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_PERIOD) "10"

set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 2500 3500"
set ::env(DESIGN_IS_CORE) 0

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg
# set ::env(FP_CONTEXT_DEF) $script_dir/../user_project_wrapper/runs/user_project_wrapper/tmp/floorplan/ioPlacer.def.macro_placement.def
# set ::env(FP_CONTEXT_LEF) $script_dir/../user_project_wrapper/runs/user_project_wrapper/tmp/merged_unpadded.lef

set ::env(PL_BASIC_PLACEMENT) 1
set ::env(PL_TARGET_DENSITY) 0.01
