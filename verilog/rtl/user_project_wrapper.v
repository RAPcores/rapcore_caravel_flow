`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7.
    inout [`MPRJ_IO_PADS-8:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2
);

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

  // GPIO output enable (0 = output, 1 = input)
    assign io_oeb[15] = vssd1;    // CHARGEPUMP
    assign io_oeb[25] = vccd1;    // analog_cmp1
    assign io_oeb[27] = vssd1;    // analog_out1
    assign io_oeb[26] = vccd1;    // analog_cmp2
    assign io_oeb[28] = vssd1;    // analog_out2
    assign io_oeb[23] = vssd1;    // PHASE_A1
    assign io_oeb[19] = vssd1;    // PHASE_A2
    assign io_oeb[16] = vssd1;    // PHASE_B1
    assign io_oeb[20] = vssd1;    // PHASE_B2
    assign io_oeb[21] = vssd1;    // PHASE_A1_H
    assign io_oeb[18] = vssd1;    // PHASE_A2_H
    assign io_oeb[14] = vssd1;    // PHASE_B1_H
    assign io_oeb[17] = vssd1;    // PHASE_B2_H
    assign io_oeb[18] = vccd1;    // ENC_B
    assign io_oeb[19] = vccd1;    // ENC_A
    assign io_oeb[12] = vssd1;    // BUFFER_DTR
    assign io_oeb[24] = vssd1;    // MOVE_DONE
    assign io_oeb[29] = vccd1;    // HALT
    assign io_oeb[10] = vccd1;    // SCK
    assign io_oeb[9] = vccd1;     // CS
    assign io_oeb[8] = vccd1;     // COPI
    assign io_oeb[11] = vssd1;    // CIPO
    assign io_oeb[30] = vssd1;    // STEPOUTPUT
    assign io_oeb[31] = vssd1;    // DIROUTPUT
    assign io_oeb[32] = vccd1;    // STEPINPUT
    assign io_oeb[33] = vccd1;    // DIRINPUT
    // unused
    assign io_oeb[0] = vccd1;    // JTAG I/O
    assign io_oeb[1] = vccd1;    // SDO
    assign io_oeb[2] = vccd1;    // SDI
    assign io_oeb[3] = vccd1;    // CSB
    assign io_oeb[4] = vccd1;    // SCK
    assign io_oeb[5] = vccd1;    // Rx
    assign io_oeb[6] = vccd1;    // Tx
    assign io_oeb[7] = vccd1;    // IRQ
    assign io_oeb[13] = vccd1;
    assign io_oeb[22] = vccd1;
    assign io_oeb[34] = vccd1;
    assign io_oeb[35] = vccd1;
    assign io_oeb[36] = vccd1;
    assign io_oeb[37] = vccd1;

    /*--------------------------------------*/
    /* User project is instantiated  here   */
    /*--------------------------------------*/

    rapcores rapcores0 (

`ifndef SIM
    `ifdef USE_POWER_PINS
	.vdda1(vdda1),	// User area 1 3.3V power
	.vdda2(vdda2),	// User area 2 3.3V power
	.vssa1(vssa1),	// User area 1 analog ground
	.vssa2(vssa2),	// User area 2 analog ground
	.vccd1(vccd1),	// User area 1 1.8V power
	.vccd2(vccd2),	// User area 2 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
	.vssd2(vssd2),	// User area 2 digital ground
    `endif
`endif
	// MGMT core clock and reset

    	.wb_clk_i(wb_clk_i),
    	.wb_rst_i(wb_rst_i),

	// MGMT SoC Wishbone Slave

	.wbs_cyc_i(wbs_cyc_i),
	.wbs_stb_i(wbs_stb_i),
	.wbs_we_i(wbs_we_i),
	.wbs_sel_i(wbs_sel_i),
	//.wbs_adr_i(wbs_adr_i),
	//.wbs_dat_i(wbs_dat_i),
	.wbs_ack_o(wbs_ack_o),
	//.wbs_dat_o(wbs_dat_o),

	// Logic Analyzer

	.la_data_in(la_data_in),
	.la_data_out(la_data_out),
	.la_oen (la_oen),

	// IO Pads

	.io_in (io_in),
    	.io_out(io_out),
    	.io_oeb(io_oeb)
    );

endmodule	// user_project_wrapper
`default_nettype wire
