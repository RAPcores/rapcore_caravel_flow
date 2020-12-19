`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

`default_nettype none

module rapcores #(
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
    //input [31:0] wbs_dat_i,
    //input [31:0] wbs_adr_i,
    output wbs_ack_o,
    //output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb
);

    wire clk;
    wire rst;
    wire enable;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    //wire [31:0] rdata;
    //wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i;
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    //assign wbs_dat_o = rdata;
    //assign wdata = wbs_dat_i;

    // IO
    //assign io_out = count;
    //assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

		assign count = 0;
    // LA
    assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register
    assign la_write = ~la_oen[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset
    assign clk = wb_clk_i;
    assign rst = ~la_oen[65] && la_data_in[65] && ~wb_rst_i;
    assign enable = ~la_oen[64] && la_data_in[64];

    // GPIO output enable (0 = output, 1 = input)
    assign io_oeb[15] = 1'b0;    // CHARGEPUMP
    assign io_oeb[25] = 1'b1;    // analog_cmp1
    assign io_oeb[27] = 1'b0;    // analog_out1
    assign io_oeb[26] = 1'b1;    // analog_cmp2
    assign io_oeb[28] = 1'b0;    // analog_out2
    assign io_oeb[23] = 1'b0;    // PHASE_A1
    assign io_oeb[19] = 1'b0;    // PHASE_A2
    assign io_oeb[16] = 1'b0;    // PHASE_B1
    assign io_oeb[20] = 1'b0;    // PHASE_B2
    assign io_oeb[21] = 1'b0;    // PHASE_A1_H
    assign io_oeb[18] = 1'b0;    // PHASE_A2_H
    assign io_oeb[14] = 1'b0;    // PHASE_B1_H
    assign io_oeb[17] = 1'b0;    // PHASE_B2_H
    assign io_oeb[12] = 1'b1;    // ENC_B
    assign io_oeb[13] = 1'b1;    // ENC_A
    assign io_oeb[37] = 1'b0;    // BUFFER_DTR
    assign io_oeb[24] = 1'b0;    // MOVE_DONE
    assign io_oeb[29] = 1'b1;    // HALT
    assign io_oeb[35] = 1'b1;    // SCK
    assign io_oeb[34] = 1'b1;     // CS
    assign io_oeb[33] = 1'b1;     // COPI
    assign io_oeb[36] = 1'b0;    // CIPO
    assign io_oeb[30] = 1'b0;    // STEPOUTPUT
    assign io_oeb[31] = 1'b0;    // DIROUTPUT
    assign io_oeb[32] = 1'b1;    // STEPINPUT
    assign io_oeb[33] = 1'b1;    // DIRINPUT
    // unused
    assign io_oeb[0] = 1'b0;    // JTAG I/O
    assign io_oeb[1] = 1'b0;    // SDO
    assign io_oeb[2] = 1'b0;    // SDI
    assign io_oeb[3] = 1'b0;    // CSB
    assign io_oeb[4] = 1'b0;    // SCK
    assign io_oeb[5] = 1'b0;    // Rx
    assign io_oeb[6] = 1'b0;    // Tx
    assign io_oeb[7] = 1'b0;    // IRQ
    assign io_oeb[8] = 1'b1;
    assign io_oeb[9] = 1'b1;
    assign io_oeb[10] = 1'b1;


		wire resetn;
    reg [13:0] resetn_counter = 0;
		assign resetn = &resetn_counter && rst;

		always @(posedge wb_clk_i) begin
		  if (!resetn && !wb_rst_i && rst) resetn_counter <= resetn_counter +1;
		end

    // IO
    assign io_out[7:0] = resetn_counter[13:6]; //count;

    rapcore rapcore0 (

        // IO Pads
        .CLK(wb_clk_i),
        .resetn_in(resetn),
        .CHARGEPUMP(io_out[15]),
        .analog_cmp1(io_in[25]),
        .analog_out1(io_out[27]),
        .analog_cmp2(io_in[26]),
        .analog_out2(io_out[28]),
        .PHASE_A1(io_out[23]),
        .PHASE_A2(io_out[19]),
        .PHASE_B1(io_out[16]),
        .PHASE_B2(io_out[20]),
        .PHASE_A1_H(io_out[21]),
        .PHASE_A2_H(io_out[18]),
        .PHASE_B1_H(io_out[14]),
        .PHASE_B2_H(io_out[17]),
        .ENC_B(io_in[12]),
        .ENC_A(io_in[13]),
        .BUFFER_DTR(io_out[37]),
        .MOVE_DONE(io_out[24]),
        .HALT(io_in[29]),
        .SCK(io_in[35]),
        .CS(io_in[34]),
        .COPI(io_in[22]),
        .CIPO(io_out[36]),
        .STEPOUTPUT(io_out[30]),
        .DIROUTPUT(io_out[31]),
        .STEPINPUT(io_in[32]),
        .DIRINPUT(io_in[33]),
        .ENINPUT(la_data_in[64]),
        .ENABLE_OEB(la_oen[64])
    );

endmodule
