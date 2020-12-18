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

module rapcore #(
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
    output [`MPRJ_IO_PADS-1:0] io_oeb
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata;
    wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i;
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    //assign io_out = count;
    //assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // LA
    assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register
    assign la_write = ~la_oen[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset
    assign clk = wb_clk_i;
    assign rst = (~la_oen[65]) ? la_data_in[65]: wb_rst_i;


    top top (

        // IO Pads
        .CLK(clk),
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
        .BUFFER_DTR(io_out[12]),
        .MOVE_DONE(io_out[24]),
        .HALT(io_in[29]),
        .SCK(io_in[10]),
        .CS(io_in[9]),
        .COPI(io_in[8]),
        .CIPO(io_out[11])
    );

endmodule

`default_nettype wire
