`default_nettype none

`timescale 1 ns / 1 ps

`include "defines.v"
`include "mpw_one_defines.v"

//`define USE_POWER_PINS

`ifdef PROJ_GL
    `include "../gl/rapcores.v"
`else
    `include "rapcores.v"
    `include "macro_params.v"
    `include "constants.v"
    `include "quad_enc.v"
    `include "spi.v"
    `include "dda_timer.v"
    `include "spi_state_machine.v"
    `include "microstepper/chargepump.v"
    `include "microstepper/microstepper_control.v"
    `include "microstepper/mytimer_8.v"
    `include "microstepper/mytimer_10.v"
    `include "microstepper/microstep_counter.v"
    `include "microstepper/cosine.v"
    `include "microstepper/analog_out.v"
    `include "microstepper/microstepper_top.v"
    `include "rapcore.v"
`endif

`include "rapcore_harness_tb.v"
`include "caravel.v"
`include "spiflash.v"
`include "hbridge_coil.v"
`include "pwm_duty.v"

module io_ports_tb;
	reg clock;
    	reg RSTB;
	reg power1, power2;
	reg power3, power4;

	wire gpio;
	wire [37:0] mprj_io;
    assign mprj_io[3] = 1'b1;

	// External clock is used by default.  Make this artificially fast for the
	// simulation.  Normally this would be a slow clock and the digital PLL
	// would be the fast clock.

	always #12.5 clock <= (clock === 1'b0);

	initial begin
		clock = 0;
	end

	initial begin
		$dumpfile("io_ports.vcd");
		$dumpvars(0, io_ports_tb);

		// Repeat cycles of 1000 clock edges as needed to complete testbench
		repeat (25) begin
			repeat (4000) @(posedge clock);
			// $display("+1000 cycles");
		end
		$display("%c[1;31m",27);
		$display ("Monitor: Timeout, Test Mega-Project IO Ports (RTL) Failed");
		$display("%c[0m",27);
		$finish;
	end

	initial begin
	    // Observe Output pins
		wait(mprj_io[24] == 1'b1); // Move done =0
		wait(mprj_io[37:36] == 2'b10); // DTR:CIPO
		#1 $display("SPI response");
		wait(mprj_io[37:36] == 2'b11);
		wait(mprj_io[37:36] == 2'b10);
		#1 $display("Encoder data transmit");
		wait(mprj_io[15] == 1'b1);
		wait(mprj_io[15] == 1'b0);
		wait(mprj_io[15] == 1'b1);
		#1 $display("Charge pump switching");
		wait(mprj_io[27] == 1'b1);
		wait(mprj_io[27] == 1'b0);
		wait(mprj_io[27] == 1'b1);
		wait(mprj_io[28] == 1'b1);
		wait(mprj_io[28] == 1'b0);
		wait(mprj_io[28] == 1'b1);
		#1 $display("Both ADC outputs switching");
		wait(mprj_io[24] == 1'b1);
		#1 $display("DDA running");
		wait(mprj_io[30] == 0); // Step out
		wait(mprj_io[31] == 0); // Direction output only tested in one direction
		wait(mprj_io[30] == 1); // Step out
		wait(mprj_io[30] == 0); // Step out
		#1 $display("Step out switching");
		wait(mprj_io[23] == 1'b1);
		wait(mprj_io[19] == 1'b1);
		wait(mprj_io[16] == 1'b1);
		wait(mprj_io[20] == 1'b1);
		wait(mprj_io[21] == 1'b1);
		wait(mprj_io[18] == 1'b1);
		wait(mprj_io[14] == 1'b1);
		wait(mprj_io[17] == 1'b1);

		wait(mprj_io[23] == 1'b0);
		wait(mprj_io[19] == 1'b0);
		wait(mprj_io[16] == 1'b0);
		wait(mprj_io[20] == 1'b0);
		wait(mprj_io[21] == 1'b0);
		wait(mprj_io[18] == 1'b0);
		wait(mprj_io[14] == 1'b0);
		wait(mprj_io[17] == 1'b0);

		wait(mprj_io[23] == 1'b1);
		wait(mprj_io[19] == 1'b1);
		wait(mprj_io[16] == 1'b1);
		wait(mprj_io[20] == 1'b1);
		wait(mprj_io[21] == 1'b1);
		wait(mprj_io[18] == 1'b1);
		wait(mprj_io[14] == 1'b1);
		wait(mprj_io[17] == 1'b1);

		wait(mprj_io[23] == 1'b0);
		wait(mprj_io[19] == 1'b0);
		wait(mprj_io[16] == 1'b0);
		wait(mprj_io[20] == 1'b0);
		wait(mprj_io[21] == 1'b0);
		wait(mprj_io[18] == 1'b0);
		wait(mprj_io[14] == 1'b0);
		wait(mprj_io[17] == 1'b0);
		#1 $display("Bridge outputs switching");
		//wait(mprj_io[10] ==  // Enable output
		//wait(mprj_io[24] ==  // Move done
		//wait(mprj_io[29] ==  // Halt
	    $display("Monitor: Test 1 RAPcores Passed");
	    $finish;
	end

	initial begin
		RSTB <= 1'b0;
		#2000;
		RSTB <= 1'b1;	    // Release reset
	end

    reg bootdone = 1'b0;
	initial begin		// Power-up sequence
		power1 <= 1'b0;
		power2 <= 1'b0;
		power3 <= 1'b0;
		power4 <= 1'b0;
        #400
		power1 <= 1'b1;
		power2 <= 1'b1;
		power3 <= 1'b1;
		power4 <= 1'b1;
        //#400000;
        //bootdone <= 1'b1;
	end

	always @(mprj_io[5]) begin
		#1 $display("Booting ", mprj_io[37:5]);
	end

	wire flash_csb;
	wire flash_clk;
	wire flash_io0;
	wire flash_io1;

	wire VDD3V3 = power1;
	wire VDD1V8 = power2;
	wire USER_VDD3V3 = power3;
	wire USER_VDD1V8 = power4;
	wire VSS = 1'b0;


    reg                 step;
    reg                 dir;
    reg                 enable_in;
    wire        [12:0]  target_current1;
    wire        [12:0]  target_current2;
    wire signed  [12:0]  current1;
    wire signed  [12:0]  current2;

	//assign resetn = RSTB;

  rapcore_harness harness0 (
        .CLK(clock),
        //.resetn_in(resetn),
        .CHARGEPUMP(mprj_io[15]),
        .analog_cmp1(mprj_io[25]),
        .analog_out1(mprj_io[27]),
        .analog_cmp2(mprj_io[26]),
        .analog_out2(mprj_io[28]),
        .PHASE_A1(mprj_io[23]),
        .PHASE_A2(mprj_io[19]),
        .PHASE_B1(mprj_io[16]),
        .PHASE_B2(mprj_io[20]),
        .PHASE_A1_H(mprj_io[21]),
        .PHASE_A2_H(mprj_io[18]),
        .PHASE_B1_H(mprj_io[14]),
        .PHASE_B2_H(mprj_io[17]),
        .ENC_B(mprj_io[12]),
        .ENC_A(mprj_io[13]),
        .BUFFER_DTR(mprj_io[37]),
        .MOVE_DONE(mprj_io[24]),
        .HALT(mprj_io[29]),
        .SCK(mprj_io[35]),
        .CS(mprj_io[34]),
        .COPI(mprj_io[22]),
        .CIPO(mprj_io[36]),
        .STEPOUTPUT(mprj_io[30]),
        .DIROUTPUT(mprj_io[31]),
        .STEPINPUT(mprj_io[32]),
        .DIRINPUT(mprj_io[33]),
        .ENINPUT(mprj_io[11]),
        .ENOUTPUT(mprj_io[10]),
		.BOOT_DONE_IN(mprj_io[15])

  );

	caravel uut (
		.vddio	  (VDD3V3),
		.vssio	  (VSS),
		.vdda	  (VDD3V3),
		.vssa	  (VSS),
		.vccd	  (VDD1V8),
		.vssd	  (VSS),
		.vdda1    (USER_VDD3V3),
		.vdda2    (USER_VDD3V3),
		.vssa1	  (VSS),
		.vssa2	  (VSS),
		.vccd1	  (USER_VDD1V8),
		.vccd2	  (USER_VDD1V8),
		.vssd1	  (VSS),
		.vssd2	  (VSS),
		.clock	  (clock),
		.gpio     (gpio),
        	.mprj_io  (mprj_io),
		.flash_csb(flash_csb),
		.flash_clk(flash_clk),
		.flash_io0(flash_io0),
		.flash_io1(flash_io1),
		.resetb	  (RSTB)
	);


	spiflash #(
		.FILENAME("io_ports.hex")
	) spiflash (
		.csb(flash_csb),
		.clk(flash_clk),
		.io0(flash_io0),
		.io1(flash_io1),
		.io2(),			// not used
		.io3()			// not used
	);

endmodule
`default_nettype wire
