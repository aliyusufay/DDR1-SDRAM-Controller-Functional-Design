`timescale 1ns/1ps

module tb_ddr_controller;

  // Clock & reset
  logic clk;
  logic clk2x;
  logic rst_n;

  // CPU-side signals
  logic [2:0]  icmd;
  logic [31:0] data_in;
  logic [31:0] iaddr;
  logic [3:0]  dmsel;

  // Status & data out
  wire         busy;
  wire [31:0]  dataout;
  wire         dataout_valid;
  wire         datain_valid;

  // DDR-side (left un-modelled for simplicity)
  wire         clkout, clkoutn, cke, cs_n, ras_n, cas_n, we_n;
  wire [1:0]   ba;
  wire [13:0]  addrout;
  wire         dm, ldm;
  tri  [15:0]  dq;
  tri          dqs;

  // Instance of DUT
  ddr_controller uut (
    .clk             (clk),
    .rst_n           (rst_n),
    .icmd            (icmd),
    .data_in         (data_in),
    .iaddr           (iaddr),
    .dmsel           (dmsel),
    .clk2x           (clk2x),
    .busy            (busy),
    .dataout         (dataout),
    .dataout_valid   (dataout_valid),
    .datain_valid    (datain_valid),
    .clkout          (clkout),
    .clkoutn         (clkoutn),
    .cke             (cke),
    .cs_n            (cs_n),
    .ras_n           (ras_n),
    .cas_n           (cas_n),
    .we_n            (we_n),
    .ba              (ba),
    .addrout         (addrout),
    .dm              (dm),
    .ldm             (ldm),
    .dq              (dq),
    .dqs             (dqs)
  );

  // Generate 100 MHz clock
  always #5   clk   = ~clk;
  // Generate 200 MHz clock
  always #2.5 clk2x = ~clk2x;

  // Simple memory model: just loopback dq/dqs (not functional)
  assign dq  = 16'hZZZZ;
  assign dqs = 1'bZ;

  // Stimulus
  initial begin
    // initial values
    clk           = 0;
    clk2x         = 0;
    rst_n         = 1;
    icmd          = 3'b100;  // C_NOP
    data_in       = 32'd0;
    iaddr         = 32'd0;
    dmsel         = 4'b0011;


    // Hold reset for a few cycles
    #10;
    rst_n = 0;
    #100;
    rst_n = 1;
    #200000;

    // Wait for controller to finish its init sequence
    wait (uut.init_done == 1);
    $display("[%0t] Controller initialization done.", $time);

    // Perform a 4-beat write to address 0x1000
    write_burst(32'h0000_1000, 4, {32'hDEAD_BEEF, 32'hC0DE_CAFE, 32'h1234_5678, 32'h8765_4321});

    // Give a few cycles before reading
    #20;

    // Perform a 4-beat read back from same address
    read_burst(32'h0000_1000, 4);

    // Finish after some time
    #10;
	@(posedge clk);
	icmd <= 3'b010;
	#10;
	@(posedge clk);
	icmd <= 3'b100;
	#300;
    $display("Testbench finished at time %0t.", $time);
    $finish;
  end

  // Task to issue a burst write of `len` words, data supplied in `data_array`
  task write_burst(
    input logic [31:0] addr,
    input int          len,
    input logic [31:0] data_array []
  );
    int i;
    begin
      // Issue WRITE command
      @(posedge clk);
      icmd         <= 3'b001;  // C_WRITE
      iaddr        <= addr;
      @(posedge clk);
      icmd         <= 3'b100;  // back to NOP

      // Wait until DUT is ready to accept data
      for (i = 0; i < len; i++) begin
        @(posedge clk);
        wait (datain_valid);
        data_in      <= data_array[i];
      end

      // Clear inputs
      @(posedge clk);
      data_in      <= '0;
      $display("[%0t] Write burst complete.", $time);
    end
  endtask

  // Task to issue a burst read of `len` words and check they match a loopback
  task read_burst(
    input logic [31:0] addr,
    input int          len
  );
    int i;
    logic [31:0] read_data [0:15];
    begin
      // Issue READ command
      @(posedge clk);
      icmd   <= 3'b000;  // C_READ
      iaddr  <= addr;
      @(posedge clk);
      icmd   <= 3'b100;  // back to NOP

      // Capture read data
      for (i = 0; i < len; i++) begin
        @(posedge clk);
        wait (dataout_valid);
        read_data[i] = dataout;
        $display("[%0t] Read word %0d = 0x%08h", $time, i, dataout);
      end

      // (In a real TB, compare read_data[] against expected values)
      $display("[%0t] Read burst complete.", $time);
    end
  endtask

endmodule
