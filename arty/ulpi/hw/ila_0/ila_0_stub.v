// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.1 (lin64) Build 2552052 Fri May 24 14:47:09 MDT 2019
// Date        : Sat Oct 19 08:04:02 2019
// Host        : z400 running 64-bit Ubuntu 19.04
// Command     : write_verilog -force -mode synth_stub /home/tom/git/TomKeddie/prj-litex/arty/ulpi/hw/ila_0/ila_0_stub.v
// Design      : ila_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a35tcsg324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila,Vivado 2019.1" *)
module ila_0(clk, probe0, probe1, probe2)
/* synthesis syn_black_box black_box_pad_pin="clk,probe0[4:0],probe1[7:0],probe2[1:0]" */;
  input clk;
  input [4:0]probe0;
  input [7:0]probe1;
  input [1:0]probe2;
endmodule