// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3 (lin64) Build 2405991 Thu Dec  6 23:36:41 MST 2018
// Date        : Wed Jul  3 09:07:13 2019
// Host        : z400 running 64-bit Arch Linux
// Command     : write_verilog -force -mode synth_stub /home/tom/git/TomKeddie/prj-litex/arty/usb/hw/ila_1/ila_1_stub.v
// Design      : ila_1
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a35ticsg324-1L
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila,Vivado 2018.3" *)
module ila_1(clk, probe0, probe1, probe2, probe3, probe4, probe5, 
  probe6, probe7, probe8, probe9, probe10, probe11, probe12)
/* synthesis syn_black_box black_box_pad_pin="clk,probe0[23:0],probe1[31:0],probe2[0:0],probe3[0:0],probe4[31:0],probe5[3:0],probe6[0:0],probe7[0:0],probe8[3:0],probe9[3:0],probe10[3:0],probe11[0:0],probe12[0:0]" */;
  input clk;
  input [23:0]probe0;
  input [31:0]probe1;
  input [0:0]probe2;
  input [0:0]probe3;
  input [31:0]probe4;
  input [3:0]probe5;
  input [0:0]probe6;
  input [0:0]probe7;
  input [3:0]probe8;
  input [3:0]probe9;
  input [3:0]probe10;
  input [0:0]probe11;
  input [0:0]probe12;
endmodule
