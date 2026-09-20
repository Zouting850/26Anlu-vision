module video_pll (input refclk, input reset, output extlock,
                  output clk0_out, output clk1_out, output clk2_out, output clk3_out);
  assign {clk0_out,clk1_out,clk2_out,clk3_out} = 4'b0;
endmodule
module sdram (input Clk, input Clk_sft, input Rst, output Sdr_init_done,
              output Sdr_init_ref_vld, output Sdr_busy,
              input App_wr_en, input [20:0] App_wr_addr, input [3:0] App_wr_dm, input [31:0] App_wr_din,
              input App_rd_en, input [20:0] App_rd_addr, output Sdr_rd_en, output [31:0] Sdr_rd_dout);
  assign {Sdr_init_done,Sdr_init_ref_vld,Sdr_busy,Sdr_rd_en} = 4'b0;
  assign Sdr_rd_dout = 32'b0;
endmodule
module hdmi_tx #(parameter FAMILY = "EG4") (
  input PXLCLK_I, input PXLCLK_5X_I, input RST_N,
  input VGA_HS, input VGA_VS, input VGA_DE, input [23:0] VGA_RGB,
  output HDMI_CLK_P, output HDMI_D2_P, output HDMI_D1_P, output HDMI_D0_P);
  assign {HDMI_CLK_P,HDMI_D2_P,HDMI_D1_P,HDMI_D0_P} = 4'b0;
endmodule
module wfifo_32_32_512 (input clkr, input clkw, input rst, input we, input re, input [31:0] di,
  output empty_flag, output full_flag, output [8:0] wrusedw, output [8:0] rdusedw, output [31:0] dout);
  assign {empty_flag,full_flag} = 2'b0; assign {wrusedw,rdusedw} = 18'b0; assign dout = 32'b0;
endmodule
module rfifo_32_32_512 (input clkr, input clkw, input rst, input we, input re, input [31:0] di,
  output empty_flag, output full_flag, output [8:0] wrusedw, output [8:0] rdusedw, output [31:0] dout);
  assign {empty_flag,full_flag} = 2'b0; assign {wrusedw,rdusedw} = 18'b0; assign dout = 32'b0;
endmodule
