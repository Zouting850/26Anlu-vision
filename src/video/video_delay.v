module video_delay
#(
	parameter DATA_WIDTH = 24,                      // Video data one clock data width
	parameter PIPE_LAT   = 0                        // read_data 相对 read_en 多出的流水线拍数
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output                      read_en,            // Read data enable
	input[DATA_WIDTH - 1:0]     read_data,          // Read data
	input                      hs,                 // horizontal synchronization
	input                      vs,                 // vertical synchronization
	input                      de,                 // video valid

	output                      hs_r,                 // horizontal synchronization
	output                      vs_r,                 // vertical synchronization
	output                      de_r,                 // video valid
	output[DATA_WIDTH - 1:0]    vout_data           // video data

);
localparam RD_IDX   = 18;                       // 向读 FIFO 发取的节拍
localparam CAP_IDX  = RD_IDX + 1 + PIPE_LAT;    // read_data 有效的节拍
localparam OUT_IDX  = CAP_IDX + 1;              // 与 vout_data_r 对齐的节拍

reg [OUT_IDX:0] hs_d;
reg [OUT_IDX:0] vs_d;
reg [OUT_IDX:0] de_d;
reg[DATA_WIDTH - 1:0]  vout_data_r;

assign read_en = de_d[RD_IDX];
assign hs_r = hs_d[OUT_IDX];
assign vs_r = vs_d[OUT_IDX];
assign de_r = de_d[OUT_IDX];
assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(de_d[CAP_IDX])
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
always @(posedge video_clk or posedge rst)begin
	if(rst)begin
    	hs_d <= {(OUT_IDX + 1){1'b0}};
        vs_d <= {(OUT_IDX + 1){1'b0}};
        de_d <= {(OUT_IDX + 1){1'b0}};
    end
	else begin
    	
    	hs_d <= {hs_d[OUT_IDX - 1:0],hs};
        vs_d <= {vs_d[OUT_IDX - 1:0],vs};
        de_d <= {de_d[OUT_IDX - 1:0],de};
    end
end
endmodule
