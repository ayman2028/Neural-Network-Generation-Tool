`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"


module net_4_8_12_16_16_1_10(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M1 = 8;
	parameter M2 = 12;
	parameter M3 = 16;
	parameter N = 4;
	parameter T = 16;
	parameter P1 = 1;
	parameter P2 = 2;
	parameter P3 = 4;

	// Top level ports
	input                 clk, reset, input_valid, output_ready;
	input signed [T-1:0]  input_data;
	output signed [T-1:0] output_data;
	output logic          output_valid, input_ready;

	// Internal signals (connect the three layers)
	logic [T-1:0] L1toL2_output_data,L2toL3_output_data;
	logic L1toL2_output_valid, L2toL3_output_valid, L1toL2_output_ready,L2toL3_output_ready;
	// Instantiate the three layers

	l1_fc_8_4_16_1_1 #(M1, N, T, P1, 1)layer_1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data), .output_valid(L1toL2_output_valid), .output_ready(L1toL2_output_ready),.output_data(L1toL2_output_data));

	l2_fc_12_8_16_1_2 #(M2, M1, T, P2, 2)layer_2(.clk(clk), .reset(reset), .input_valid(L1toL2_output_valid), .input_ready(L1toL2_output_ready), .input_data(L1toL2_output_data), .output_valid(L2toL3_output_valid), .output_ready(L2toL3_output_ready), .output_data(L2toL3_output_data));

	l3_fc3_16_12_16_1_4 #(M3, M2, T, P3, 3)layer_3(.clk(clk), .reset(reset), .input_valid(L2toL3_output_valid), .input_ready(L2toL3_output_ready), .input_data(L2toL3_output_data), .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));


endmodule

module l1_fc_8_4_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 8;
	parameter N = 4;
	parameter T = 16;
	parameter P = 1;
	parameter L = 1;

	// Top level ports
	input                 clk, reset, input_valid, output_ready;
	input signed [T-1:0]  input_data;
	output signed [T-1:0] output_data;
	output logic          output_valid, input_ready;

	// Internal signals (connect control with datapath)
	localparam                VECTOR_SIZE=$clog2(N);
	localparam               MATRIX_SIZE=$clog2(M*N/P);
	logic [VECTOR_SIZE-1:0]  addr_x;
	logic [MATRIX_SIZE-1:0]  addr_w;
	logic [P-1:0] f_sel;
	logic                    wr_en_x, wr_en_w, clear_acc, en_acc;

	//Instantiate Control
	control_gen #(M,N,T,P)control1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .output_ready(output_ready), .output_valid(output_valid), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .wr_en_w(wr_en_w), .clear_acc(clear_acc), .en_acc(en_acc), .f_sel(f_sel));

	//Instantiate Datapath
	datapath_gen_p3_relu #(M,N,T,P,L)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p1(clk, addr, z);
	parameter			 M = 8;
	parameter			 N = 4;
	parameter			 T = 16;
	parameter			 P = 1;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = -16'd4;
			assign values[1] = 16'd5;
			assign values[2] = -16'd8;
			assign values[3] = 16'd7;
			assign values[4] = -16'd4;
			assign values[5] = -16'd4;
			assign values[6] = 16'd5;
			assign values[7] = -16'd2;
			assign values[8] = -16'd2;
			assign values[9] = 16'd5;
			assign values[10] = -16'd7;
			assign values[11] = -16'd6;
			assign values[12] = 16'd7;
			assign values[13] = 16'd5;
			assign values[14] = -16'd6;
			assign values[15] = -16'd4;
			assign values[16] = 16'd0;
			assign values[17] = 16'd7;
			assign values[18] = -16'd8;
			assign values[19] = 16'd1;
			assign values[20] = -16'd4;
			assign values[21] = -16'd6;
			assign values[22] = -16'd4;
			assign values[23] = 16'd2;
			assign values[24] = -16'd8;
			assign values[25] = 16'd1;
			assign values[26] = -16'd5;
			assign values[27] = 16'd1;
			assign values[28] = 16'd0;
			assign values[29] = -16'd3;
			assign values[30] = -16'd1;
			assign values[31] = 16'd5;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule

module l2_fc_12_8_16_1_2(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 12;
	parameter N = 8;
	parameter T = 16;
	parameter P = 2;
	parameter L = 2;

	// Top level ports
	input                 clk, reset, input_valid, output_ready;
	input signed [T-1:0]  input_data;
	output signed [T-1:0] output_data;
	output logic          output_valid, input_ready;

	// Internal signals (connect control with datapath)
	localparam                VECTOR_SIZE=$clog2(N);
	localparam               MATRIX_SIZE=$clog2(M*N/P);
	logic [VECTOR_SIZE-1:0]  addr_x;
	logic [MATRIX_SIZE-1:0]  addr_w;
	logic [P-1:0] f_sel;
	logic                    wr_en_x, wr_en_w, clear_acc, en_acc;

	//Instantiate Control
	control_gen #(M,N,T,P)control1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .output_ready(output_ready), .output_valid(output_valid), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .wr_en_w(wr_en_w), .clear_acc(clear_acc), .en_acc(en_acc), .f_sel(f_sel));

	//Instantiate Datapath
	datapath_gen_p3_relu #(M,N,T,P,L)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p2(clk, addr, z);
	parameter			 M = 12;
	parameter			 N = 8;
	parameter			 T = 16;
	parameter			 P = 2;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = -16'd5;
			assign values[1] = -16'd1;
			assign values[2] = 16'd4;
			assign values[3] = -16'd1;
			assign values[4] = 16'd4;
			assign values[5] = 16'd1;
			assign values[6] = 16'd5;
			assign values[7] = -16'd6;
			assign values[8] = -16'd2;
			assign values[9] = 16'd2;
			assign values[10] = 16'd5;
			assign values[11] = 16'd3;
			assign values[12] = 16'd4;
			assign values[13] = -16'd6;
			assign values[14] = -16'd3;
			assign values[15] = 16'd4;
			assign values[16] = -16'd5;
			assign values[17] = 16'd4;
			assign values[18] = 16'd0;
			assign values[19] = 16'd7;
			assign values[20] = -16'd3;
			assign values[21] = -16'd2;
			assign values[22] = -16'd6;
			assign values[23] = 16'd4;
			assign values[24] = -16'd3;
			assign values[25] = -16'd6;
			assign values[26] = 16'd7;
			assign values[27] = -16'd7;
			assign values[28] = -16'd4;
			assign values[29] = -16'd3;
			assign values[30] = 16'd5;
			assign values[31] = 16'd7;
			assign values[32] = -16'd8;
			assign values[33] = -16'd3;
			assign values[34] = -16'd7;
			assign values[35] = -16'd3;
			assign values[36] = 16'd3;
			assign values[37] = -16'd5;
			assign values[38] = -16'd7;
			assign values[39] = 16'd7;
			assign values[40] = -16'd1;
			assign values[41] = 16'd4;
			assign values[42] = -16'd3;
			assign values[43] = 16'd4;
			assign values[44] = -16'd7;
			assign values[45] = -16'd5;
			assign values[46] = 16'd3;
			assign values[47] = 16'd7;
		end
		else if (I == 1) begin
			assign values[0] = -16'd2;
			assign values[1] = 16'd6;
			assign values[2] = -16'd3;
			assign values[3] = -16'd3;
			assign values[4] = 16'd4;
			assign values[5] = -16'd1;
			assign values[6] = 16'd2;
			assign values[7] = -16'd4;
			assign values[8] = 16'd3;
			assign values[9] = 16'd0;
			assign values[10] = -16'd3;
			assign values[11] = -16'd5;
			assign values[12] = 16'd6;
			assign values[13] = 16'd4;
			assign values[14] = -16'd8;
			assign values[15] = -16'd7;
			assign values[16] = -16'd4;
			assign values[17] = -16'd1;
			assign values[18] = -16'd7;
			assign values[19] = -16'd8;
			assign values[20] = 16'd6;
			assign values[21] = 16'd3;
			assign values[22] = -16'd3;
			assign values[23] = -16'd4;
			assign values[24] = 16'd5;
			assign values[25] = -16'd6;
			assign values[26] = -16'd5;
			assign values[27] = 16'd3;
			assign values[28] = 16'd6;
			assign values[29] = -16'd5;
			assign values[30] = 16'd4;
			assign values[31] = -16'd6;
			assign values[32] = 16'd2;
			assign values[33] = -16'd5;
			assign values[34] = -16'd8;
			assign values[35] = 16'd0;
			assign values[36] = 16'd6;
			assign values[37] = -16'd3;
			assign values[38] = 16'd5;
			assign values[39] = -16'd4;
			assign values[40] = -16'd3;
			assign values[41] = 16'd6;
			assign values[42] = 16'd2;
			assign values[43] = -16'd4;
			assign values[44] = -16'd6;
			assign values[45] = -16'd1;
			assign values[46] = -16'd2;
			assign values[47] = -16'd6;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule

module l3_fc3_16_12_16_1_4(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 16;
	parameter N = 12;
	parameter T = 16;
	parameter P = 4;
	parameter L = 3;

	// Top level ports
	input                 clk, reset, input_valid, output_ready;
	input signed [T-1:0]  input_data;
	output signed [T-1:0] output_data;
	output logic          output_valid, input_ready;

	// Internal signals (connect control with datapath)
	localparam                VECTOR_SIZE=$clog2(N);
	localparam               MATRIX_SIZE=$clog2(M*N/P);
	logic [VECTOR_SIZE-1:0]  addr_x;
	logic [MATRIX_SIZE-1:0]  addr_w;
	logic [P-1:0] f_sel;
	logic                    wr_en_x, wr_en_w, clear_acc, en_acc;

	//Instantiate Control
	control_gen #(M,N,T,P)control1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .output_ready(output_ready), .output_valid(output_valid), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .wr_en_w(wr_en_w), .clear_acc(clear_acc), .en_acc(en_acc), .f_sel(f_sel));

	//Instantiate Datapath
	datapath_gen_p3_relu #(M,N,T,P,L)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p3(clk, addr, z);
	parameter			 M = 16;
	parameter			 N = 12;
	parameter			 T = 16;
	parameter			 P = 4;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = 16'd4;
			assign values[1] = -16'd1;
			assign values[2] = -16'd1;
			assign values[3] = -16'd1;
			assign values[4] = 16'd3;
			assign values[5] = 16'd1;
			assign values[6] = -16'd2;
			assign values[7] = -16'd3;
			assign values[8] = 16'd4;
			assign values[9] = -16'd2;
			assign values[10] = 16'd6;
			assign values[11] = 16'd2;
			assign values[12] = -16'd2;
			assign values[13] = -16'd1;
			assign values[14] = 16'd0;
			assign values[15] = 16'd5;
			assign values[16] = -16'd7;
			assign values[17] = -16'd8;
			assign values[18] = 16'd2;
			assign values[19] = 16'd2;
			assign values[20] = -16'd6;
			assign values[21] = 16'd2;
			assign values[22] = -16'd3;
			assign values[23] = 16'd4;
			assign values[24] = 16'd7;
			assign values[25] = -16'd3;
			assign values[26] = 16'd6;
			assign values[27] = -16'd8;
			assign values[28] = 16'd3;
			assign values[29] = 16'd4;
			assign values[30] = 16'd3;
			assign values[31] = 16'd6;
			assign values[32] = 16'd6;
			assign values[33] = 16'd2;
			assign values[34] = 16'd1;
			assign values[35] = -16'd6;
			assign values[36] = -16'd3;
			assign values[37] = -16'd5;
			assign values[38] = -16'd4;
			assign values[39] = 16'd5;
			assign values[40] = 16'd6;
			assign values[41] = 16'd6;
			assign values[42] = -16'd5;
			assign values[43] = 16'd1;
			assign values[44] = -16'd1;
			assign values[45] = -16'd2;
			assign values[46] = -16'd1;
			assign values[47] = -16'd6;
		end
		else if (I == 1) begin
			assign values[0] = 16'd3;
			assign values[1] = 16'd3;
			assign values[2] = 16'd6;
			assign values[3] = -16'd5;
			assign values[4] = -16'd1;
			assign values[5] = -16'd4;
			assign values[6] = 16'd7;
			assign values[7] = 16'd1;
			assign values[8] = -16'd1;
			assign values[9] = 16'd2;
			assign values[10] = 16'd0;
			assign values[11] = 16'd4;
			assign values[12] = -16'd7;
			assign values[13] = -16'd6;
			assign values[14] = -16'd7;
			assign values[15] = 16'd7;
			assign values[16] = -16'd1;
			assign values[17] = 16'd6;
			assign values[18] = 16'd0;
			assign values[19] = -16'd4;
			assign values[20] = -16'd7;
			assign values[21] = -16'd2;
			assign values[22] = 16'd6;
			assign values[23] = 16'd2;
			assign values[24] = -16'd6;
			assign values[25] = -16'd6;
			assign values[26] = 16'd5;
			assign values[27] = -16'd8;
			assign values[28] = -16'd4;
			assign values[29] = -16'd2;
			assign values[30] = -16'd8;
			assign values[31] = -16'd3;
			assign values[32] = 16'd0;
			assign values[33] = 16'd3;
			assign values[34] = 16'd1;
			assign values[35] = -16'd3;
			assign values[36] = -16'd1;
			assign values[37] = 16'd6;
			assign values[38] = 16'd5;
			assign values[39] = 16'd4;
			assign values[40] = -16'd5;
			assign values[41] = 16'd0;
			assign values[42] = 16'd5;
			assign values[43] = -16'd4;
			assign values[44] = 16'd6;
			assign values[45] = 16'd5;
			assign values[46] = -16'd5;
			assign values[47] = 16'd6;
		end
		else if (I == 2) begin
			assign values[0] = 16'd1;
			assign values[1] = -16'd6;
			assign values[2] = -16'd8;
			assign values[3] = 16'd3;
			assign values[4] = 16'd1;
			assign values[5] = -16'd2;
			assign values[6] = 16'd5;
			assign values[7] = -16'd3;
			assign values[8] = 16'd6;
			assign values[9] = -16'd4;
			assign values[10] = 16'd4;
			assign values[11] = 16'd1;
			assign values[12] = -16'd5;
			assign values[13] = -16'd6;
			assign values[14] = 16'd7;
			assign values[15] = 16'd2;
			assign values[16] = -16'd4;
			assign values[17] = -16'd1;
			assign values[18] = 16'd1;
			assign values[19] = 16'd3;
			assign values[20] = 16'd6;
			assign values[21] = -16'd7;
			assign values[22] = 16'd0;
			assign values[23] = -16'd8;
			assign values[24] = 16'd3;
			assign values[25] = 16'd1;
			assign values[26] = -16'd5;
			assign values[27] = 16'd5;
			assign values[28] = 16'd3;
			assign values[29] = -16'd8;
			assign values[30] = -16'd1;
			assign values[31] = 16'd2;
			assign values[32] = -16'd3;
			assign values[33] = -16'd3;
			assign values[34] = 16'd3;
			assign values[35] = -16'd7;
			assign values[36] = -16'd3;
			assign values[37] = -16'd4;
			assign values[38] = -16'd7;
			assign values[39] = -16'd8;
			assign values[40] = -16'd5;
			assign values[41] = -16'd4;
			assign values[42] = 16'd7;
			assign values[43] = 16'd0;
			assign values[44] = -16'd1;
			assign values[45] = -16'd4;
			assign values[46] = -16'd2;
			assign values[47] = -16'd2;
		end
		else if (I == 3) begin
			assign values[0] = 16'd5;
			assign values[1] = -16'd5;
			assign values[2] = 16'd6;
			assign values[3] = 16'd1;
			assign values[4] = 16'd1;
			assign values[5] = 16'd4;
			assign values[6] = -16'd4;
			assign values[7] = -16'd3;
			assign values[8] = -16'd1;
			assign values[9] = -16'd6;
			assign values[10] = 16'd0;
			assign values[11] = 16'd7;
			assign values[12] = -16'd7;
			assign values[13] = -16'd6;
			assign values[14] = 16'd2;
			assign values[15] = -16'd4;
			assign values[16] = 16'd5;
			assign values[17] = -16'd8;
			assign values[18] = -16'd8;
			assign values[19] = 16'd6;
			assign values[20] = -16'd6;
			assign values[21] = -16'd7;
			assign values[22] = 16'd5;
			assign values[23] = 16'd1;
			assign values[24] = -16'd7;
			assign values[25] = -16'd2;
			assign values[26] = 16'd7;
			assign values[27] = 16'd7;
			assign values[28] = -16'd8;
			assign values[29] = 16'd0;
			assign values[30] = -16'd7;
			assign values[31] = -16'd6;
			assign values[32] = 16'd3;
			assign values[33] = 16'd7;
			assign values[34] = -16'd5;
			assign values[35] = 16'd7;
			assign values[36] = -16'd6;
			assign values[37] = 16'd1;
			assign values[38] = 16'd7;
			assign values[39] = 16'd1;
			assign values[40] = 16'd7;
			assign values[41] = -16'd2;
			assign values[42] = 16'd4;
			assign values[43] = -16'd2;
			assign values[44] = -16'd4;
			assign values[45] = 16'd1;
			assign values[46] = -16'd5;
			assign values[47] = -16'd1;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
