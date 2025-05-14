`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_6_6_8_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 6;
	parameter N = 6;
	parameter T = 8;
	parameter P = 1;

	// Ignoring L parameter, layer intended to be used alone

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
	datapath_gen_p3_relu #(M,N,T,P)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p1(clk, addr, z);
	parameter			 M = 6;
	parameter			 N = 6;
	parameter			 T = 8;
	parameter			 P = 1;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = 8'd4;
			assign values[1] = 8'd3;
			assign values[2] = -8'd6;
			assign values[3] = -8'd6;
			assign values[4] = -8'd2;
			assign values[5] = -8'd1;
			assign values[6] = -8'd1;
			assign values[7] = 8'd6;
			assign values[8] = -8'd1;
			assign values[9] = -8'd8;
			assign values[10] = -8'd7;
			assign values[11] = -8'd5;
			assign values[12] = 8'd1;
			assign values[13] = 8'd5;
			assign values[14] = 8'd0;
			assign values[15] = 8'd2;
			assign values[16] = -8'd1;
			assign values[17] = -8'd6;
			assign values[18] = -8'd2;
			assign values[19] = 8'd1;
			assign values[20] = 8'd4;
			assign values[21] = 8'd1;
			assign values[22] = -8'd4;
			assign values[23] = -8'd4;
			assign values[24] = 8'd6;
			assign values[25] = -8'd2;
			assign values[26] = -8'd7;
			assign values[27] = 8'd7;
			assign values[28] = -8'd4;
			assign values[29] = 8'd2;
			assign values[30] = -8'd3;
			assign values[31] = -8'd8;
			assign values[32] = -8'd2;
			assign values[33] = -8'd1;
			assign values[34] = -8'd5;
			assign values[35] = 8'd4;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
