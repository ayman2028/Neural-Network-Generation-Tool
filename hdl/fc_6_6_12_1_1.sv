`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_6_6_12_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 6;
	parameter N = 6;
	parameter T = 12;
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
	parameter			 T = 12;
	parameter			 P = 1;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = 12'd23;
			assign values[1] = 12'd9;
			assign values[2] = 12'd20;
			assign values[3] = -12'd14;
			assign values[4] = -12'd10;
			assign values[5] = 12'd29;
			assign values[6] = 12'd15;
			assign values[7] = -12'd4;
			assign values[8] = 12'd7;
			assign values[9] = 12'd4;
			assign values[10] = -12'd1;
			assign values[11] = -12'd1;
			assign values[12] = -12'd9;
			assign values[13] = -12'd4;
			assign values[14] = -12'd30;
			assign values[15] = -12'd13;
			assign values[16] = -12'd14;
			assign values[17] = -12'd14;
			assign values[18] = -12'd19;
			assign values[19] = 12'd17;
			assign values[20] = 12'd26;
			assign values[21] = 12'd20;
			assign values[22] = 12'd1;
			assign values[23] = 12'd28;
			assign values[24] = 12'd14;
			assign values[25] = -12'd21;
			assign values[26] = 12'd2;
			assign values[27] = -12'd26;
			assign values[28] = 12'd22;
			assign values[29] = 12'd23;
			assign values[30] = -12'd22;
			assign values[31] = 12'd14;
			assign values[32] = 12'd1;
			assign values[33] = 12'd30;
			assign values[34] = -12'd32;
			assign values[35] = 12'd23;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
