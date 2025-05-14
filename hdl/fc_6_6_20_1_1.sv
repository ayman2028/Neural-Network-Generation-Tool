`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_6_6_20_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 6;
	parameter N = 6;
	parameter T = 20;
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
	parameter			 T = 20;
	parameter			 P = 1;
	parameter			 I = 0;
	localparam			 MATRIX_SIZE = $clog2(M*N/P);

	input clk;
	input [MATRIX_SIZE-1:0] addr;
	output logic signed [T-1:0] z;

	generate
		logic signed [T-1:0] values[(M*N/P)-1:0];
		if (I == 0) begin
			assign values[0] = -20'd147;
			assign values[1] = 20'd357;
			assign values[2] = -20'd143;
			assign values[3] = -20'd75;
			assign values[4] = 20'd76;
			assign values[5] = -20'd511;
			assign values[6] = 20'd213;
			assign values[7] = 20'd262;
			assign values[8] = -20'd315;
			assign values[9] = -20'd490;
			assign values[10] = -20'd303;
			assign values[11] = -20'd346;
			assign values[12] = -20'd383;
			assign values[13] = -20'd434;
			assign values[14] = 20'd365;
			assign values[15] = 20'd397;
			assign values[16] = 20'd25;
			assign values[17] = 20'd489;
			assign values[18] = 20'd129;
			assign values[19] = -20'd258;
			assign values[20] = 20'd90;
			assign values[21] = -20'd328;
			assign values[22] = 20'd375;
			assign values[23] = 20'd132;
			assign values[24] = -20'd485;
			assign values[25] = -20'd413;
			assign values[26] = -20'd26;
			assign values[27] = 20'd402;
			assign values[28] = 20'd7;
			assign values[29] = -20'd330;
			assign values[30] = -20'd259;
			assign values[31] = 20'd372;
			assign values[32] = -20'd485;
			assign values[33] = 20'd110;
			assign values[34] = -20'd215;
			assign values[35] = 20'd103;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
