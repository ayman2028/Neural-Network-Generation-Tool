`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3.sv"

module fc_16_8_8_2_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 16;
	parameter N = 8;
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
	datapath_gen_p3 #(M,N,T,P)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p1(clk, addr, z);
	parameter			 M = 16;
	parameter			 N = 8;
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
			assign values[0] = 8'd1;
			assign values[1] = 8'd1;
			assign values[2] = 8'd1;
			assign values[3] = 8'd1;
			assign values[4] = 8'd1;
			assign values[5] = 8'd1;
			assign values[6] = 8'd1;
			assign values[7] = 8'd1;
			assign values[8] = 8'd1;
			assign values[9] = 8'd1;
			assign values[10] = 8'd1;
			assign values[11] = 8'd1;
			assign values[12] = 8'd1;
			assign values[13] = 8'd1;
			assign values[14] = 8'd1;
			assign values[15] = 8'd1;
			assign values[16] = 8'd1;
			assign values[17] = 8'd1;
			assign values[18] = 8'd1;
			assign values[19] = 8'd1;
			assign values[20] = 8'd1;
			assign values[21] = 8'd1;
			assign values[22] = 8'd1;
			assign values[23] = 8'd1;
			assign values[24] = 8'd1;
			assign values[25] = 8'd1;
			assign values[26] = 8'd1;
			assign values[27] = 8'd1;
			assign values[28] = 8'd1;
			assign values[29] = 8'd1;
			assign values[30] = 8'd1;
			assign values[31] = 8'd1;
			assign values[32] = 8'd1;
			assign values[33] = 8'd1;
			assign values[34] = 8'd1;
			assign values[35] = 8'd1;
			assign values[36] = 8'd1;
			assign values[37] = 8'd1;
			assign values[38] = 8'd1;
			assign values[39] = 8'd1;
			assign values[40] = 8'd1;
			assign values[41] = 8'd1;
			assign values[42] = 8'd1;
			assign values[43] = 8'd1;
			assign values[44] = 8'd1;
			assign values[45] = 8'd1;
			assign values[46] = 8'd1;
			assign values[47] = 8'd1;
			assign values[48] = 8'd1;
			assign values[49] = 8'd1;
			assign values[50] = 8'd1;
			assign values[51] = 8'd1;
			assign values[52] = 8'd1;
			assign values[53] = 8'd1;
			assign values[54] = 8'd1;
			assign values[55] = 8'd1;
			assign values[56] = 8'd1;
			assign values[57] = 8'd1;
			assign values[58] = 8'd1;
			assign values[59] = 8'd1;
			assign values[60] = 8'd1;
			assign values[61] = 8'd1;
			assign values[62] = 8'd1;
			assign values[63] = 8'd1;
			assign values[64] = 8'd1;
			assign values[65] = 8'd1;
			assign values[66] = 8'd1;
			assign values[67] = 8'd1;
			assign values[68] = 8'd1;
			assign values[69] = 8'd1;
			assign values[70] = 8'd1;
			assign values[71] = 8'd1;
			assign values[72] = 8'd1;
			assign values[73] = 8'd1;
			assign values[74] = 8'd1;
			assign values[75] = 8'd1;
			assign values[76] = 8'd1;
			assign values[77] = 8'd1;
			assign values[78] = 8'd1;
			assign values[79] = 8'd1;
			assign values[80] = 8'd1;
			assign values[81] = 8'd1;
			assign values[82] = 8'd1;
			assign values[83] = 8'd1;
			assign values[84] = 8'd1;
			assign values[85] = 8'd1;
			assign values[86] = 8'd1;
			assign values[87] = 8'd1;
			assign values[88] = 8'd1;
			assign values[89] = 8'd1;
			assign values[90] = 8'd1;
			assign values[91] = 8'd1;
			assign values[92] = 8'd1;
			assign values[93] = 8'd1;
			assign values[94] = 8'd1;
			assign values[95] = 8'd1;
			assign values[96] = 8'd1;
			assign values[97] = 8'd1;
			assign values[98] = 8'd1;
			assign values[99] = 8'd1;
			assign values[100] = 8'd1;
			assign values[101] = 8'd1;
			assign values[102] = 8'd1;
			assign values[103] = 8'd1;
			assign values[104] = 8'd1;
			assign values[105] = 8'd1;
			assign values[106] = 8'd1;
			assign values[107] = 8'd1;
			assign values[108] = 8'd1;
			assign values[109] = 8'd1;
			assign values[110] = 8'd1;
			assign values[111] = 8'd1;
			assign values[112] = 8'd1;
			assign values[113] = 8'd1;
			assign values[114] = 8'd1;
			assign values[115] = 8'd1;
			assign values[116] = 8'd1;
			assign values[117] = 8'd1;
			assign values[118] = 8'd1;
			assign values[119] = 8'd1;
			assign values[120] = 8'd1;
			assign values[121] = 8'd1;
			assign values[122] = 8'd1;
			assign values[123] = 8'd1;
			assign values[124] = 8'd1;
			assign values[125] = 8'd1;
			assign values[126] = 8'd1;
			assign values[127] = 8'd1;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
