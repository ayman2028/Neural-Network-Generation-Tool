`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_16_8_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 16;
	parameter N = 8;
	parameter T = 16;
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
	parameter			 M = 16;
	parameter			 N = 8;
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
			assign values[0] = 16'd83;
			assign values[1] = 16'd108;
			assign values[2] = 16'd106;
			assign values[3] = -16'd110;
			assign values[4] = 16'd59;
			assign values[5] = -16'd94;
			assign values[6] = -16'd20;
			assign values[7] = -16'd15;
			assign values[8] = 16'd28;
			assign values[9] = 16'd53;
			assign values[10] = 16'd97;
			assign values[11] = 16'd99;
			assign values[12] = 16'd63;
			assign values[13] = -16'd8;
			assign values[14] = 16'd89;
			assign values[15] = -16'd6;
			assign values[16] = -16'd10;
			assign values[17] = 16'd116;
			assign values[18] = 16'd87;
			assign values[19] = -16'd78;
			assign values[20] = 16'd26;
			assign values[21] = 16'd125;
			assign values[22] = 16'd6;
			assign values[23] = -16'd96;
			assign values[24] = -16'd77;
			assign values[25] = -16'd120;
			assign values[26] = -16'd2;
			assign values[27] = 16'd85;
			assign values[28] = -16'd84;
			assign values[29] = -16'd123;
			assign values[30] = 16'd105;
			assign values[31] = -16'd128;
			assign values[32] = 16'd113;
			assign values[33] = 16'd84;
			assign values[34] = -16'd110;
			assign values[35] = 16'd44;
			assign values[36] = 16'd118;
			assign values[37] = -16'd2;
			assign values[38] = -16'd98;
			assign values[39] = 16'd18;
			assign values[40] = -16'd76;
			assign values[41] = 16'd127;
			assign values[42] = -16'd11;
			assign values[43] = 16'd115;
			assign values[44] = -16'd9;
			assign values[45] = -16'd50;
			assign values[46] = -16'd19;
			assign values[47] = 16'd109;
			assign values[48] = -16'd62;
			assign values[49] = -16'd60;
			assign values[50] = -16'd97;
			assign values[51] = 16'd93;
			assign values[52] = -16'd63;
			assign values[53] = 16'd38;
			assign values[54] = 16'd125;
			assign values[55] = -16'd12;
			assign values[56] = 16'd46;
			assign values[57] = -16'd4;
			assign values[58] = -16'd54;
			assign values[59] = 16'd91;
			assign values[60] = 16'd1;
			assign values[61] = -16'd77;
			assign values[62] = 16'd91;
			assign values[63] = -16'd14;
			assign values[64] = -16'd121;
			assign values[65] = 16'd109;
			assign values[66] = -16'd98;
			assign values[67] = 16'd125;
			assign values[68] = -16'd21;
			assign values[69] = -16'd68;
			assign values[70] = 16'd15;
			assign values[71] = 16'd31;
			assign values[72] = -16'd69;
			assign values[73] = -16'd124;
			assign values[74] = 16'd18;
			assign values[75] = 16'd50;
			assign values[76] = -16'd46;
			assign values[77] = 16'd127;
			assign values[78] = 16'd31;
			assign values[79] = 16'd21;
			assign values[80] = -16'd60;
			assign values[81] = 16'd63;
			assign values[82] = -16'd14;
			assign values[83] = 16'd5;
			assign values[84] = -16'd27;
			assign values[85] = -16'd17;
			assign values[86] = 16'd122;
			assign values[87] = -16'd109;
			assign values[88] = 16'd107;
			assign values[89] = -16'd60;
			assign values[90] = 16'd110;
			assign values[91] = -16'd20;
			assign values[92] = -16'd9;
			assign values[93] = 16'd73;
			assign values[94] = 16'd94;
			assign values[95] = -16'd1;
			assign values[96] = 16'd54;
			assign values[97] = 16'd125;
			assign values[98] = -16'd4;
			assign values[99] = -16'd94;
			assign values[100] = -16'd71;
			assign values[101] = -16'd116;
			assign values[102] = 16'd65;
			assign values[103] = -16'd11;
			assign values[104] = -16'd112;
			assign values[105] = -16'd44;
			assign values[106] = -16'd89;
			assign values[107] = -16'd29;
			assign values[108] = -16'd45;
			assign values[109] = 16'd71;
			assign values[110] = 16'd120;
			assign values[111] = 16'd23;
			assign values[112] = 16'd6;
			assign values[113] = -16'd22;
			assign values[114] = -16'd99;
			assign values[115] = 16'd107;
			assign values[116] = 16'd89;
			assign values[117] = -16'd105;
			assign values[118] = 16'd126;
			assign values[119] = 16'd69;
			assign values[120] = -16'd37;
			assign values[121] = 16'd109;
			assign values[122] = -16'd79;
			assign values[123] = 16'd82;
			assign values[124] = 16'd54;
			assign values[125] = -16'd112;
			assign values[126] = -16'd47;
			assign values[127] = -16'd19;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
