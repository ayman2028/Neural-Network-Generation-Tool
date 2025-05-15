`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_6_8_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 6;
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
	parameter			 M = 6;
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
			assign values[0] = -16'd24;
			assign values[1] = 16'd77;
			assign values[2] = -16'd55;
			assign values[3] = 16'd30;
			assign values[4] = -16'd87;
			assign values[5] = 16'd57;
			assign values[6] = -16'd103;
			assign values[7] = 16'd97;
			assign values[8] = -16'd101;
			assign values[9] = 16'd110;
			assign values[10] = 16'd18;
			assign values[11] = -16'd46;
			assign values[12] = -16'd46;
			assign values[13] = -16'd126;
			assign values[14] = -16'd20;
			assign values[15] = -16'd121;
			assign values[16] = 16'd98;
			assign values[17] = -16'd114;
			assign values[18] = 16'd98;
			assign values[19] = 16'd37;
			assign values[20] = -16'd113;
			assign values[21] = -16'd106;
			assign values[22] = -16'd40;
			assign values[23] = -16'd34;
			assign values[24] = -16'd27;
			assign values[25] = -16'd120;
			assign values[26] = 16'd107;
			assign values[27] = 16'd68;
			assign values[28] = 16'd42;
			assign values[29] = -16'd25;
			assign values[30] = -16'd92;
			assign values[31] = -16'd110;
			assign values[32] = -16'd76;
			assign values[33] = -16'd18;
			assign values[34] = 16'd48;
			assign values[35] = -16'd35;
			assign values[36] = -16'd89;
			assign values[37] = 16'd73;
			assign values[38] = -16'd66;
			assign values[39] = -16'd62;
			assign values[40] = 16'd55;
			assign values[41] = 16'd81;
			assign values[42] = 16'd20;
			assign values[43] = -16'd119;
			assign values[44] = 16'd83;
			assign values[45] = -16'd128;
			assign values[46] = -16'd112;
			assign values[47] = 16'd53;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
