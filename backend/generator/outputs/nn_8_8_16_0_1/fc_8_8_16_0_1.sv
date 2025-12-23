`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3.sv"

module fc_8_8_16_0_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 8;
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
	datapath_gen_p3 #(M,N,T,P)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));
endmodule

module rom_gem_p1(clk, addr, z);
	parameter			 M = 8;
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
			assign values[0] = 16'd0;
			assign values[1] = 16'd1;
			assign values[2] = 16'd2;
			assign values[3] = 16'd3;
			assign values[4] = 16'd4;
			assign values[5] = 16'd5;
			assign values[6] = 16'd6;
			assign values[7] = 16'd7;
			assign values[8] = 16'd8;
			assign values[9] = 16'd9;
			assign values[10] = 16'd10;
			assign values[11] = 16'd11;
			assign values[12] = 16'd12;
			assign values[13] = 16'd13;
			assign values[14] = 16'd14;
			assign values[15] = 16'd15;
			assign values[16] = 16'd16;
			assign values[17] = 16'd17;
			assign values[18] = 16'd18;
			assign values[19] = 16'd19;
			assign values[20] = 16'd20;
			assign values[21] = 16'd21;
			assign values[22] = 16'd22;
			assign values[23] = 16'd23;
			assign values[24] = 16'd24;
			assign values[25] = 16'd25;
			assign values[26] = 16'd26;
			assign values[27] = 16'd27;
			assign values[28] = 16'd28;
			assign values[29] = 16'd29;
			assign values[30] = 16'd30;
			assign values[31] = 16'd31;
			assign values[32] = 16'd32;
			assign values[33] = 16'd33;
			assign values[34] = 16'd34;
			assign values[35] = 16'd35;
			assign values[36] = 16'd36;
			assign values[37] = 16'd37;
			assign values[38] = 16'd38;
			assign values[39] = 16'd39;
			assign values[40] = 16'd40;
			assign values[41] = 16'd41;
			assign values[42] = 16'd42;
			assign values[43] = 16'd43;
			assign values[44] = 16'd44;
			assign values[45] = 16'd45;
			assign values[46] = 16'd46;
			assign values[47] = 16'd47;
			assign values[48] = 16'd48;
			assign values[49] = 16'd49;
			assign values[50] = 16'd50;
			assign values[51] = 16'd51;
			assign values[52] = 16'd52;
			assign values[53] = 16'd53;
			assign values[54] = 16'd54;
			assign values[55] = 16'd55;
			assign values[56] = 16'd56;
			assign values[57] = 16'd57;
			assign values[58] = 16'd58;
			assign values[59] = 16'd59;
			assign values[60] = 16'd60;
			assign values[61] = 16'd61;
			assign values[62] = 16'd62;
			assign values[63] = 16'd63;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
