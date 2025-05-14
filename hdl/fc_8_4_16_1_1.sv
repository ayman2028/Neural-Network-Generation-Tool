`include "memory.sv"
`include "controller.sv"
`include "datapath_gen_p3_relu.sv"

module fc_8_4_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);
	parameter M = 8;
	parameter N = 4;
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
			assign values[0] = -16'd61;
			assign values[1] = 16'd105;
			assign values[2] = -16'd75;
			assign values[3] = -16'd77;
			assign values[4] = -16'd62;
			assign values[5] = -16'd102;
			assign values[6] = -16'd109;
			assign values[7] = 16'd51;
			assign values[8] = -16'd116;
			assign values[9] = -16'd49;
			assign values[10] = -16'd116;
			assign values[11] = 16'd107;
			assign values[12] = -16'd48;
			assign values[13] = -16'd108;
			assign values[14] = 16'd65;
			assign values[15] = -16'd45;
			assign values[16] = 16'd82;
			assign values[17] = 16'd114;
			assign values[18] = -16'd21;
			assign values[19] = -16'd85;
			assign values[20] = -16'd80;
			assign values[21] = -16'd73;
			assign values[22] = -16'd118;
			assign values[23] = 16'd53;
			assign values[24] = 16'd48;
			assign values[25] = -16'd69;
			assign values[26] = -16'd109;
			assign values[27] = -16'd57;
			assign values[28] = -16'd34;
			assign values[29] = -16'd59;
			assign values[30] = -16'd48;
			assign values[31] = 16'd34;
		end
	endgenerate

	always_ff @(posedge clk) begin
		z <= values[addr];
	end
endmodule
