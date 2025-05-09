									
// datapath module
module datapath_gen_p3(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, en_acc, m_data_out_y,f_sel);
	
	parameter M = 8;
	parameter N = 8;
	parameter T = 8;
	parameter P = 1;
	parameter L = 1;
    localparam              VECTOR_SIZE=$clog2(N);
    localparam              MATRIX_SIZE=$clog2(M*N/P);
	input 				    clk, reset, wr_en_x, clear_acc, en_acc;
    input [VECTOR_SIZE-1:0]   addr_x;
    input [MATRIX_SIZE-1:0]   addr_w;
	input [P-1:0]             f_sel;
	input signed [T-1:0]		input_data; 
	output logic [T-1:0] 	m_data_out_y;

	logic [T-1:0] vector_out;
	logic signed [P-1:0][T-1:0] f;
	//Delete Later
	logic wr_en_w;	
	
	assign m_data_out_y = f[f_sel];

    // Vector 4 words, 12 bits each
    memory #(T, N)vectorMem(.clk(clk), .data_in(input_data), .data_out(vector_out), .addr(addr_x), .wr_en(wr_en_x));
    
    
    // Reset of the dapath can either happen by whole system reset or accumulation reset
    logic datapath_reset;
    assign datapath_reset = clear_acc || reset;
    
// Instantiate Saturating MAC unit on datapath (save file on same directory)
    genSatMac_p3#(M,N,T,P,L) satmac(.clk(clk), .reset(datapath_reset), .a(vector_out), .valid_in(1'b1), .f(f), .valid_out(),.en_acc(en_acc), .addr_w(addr_w));
    
endmodule

module genSatMac_p3(clk, reset, a, valid_in, f, valid_out, en_acc,addr_w);
	parameter M = 3;
	parameter N = 3;
	parameter T = 8;
	parameter P = 1;
	parameter L = 1;
	
	localparam              MATRIX_SIZE=$clog2(M*N/P);
    input [MATRIX_SIZE-1:0]             addr_w;
	input logic signed [T-1:0] a; 
	input clk, reset, valid_in,	en_acc;
	output logic [P-1:0][T-1:0] f;
	output logic valid_out[P-1:0];
	generate
		genvar i;
		for(i=0; i<P; i=i+1) begin:	my_sat_mac
			sat_mac_p3#(M,N,T,P,i,L) s(.clk(clk), .reset(reset), .a(a), .valid_in(valid_in), .f(f[i]), .valid_out(valid_out[i]), .en_acc(en_acc),.addr_w(addr_w));
		end
	endgenerate
endmodule


module sat_mac_p3(clk, reset, a, valid_in, f, valid_out, en_acc,addr_w);
	parameter M = 3;
	parameter N = 3;
	parameter T = 8;
	parameter P = 1;
	parameter I = 0;
	parameter L = 1; 
	localparam              MATRIX_SIZE=$clog2(M*N/P);
    input [MATRIX_SIZE-1:0]             addr_w;
 input clk, reset, valid_in, en_acc;
 input signed [T-1:0] a;
 output logic signed [T-1:0] f;
 output logic valid_out;	 	
 //Internal signals
 logic signed [T-1:0] a_out, b_out, b_rom;		  
 logic signed [T-1:0] f_in, m, sum, saturate, m_in, product;  
 logic enable_ab, enable_f;
 logic overflow_check, overflow;						  
 parameter signed [T-1:0] max_number = 2**(T-1)-1;
 parameter signed [T-1:0] min_number = -(2**(T-1));
 
 logic signed [T*2-1:0] multiply ; 
 logic signed [T*2-1:0] atemp,btemp, maxtemp, mintemp ;
	 
 //control2 crtl(.clk(clk), .reset(reset), .valid_in(valid_in), .valid_out(valid_out), .enable_ab(enable_ab), .enable_f(enable_f));
 generate
	if(L==1) begin
		rom_gem_p1 #(M,N,T,P,I) rom1(.clk(clk), .addr(addr_w), .z(b_rom));
	end
	else if (L==2) begin
		rom_gem_p2 #(M,N,T,P,I) rom2(.clk(clk), .addr(addr_w), .z(b_rom));
	end	else if (L==3) begin
		rom_gem_p3 #(M,N,T,P,I) rom3(.clk(clk), .addr(addr_w), .z(b_rom));
	end
 endgenerate
 
 always_ff @(posedge clk) begin
	 valid_out <= valid_in;
 	 if (reset)
		 f <= 0;
	 else if (en_acc)
		 f <= f_in;	
	
	m <=  m_in;
 end
 
 
 always_comb begin
	 multiply = 0;
	 //atemp = 0;
	 //btemp = 0;
	 atemp = a;
	 btemp = b_rom;
	 maxtemp = max_number;
	 mintemp = min_number;
	 product = a*b_rom;	 
	 multiply = atemp * btemp;
	 sum = m + f;
	 
	 
	 
	 if ((!(a[T-1] ^ b_rom[T-1])) && multiply >= max_number) //Positive overflow
		m_in = max_number;
	else if(multiply<=min_number)// Negative overflow
		m_in = min_number;
	else
		m_in = product;	
		
	 overflow_check = !(f[T-1] ^ m[T-1]); 
	 
	 if ((overflow_check == 1) && (sum[T-1] != f[T-1]))	
		 overflow = 1;
	else 
	 	overflow = 0;
	
 	 if (f[T-1] == 1) 
		  saturate = min_number;
	  else
		  saturate = max_number;
		  
	 if (overflow == 1) 
		 f_in = saturate;
	 else
		 f_in = sum;
	
 end 
endmodule