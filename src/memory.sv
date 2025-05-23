 module memory(clk, data_in, data_out, addr, wr_en);
   
    parameter                   WIDTH=16, SIZE=64;
    localparam                  LOGSIZE=$clog2(SIZE);
    input [WIDTH-1:0]           data_in;
    output logic [WIDTH-1:0]    data_out;
    input [LOGSIZE-1:0]         addr;
    input                       clk, wr_en;
    
    logic [SIZE-1:0][WIDTH-1:0] mem;
    
// 	always_comb begin
// 		data_out = mem[addr];
// 	end
	
	
    always_ff @(posedge clk) begin
        data_out <= mem[addr];
        if (wr_en)
            mem[addr] <= data_in;
    end
endmodule


