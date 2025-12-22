// Memory module for neural network
module memory #(parameter WIDTH=8, parameter SIZE=64) (
    input clk,
    input [$clog2(SIZE)-1:0] addr,
    input [WIDTH-1:0] data_in,
    input wr_en,
    output logic [WIDTH-1:0] data_out
);

    // Memory array
    logic [WIDTH-1:0] mem [SIZE-1:0];

    // Write operation
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[addr] <= data_in;
    end

    // Read operation
    assign data_out = mem[addr];
endmodule
