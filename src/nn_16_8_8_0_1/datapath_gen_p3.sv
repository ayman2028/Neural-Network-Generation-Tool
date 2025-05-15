// Datapath module for neural network
module datapath_gen_p3 #(parameter M=16, parameter N=8, parameter T=8, parameter P=1, parameter L=0) (
    input clk, reset,
    input signed [T-1:0] input_data,
    input [($clog2(N))-1:0] addr_x,
    input wr_en_x,
    input [($clog2((M*N)/P))-1:0] addr_w,
    input clear_acc, en_acc,
    input [P-1:0] f_sel,
    output signed [T-1:0] m_data_out_y
);

    // Internal signals
    logic signed [T-1:0] x_out;
    logic signed [T-1:0] w_out [P-1:0];
    logic signed [T-1:0] mac_out [P-1:0];
    logic signed [T-1:0] acc_out [P-1:0];

    // Memory for input vector X
    memory #(T, N) mem_x (
        .clk(clk),
        .addr(addr_x),
        .data_in(input_data),
        .wr_en(wr_en_x),
        .data_out(x_out)
    );

    // ROM for weights
    generate
        for (genvar i = 0; i < P; i++) begin : weight_roms
            if (L == 0)
                rom_gem_p1 #(M, N, T, P, i) rom_w (
                    .clk(clk),
                    .addr(addr_w),
                    .z(w_out[i])
                );
            else
                rom_gem_p #(M, N, T, P, i) rom_w (
                    .clk(clk),
                    .addr(addr_w),
                    .z(w_out[i])
                );
        end
    endgenerate

    // MAC units
    generate
        for (genvar i = 0; i < P; i++) begin : mac_units
            // Multiply
            logic signed [2*T-1:0] mult_result;
            assign mult_result = x_out * w_out[i];

            // Saturate multiplication result
            always_comb begin
                if (mult_result > (2**(T-1))-1)
                    mac_out[i] = (2**(T-1))-1;
                else if (mult_result < -(2**(T-1)))
                    mac_out[i] = -(2**(T-1));
                else
                    mac_out[i] = mult_result[T-1:0];
            end

            // Accumulator
            logic signed [T-1:0] acc_next;
            always_comb begin
                if (clear_acc)
                    acc_next = mac_out[i];
                else if (en_acc) begin
                    // Saturating addition
                    logic signed [T:0] add_result;
                    add_result = acc_out[i] + mac_out[i];
                    
                    if (add_result > (2**(T-1))-1)
                        acc_next = (2**(T-1))-1;
                    else if (add_result < -(2**(T-1)))
                        acc_next = -(2**(T-1));
                    else
                        acc_next = add_result[T-1:0];
                end else
                    acc_next = acc_out[i];
            end

            // Register
            always_ff @(posedge clk or posedge reset) begin
                if (reset)
                    acc_out[i] <= 0;
                else
                    acc_out[i] <= acc_next;
            end
        end
    endgenerate

    // Output selection
    logic signed [T-1:0] selected_output;
    always_comb begin
        selected_output = 0;
        for (int i = 0; i < P; i++) begin
            if (f_sel[i])
                selected_output = acc_out[i];
        end
    end

    // Output assignment
    assign m_data_out_y = selected_output;
endmodule
