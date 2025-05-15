// Controller module for neural network
module control_gen #(parameter M=16, parameter N=8, parameter T=8, parameter P=1) (
    input clk, reset, input_valid, output_ready,
    output logic input_ready, output_valid,
    output logic [($clog2(N))-1:0] addr_x,
    output logic wr_en_x,
    output logic [($clog2((M*N)/P))-1:0] addr_w,
    output logic wr_en_w,
    output logic clear_acc, en_acc,
    output logic [P-1:0] f_sel
);

    // Define states
    typedef enum {S_IDLE, S_LOAD_X, S_LOAD_W, S_COMPUTE, S_OUTPUT} state_t;
    state_t state, next_state;

    // Counter for input vector elements
    logic [($clog2(N))-1:0] count_x;
    // Counter for weight matrix elements
    logic [($clog2((M*N)/P))-1:0] count_w;
    // Counter for output vector elements
    logic [($clog2(M))-1:0] count_y;

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_IDLE;
            count_x <= 0;
            count_w <= 0;
            count_y <= 0;
        end else begin
            state <= next_state;
            
            // Update counters based on state
            case (state)
                S_LOAD_X: begin
                    if (input_valid && input_ready)
                        count_x <= count_x + 1;
                end
                S_LOAD_W: begin
                    count_w <= count_w + 1;
                    if (count_w == (N*M/P)-1)
                        count_w <= 0;
                end
                S_COMPUTE: begin
                    if (count_x < N-1)
                        count_x <= count_x + 1;
                    else begin
                        count_x <= 0;
                        count_y <= count_y + P;
                        if (count_y >= M-P)
                            count_y <= 0;
                    end
                end
            endcase
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (input_valid)
                    next_state = S_LOAD_X;
            end
            S_LOAD_X: begin
                if (count_x == N-1 && input_valid && input_ready)
                    next_state = S_LOAD_W;
            end
            S_LOAD_W: begin
                if (count_w == (N*M/P)-1)
                    next_state = S_COMPUTE;
            end
            S_COMPUTE: begin
                if (count_x == N-1 && count_y == M-P)
                    next_state = S_OUTPUT;
            end
            S_OUTPUT: begin
                if (output_ready)
                    next_state = S_IDLE;
            end
        endcase
    end

    // Output logic
    always_comb begin
        // Default values
        input_ready = 0;
        output_valid = 0;
        addr_x = 0;
        wr_en_x = 0;
        addr_w = 0;
        wr_en_w = 0;
        clear_acc = 0;
        en_acc = 0;
        f_sel = 0;
        
        case (state)
            S_IDLE: begin
                input_ready = 1;
                clear_acc = 1;
            end
            S_LOAD_X: begin
                input_ready = 1;
                addr_x = count_x;
                wr_en_x = 1;
            end
            S_LOAD_W: begin
                addr_w = count_w;
                wr_en_w = 1;
            end
            S_COMPUTE: begin
                addr_x = count_x;
                addr_w = count_x + count_y * N/P;
                en_acc = 1;
                
                // Set f_sel based on current output row
                for (int i = 0; i < P; i++) begin
                    if (count_y + i < M)
                        f_sel[i] = 1;
                end
                
                if (count_x == 0)
                    clear_acc = 1;
            end
            S_OUTPUT: begin
                output_valid = 1;
            end
        endcase
    end
endmodule
