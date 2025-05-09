  
// control module
module control_gen(clk, reset, input_valid, input_ready, output_ready, output_valid, addr_x, wr_en_x, addr_w, wr_en_w, clear_acc, en_acc,f_sel);

    parameter M = 8;
	parameter N = 8;
	parameter T = 12;
	parameter P = 1;
    localparam              VECTOR_SIZE=$clog2(N); 
	localparam              ROWS_SIZE=$clog2(M);
    localparam              MATRIX_SIZE=$clog2(M*N/P);
	localparam              ROM_M_SIZE=$clog2(M/P);
    input logic clk, reset, input_valid, output_ready;
    output logic [VECTOR_SIZE-1:0]   addr_x;
    output logic [MATRIX_SIZE-1:0]             addr_w;
	output logic [P-1:0] f_sel;
    output logic wr_en_x, wr_en_w, clear_acc, en_acc, input_ready, output_valid;
    logic new_matrix;
	logic error;
	
    //-------------- FSM Logic ---------------------------------
    logic read_done, process_done, next_row, output_done; //Signals that indicate that the next state is ready.
	logic process_initialize;
	logic process_stall;
	
	logic read_initialize; // Represents the first time we enter state READ. This is used to recognize the first clock cycle that we are in state READ.
	//COUNTERs
	logic [MATRIX_SIZE:0] input_cnt;//Represents the number of inputs taken during a input of a matrix and a vector.
	logic [ROWS_SIZE-1:0] y_cnt;				//The number of outputs that we're sent.
	logic [VECTOR_SIZE:0] process_cnt;		//The number of elements in a row, used to keep track of the elements in the row of the matrix and vector.
	logic [ROM_M_SIZE-1:0] iteration_cnt;
	logic [P:0]   iteration_out;
	
    // States
    parameter [1:0] READ = 2'b00, PROCESS = 2'b01, DONE = 2'b10;
    logic [1:0] state, next_state;
    
    // Next state logic
    always_ff @(posedge clk) begin
		if (reset) begin
			state <= READ;
			new_matrix <= 0;
		end else
			state <= next_state;
	end
	
	// Combinational next state logic
	always_comb begin
		if((state==READ) && (read_done == 1)) begin
			next_state = PROCESS;			
		end else if ((state==PROCESS) && (process_done)) begin		 
			next_state = DONE;

		end else if ((state==DONE) && (next_row)) begin 		 
			next_state = PROCESS;

		end else if ((state==DONE) && (output_done)) begin
			next_state = READ;

		end else begin
			next_state = state;
		end
	end
    //Additional transitional logic.
	
	
	
	
	
	always_comb begin
		//Here we are determining the addresses for the matrix and vector.
		//If we're writing to the memory modules, then we are in the read state since we're taking inputs.
		//Since we want to read on the first clock cycle entering the state, we determine the value of the address
		// the cycle before we enter the state.
		if ((state == READ))begin
			
			//For part 2, we change the adddress if we're are in the first input only, the value of input_cnt will be updated for the follwoing CC.
			//if(read_initialize == 0) begin // We havenet taken an input yet.
//				 if (new_matrix == 0) begin // We're writing to the vector and not the matrix
//				 	//addr_x = M*N; //It will now be pointing to the first element in the vector.
//					 addr_x = 0;
//					addr_w = input_cnt;
//				 end else begin
//				 //WHEN WE ARE READING FROM INPUTS INTO THE MEMORY UNITS.
//					addr_x = 0; //NOTE: Chance to optimize by using pipelined multiplication.	 
//					addr_w = input_cnt;
//				end	 
//			end else begin 
//				//WHEN WE ARE READING FROM INPUTS INTO THE MEMORY UNITS.
//				addr_x = input_cnt - M*N; //NOTE: Chance to optimize by using pipelined multiplication.	 
//				addr_w = input_cnt;
//			end	
			addr_x = input_cnt;
			// Fixed inferred latch
			addr_w = 0;
			
		end else begin //If we're not reading then we are not writing to either of the memory units.
			//WHEN WE ARE READING FROM THE MEMORY UNITS.
			addr_x = process_cnt; 
			addr_w = iteration_cnt * N + process_cnt;  //row# * (# of elements in row) + The element in row we are reading 
		
		end
	end
	
	
	//This is alwaysff block for variables that require memory
	always_ff @(posedge clk) begin
		//When we reset, the counters should be set to 0.
		if (reset) begin
			input_cnt <= 0; 
			process_cnt <= 0;
			y_cnt <= 0;
			process_stall <= 0;
			read_initialize <= 0;
		end else begin
		
		if (read_initialize == 0 && state == READ && input_valid == 1) begin //This is how we know that this is the first cc we are taking the input.
			read_initialize <= 1; //For the next clock cycles in the READ state, this value will stay asserted. 
		end
		if (state == PROCESS) begin
			read_initialize <= 0;// when we exit the read state, this should be reset.
		end
		
		
		//During the next state the input_cnt will determine the addess for the input into either of the memory blocks.
		//This only occurs when we have a valid input.	
			if((state == READ))begin
				if(input_valid) begin
					
					//If we are in the READ state and a new input hasn't been taken yet, then we set the input_cnt to k^2.
					// This will take affect the following cc.
					//if(read_initialize == 0 && new_matrix == 0) begin
//						input_cnt <= M*N+1; //Since we're not taking a new matrix, the address will point to the vector. 
//					end else if (input_cnt < M*N+N) begin
//						input_cnt <= input_cnt + 1;
//					end	 

					if (input_cnt < N) begin
						input_cnt <= input_cnt + 1;
					end
				end
			end
			// When we enter the process state, we must reset the input counter for the next time we enter the read state.
			if (state == PROCESS)
				input_cnt <= 0;
		end
		// This is to create an empty clock cycle at the start of the process state in order to allow the read address to be set up for the next clock cyle. 
		if (state == PROCESS ) begin
				process_initialize = 1;
		end else begin
			process_initialize = 0;	 
		end
		
		//Adding stuff for pipelneing.
		if (state == PROCESS && process_initialize == 0) begin
			process_stall <= 1;
		end
		if (state == PROCESS && process_initialize == 1) begin
			process_stall <= ~process_stall;
		end
		
		
		//The process_cnt variable represents the number of times we multiplied and accumilated variables from the memory block and is used for addressing.	
		if (state == PROCESS && process_initialize ==1 && process_stall == 0) begin
			process_cnt <= process_cnt +1;
		end
		//When we exit the process state, the counter should be reset for the next 
		if (state == DONE) begin
			process_cnt <= 0;
		end
		//When we enter the read state, the y_cnt should be reset.
		// This variable represents the number of times we have output a value and is also used for addressing.
		//When we access a row in the matrix, the row will be represented by the y_cnt value. Thus, if we multiply it by K,
		// we should have an address pointing to the first element of that row. So if we add the process_cnt to Y_cnt*K, we should be able to 
		// access any element in any row.
		if((state == READ))begin
			y_cnt <= 0;
		end
		if (state == DONE && output_ready == 1) begin 
			y_cnt <= y_cnt +1;
			iteration_out <= iteration_out+1;
			if(y_cnt % P == 0) begin
				iteration_cnt <= iteration_cnt+1;
			end
		end
		if((state == PROCESS))begin
			iteration_out <= 0;
		end
		if((state == READ))begin
			iteration_cnt <= 0;
		end
			
	end
	
	
    // Control signals output
    always_comb begin
		f_sel = iteration_out;
		if (reset==1) begin
			error = 0;
		end 
		
		
		//We only write to a memory state when we are in the read state and there is a valid input.
		if (state == READ && input_valid == 1) begin //We want to take the firt input at the beginning of the first clock cyle in the read state.
			//if(read_initialize == 0 && new_matrix == 0) begin
//				wr_en_w = 0;
//				wr_en_x = 1;
//			end else begin
//				wr_en_w = 1;
//				wr_en_x = 0;
//			end
			wr_en_x = 1;

		end else begin
		    //wr_en_w = 0;
			wr_en_x = 0;
		end
		
		
		
		//Here we determine if we are accepting any data, this should only happen if we are entering or staying
		// in the read state.
		if ((state == READ))begin
			input_ready = 1;
		end else begin
			input_ready = 0;
		end
		
		//FOR EN_ACC: We enable this when we want to accumilate, which is only when we enter or reenter the process state.
		// The F register in MAC should be cleared by this point if we are entering it for the first time.
		if (state == PROCESS && process_initialize == 1 && process_cnt <= N && process_stall == 0) begin
			en_acc = 1;
		end else begin
			en_acc = 0;
		end
		
		//CLEAR_ACC: When we're transitioning from state DONE to PROCESS or READ to PROCESS, we want to clear the F reg in MAC.
		// if we do it any other time, then the output of Y will be errased before the source can read from it.	
		if ((state == PROCESS) && process_initialize == 0) begin
			clear_acc = 1;
		end else begin
			clear_acc = 0;
		end	 	
		
		// For each state, we initally reset the transition variable and assert them depending on the current state and the reqirements to move on to the next state.
		if((state == READ))begin	 
			//wr_en_w = 0;
			wr_en_x = 0;
			//We are clearing anyt transition indicators.
			read_done = 0;
			process_done = 0;
			next_row = 0;
			output_done = 0;
			output_valid = 0;
			if(input_valid) begin
				
				//if (read_initialize == 0 && new_matrix == 0) begin //For we are witing to the vector instead of the matrix first. 
				//	wr_en_w = 0;
				//	wr_en_x = 1;
				//end else begin 
					
					//if (input_cnt < M*N) begin //Here we are taking inputs for the matrix
//						wr_en_w = 1;
//						wr_en_x = 0;  
//					end else if (M*N <= input_cnt && input_cnt < (M*N+N)) begin	 //We're now writing to the vector
//						wr_en_w = 0;
//						wr_en_x = 1;  
//					end else begin //We're not writing to anything
//						wr_en_w = 0;
//						wr_en_x = 0;
//					end	  
					if (input_cnt < (N)) begin	 //We're now writing to the vector
						//wr_en_w = 0;
						wr_en_x = 1;  
					end else begin //We're not writing to anything
						//wr_en_w = 0;
						wr_en_x = 0;
					end
					//We move on to the next state when we are done reading all the values required.
					//We know this has happened when we have a value of input_cnt that represents a number indicating that all
					// elements of matrix w and vector x have been witten to.
					if ( input_cnt >= (N-1) ) begin //We're exitting the state.
						read_done = 1;
					end	else begin
						read_done = 0;
					end
			
				//end	
				
			end
			
			
		end else if ((state == PROCESS)) begin
			//We are clearing anyt transition indicators.
			read_done = 0;
			process_done = 0;
			next_row = 0; 
			output_done = 0;
		    output_valid = 0;
		    
			
			// Done with the row, ready to output element of the vector.
			// We know this because process_cnt shows that we have done this for all elements in a row or the vector.
		    if(process_cnt == N +1  ) begin  
		        process_done = 1;
				output_valid = 0;
		    end else begin
			output_valid = 0;
			process_done = 0;
			end
			
		end else if ((state == DONE)) begin
			//We are clearing any transition indicators.
			read_done = 0;
			process_done = 0;
			next_row = 0;
			output_done = 0;
			output_valid = 1;
			if(output_ready == 1) begin	
				//Output is being read.
				if( y_cnt == M-1 ) begin //We've already done y = 0 to 3.
					output_done = 1;
					next_row = 0;
				end else begin
					if(iteration_out >= P-1) begin
						 output_done = 0;
						next_row = 1;
					end else begin
						output_done = 0;
						next_row = 0;
					end 
				
				end	 
			end
		//This remaining else statement is present to clear any infered latches that may be present. This is not required.	
		end else begin 
			error = 1;
			read_done = 0;
			process_done = 0;
			next_row = 0;
			output_done = 0;
			output_valid = 0;
		end
	end	 
	
endmodule
