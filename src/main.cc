// ESE 507 Project 3 Handout Code
// Fall 2020
// You may not redistribute this code

// Getting started:
// The main() function contains the code to read the parameters. 
// For Parts 1 and 2, your code should be in the genFCLayer() function. Please
// also look at this function to see an example for how to create the ROMs.
//
// For Part 3, your code should be in the genNetwork() function.

// Classes used for optimization
#include <stack>
#include <vector> 
#include <string.h>

// Classes given by handout code
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <cstdlib>
#include <cstring>
#include <assert.h>
#include <math.h>
using namespace std;

void printUsage();
// Note: genFCLayer takes L parameter to specify if in network and layer number
void genFCLayer(int M, int N, int T, int R, int P, int L, vector<int>& constvector, string modName, ofstream &os);
// Note: Changed B to B_local to be more explicit
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B_local, vector<int>& constVector, string modName, ofstream &os);
void readConstants(ifstream &constStream, vector<int>& constvector);
// Note: genRom takes was changed to take more M,N,P,L parameters
void genROM(vector<int>& constVector, int bits, ofstream &os, int M, int N, int P, int L);

// Global multiplier budget accessed by every layer class
int B;

// ------------------------ Classes and functions for optimization ------------------------
class layer {
public:
	stack <int> factors;
	int layer_number;
	int M, N;
	int P=1;
	int C;
	layer(int l, int n, int m) {
		cout << " Making layer " << l << endl;
		layer_number = l;
		M = m;
		N = n;
		get_factors();
		factors.pop();
		find_C();
		B--;
	}
	layer() {
		cout << "Extra layer made" << endl;
	}

	//Places factors of M in stack.
	void get_factors() {
		int temp = 1;
		stack <int> tempStk;
		while (temp <= M)
		{
			if (not(M % temp))
				tempStk.push(temp);

			temp++;
		}
		while (!tempStk.empty()) {
			factors.push(tempStk.top());
			tempStk.pop();
		}
	}
	int find_C() {
		C = (1 + 2 * N) * (M / P) + M;
		return C;
	}
	//If the layer were to use its next value in factors, what would be the improvement in C.
	int find_C_change() {
		int tempP = 0;
		int tempC = 0;
		if (!factors.empty()) {
			tempP = factors.top();
			factors.pop();
			tempC = (1 + 2 * N) * (M / tempP) + M;
			return (C - tempC);
		}
		else
			return 0;

	}
	bool change_P() {
		if (!factors.empty()) {
			int temp = factors.top() - P;
			P = factors.top();
			factors.pop();
			find_C();
			B = B - temp;
			return true;
		}
		else
			return false;
	}


};

void print_layer_status(layer* l1) {
	cout << "Layer " << l1->layer_number << ": P = " << l1->P << ", (N,M) = (" << l1->N << ", " << l1->M << ").  C = " << l1->C << endl;

}

class network {
public:
	vector<layer*> net;
	layer* bottleneck;
	int delta;
	network(layer* l1, layer* l2, layer* l3) {
		net.push_back(l1);
		net.push_back(l2);
		net.push_back(l3);
		find_bottleneck();
		find_delta();	
	}
	int find_bottleneck() {
		layer* lPtr;
		int max_index = 0;
		int max_c = 0;
		for (vector<layer*>::iterator it = net.begin(); it != net.end(); ++it) {
			lPtr = *it;
			if (max_c <= lPtr->find_C()) {
				max_index = lPtr->layer_number;
				max_c = lPtr->C;
				bottleneck = *it;
			}

		}
		//cout << "Bottleneck is at layer" << max_index << " with C = " << max_c << endl;
		return max_index;
	}
	void print_layers() {
		layer* l1 = net.at(0);
		layer* l2 = net.at(1);
		layer* l3 = net.at(2);
		cout << "Layer 1: P = " << l1->P << ", (N,M1) = (" << l1->N << ", " << l1->M << ").  C = " << l1->C << endl;
		cout << "Layer 2: P = " << l2->P << ", (M1,M2) = (" << l2->N << ", " << l2->M << ").  C = " << l2->C << endl;
		cout << "Layer 2: P = " << l3->P << ", (M2,M3) = (" << l3->N << ", " << l3->M << ").  C = " << l3->C << endl;
		cout << " NEW B = " << B << endl;
	}
	int find_delta() {
		layer* lPtr;
		int max_index_2 = 0;
		int max_c_2 = 0;
		for (vector<layer*>::iterator it = net.begin(); it != net.end(); ++it) {
			lPtr = *it;
			if (max_c_2 <= lPtr->find_C() && lPtr->layer_number != bottleneck->layer_number) {
				max_index_2 = lPtr->layer_number;
				max_c_2 = lPtr->C;

			}

		}
		delta = bottleneck->C - net.at(max_index_2-1)->C;
		return delta;
	}
	void optimize_ps() {
		find_bottleneck();
		find_delta();
		cout << "New bottle neck L = " << bottleneck->layer_number << " with remaining B = " << B;
		if (!bottleneck->factors.empty()) {
			cout <<", Current P = "<< bottleneck->P <<" & next P = " << bottleneck->factors.top() << endl;
		}
		else {
			cout << endl << "This layer has reached it's max value of P, no further optimization will improve throughpout." << endl;
		}
		//cout << "We need to optimize L" << bottleneck->layer_number << ", new P would be " << bottleneck->factors.top() << " the improvement would be " << bottleneck->find_C_change();
		if (!bottleneck->factors.empty() && B >= (bottleneck->factors.top() - bottleneck->P)) {
			bottleneck->change_P();
			cout << "Optimized " << bottleneck->layer_number << endl;
			print_layers();
			optimize_ps();
		}
		
	}
};

void print_Cs(layer* l1, layer* l2, layer* l3) {
	cout << "Layer 1: P = " << l1->P << ", (N,M1) = (" << l1->N << ", " << l1->M <<    ").  C = " << l1->C << endl;
	cout << "Layer 2: P = " << l2->P << ", (M1,M2) = (" << l2->N << ", " << l2->M << ").  C = " << l2->C << endl;
	cout << "Layer 2: P = " << l2->P << ", (M2,M3) = (" << l3->N << ", " << l3->M << ").  C = " << l3->C << endl;
}

// ------------------------ End of Classes and functions for optimization ------------------------

// Main function
int main(int argc, char* argv[]) {

   // If the user runs the program without enough parameters, print a helpful message
   // and quit.
   if (argc < 7) {
      printUsage();
      return 1;
   }

   int mode = atoi(argv[1]);

   ifstream const_file;
   ofstream os;
   vector<int> constVector;

   //----------------------------------------------------------------------
   // Look here for Part 1 and 2
   if (((mode == 1) && (argc == 7)) || ((mode == 2) && (argc == 8))) {

      // Mode 1/2: Generate one layer with given dimensions and one testbench

      // --------------- read parameters, etc. ---------------
      int M = atoi(argv[2]);
      int N = atoi(argv[3]);
      int T = atoi(argv[4]);
      int R = atoi(argv[5]);

      int P;

      if (mode == 1) {
         P=1;
         const_file.open(argv[6]);         
      }
      else {
         P = atoi(argv[6]);
         const_file.open(argv[7]);
      }

      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[6] << endl;
         return 1;
      }

      // Read the constants out of the provided file and place them in the constVector vector
      readConstants(const_file, constVector);

      string out_file = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P) + ".sv";

      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      // call the genFCLayer function you will write to generate this layer
      string modName = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P);

      // 0 denotes this layer is not part of a network
      genFCLayer(M, N, T, R, P, 0, constVector, modName, os); 

   }
   //--------------------------------------------------------------------


   // ----------------------------------------------------------------
   // Look here for Part 3
   else if ((mode == 3) && (argc == 10)) {
      // Mode 3: Generate three layers interconnected

      // --------------- read parameters, etc. ---------------
      int N  = atoi(argv[2]);
      int M1 = atoi(argv[3]);
      int M2 = atoi(argv[4]);
      int M3 = atoi(argv[5]);
      int T  = atoi(argv[6]);
      int R  = atoi(argv[7]);
      int B  = atoi(argv[8]);

      const_file.open(argv[9]);
      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[8] << endl;
         return 1;
      }
      readConstants(const_file, constVector);

      // Create name for part1 and 2 part2
      string out_file = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B)+ ".sv";


      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      // Create module name for part3
      string mod_name = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B);

      // generate the design
      genNetwork(N, M1, M2, M3, T, R, B, constVector, mod_name, os);

   }
   //-------------------------------------------------------

   else {
      printUsage();
      return 1;
   }

   // close the output stream
   os.close();

}

// Read values from the constant file into the vector
void readConstants(ifstream &constStream, vector<int>& constvector) {
   string constLineString;
   while(getline(constStream, constLineString)) {
      int val = atoi(constLineString.c_str());
      constvector.push_back(val);
   }
}

// Generate a ROM based on values constVector.
// Values should each be "bits" number of bits.
void genROM(vector<int>& constVector, int bits, ofstream &os, int M, int N, int P, int L) {

      int numWords = constVector.size();
      int addrBits = ceil(log2(numWords));

      // Define module name based on parameter L
      if(L == 0)
        // TODO: Change to p not p1
        os << "module " << "rom_gem_p1" << "(clk, addr, z);" << endl;
      else if (L == 1 || L == 2| L == 3){
         os << "module " << "rom_gem_p" << L << "(clk, addr, z);" << endl;
      }

      // Parameters: We set up an arbitrary value since they are going to be set by anothe module
      os <<"\tparameter\t\t\t M = " << M << ";" << endl;
      os <<"\tparameter\t\t\t N = " << N << ";" << endl;
      os <<"\tparameter\t\t\t T = " << bits << ";" << endl;
      os <<"\tparameter\t\t\t P = " << P << ";" << endl;
      os <<"\tparameter\t\t\t I = " << 0 << ";" << endl;
      os <<"\tlocalparam\t\t\t MATRIX_SIZE = $clog2(M*N/P);\n" << endl;

      os << "\tinput clk;" << endl;
      os << "\tinput [MATRIX_SIZE-1:0] addr;" << endl;
      os << "\toutput logic signed [T-1:0] z;\n" << endl;

      // Generate the desired rom based on parameters
      os << "\tgenerate" << endl;
      os <<"\t\tlogic signed [T-1:0] values[(M*N/P)-1:0];" << endl;

     // Create ROM vector splits
      // Create a list of vectors 
      int start;
      int stop;
      int i, j, k;
      int location;
      vector<vector<int>> romList;

      for(i = 0; i < P; i++){
        // Create a new rom:
        vector<int> rom;

        // Number of rows per rom
        for(j = 0; j < M/P; j++){

          // Number of elements per rom
          for (k = 0; k < N;  k++){
            location = N*i+k + (N + (N * (P-1)))*j;
            rom.push_back(constVector.at(location));
          }
        }

        // Append row to ROM list
        romList.push_back(rom);
      }

      // Generate ROM code
      for(i = 0; i < P;  i++){
        j = 0;
        if(i == 0){
          os <<"\t\tif (I == 0) begin" << endl;
          for(vector<int>::iterator it = romList.at(i).begin(); it < romList.at(i).end(); it++, j++){
            if (*it < 0)
              os<<"\t\t\tassign values[" << j << "] = " << "-" << bits <<"'d" << abs(*it) <<";" << endl;
            else
              os<<"\t\t\tassign values[" << j << "] = " << bits <<"'d" << *it <<";" << endl;
          }
          os << "\t\tend" << endl;
        }
        else{
          os <<"\t\telse if (I == " << i << ") begin" << endl;
          for(vector<int>::iterator it = romList.at(i).begin(); it < romList.at(i).end(); it++, j++){
            if (*it < 0)
              os<<"\t\t\tassign values[" << j << "] = " << "-" << bits <<"'d" << abs(*it) <<";" << endl;
            else
              os<<"\t\t\tassign values[" << j << "] = " << bits <<"'d" << *it <<";" << endl;
          }
          os << "\t\tend" << endl;
        }
      }

      os <<"\tendgenerate" << endl;

      // ROM logic
      os <<"\n\talways_ff @(posedge clk) begin"<< endl;
      os <<"\t\tz <= values[addr];" << endl;
      os << "\tend" << endl;

      // End module
      os << "endmodule" << endl;
}

// Parts 1 and 2
// Here is where you add your code to produce a neural network layer.
void genFCLayer(int M, int N, int T, int R, int P, int L, vector<int>& constVector, string modName, ofstream &os) {


   // Everything can be generated with HDL files of part 3, since they are generalized
   if (L == 0){
      cout << "Generating 1 layer for test" << endl;
      os << "`include \"memory.sv\"" << endl;
      os << "`include \"controller.sv\"" << endl;

      if (R == 1)
         os << "`include \"datapath_gen_p3_relu.sv\"" << endl;
      else
         os << "`include \"datapath_gen_p3.sv\"" << endl;
   }
   
  
  // Module declaration
   os << "\nmodule " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;

  // Parameters
   os << "\tparameter M = " << M <<";" << endl;
   os << "\tparameter N = " << N <<";" << endl;
   os << "\tparameter T = " << T <<";" << endl;
   os << "\tparameter P = " << P <<";" << endl;

   // Conditional Parameter L
   if(L == 1 || L == 2 || L == 3)
    os << "\tparameter L = " << L <<";" << endl;
   else if(L == 0)
    os << "\n\t// Ignoring L parameter, layer intended to be used alone" << endl;
   else{
    cout << "ERROR: L value has to be either 0 (for single layers) or (1,2,3) for Networks" << endl;
    assert(false);
   }


   // Port declarations
   os << "\n\t// Top level ports" << endl;
   os << "\tinput                 clk, reset, input_valid, output_ready;" << endl;
   os << "\tinput signed [T-1:0]  input_data;" << endl;
   os << "\toutput signed [T-1:0] output_data;" << endl;
   os << "\toutput logic          output_valid, input_ready;" << endl;

  // Internal signals (connect control with datapath)
  os << "\n\t// Internal signals (connect control with datapath)" << endl;
  os << "\tlocalparam                VECTOR_SIZE=$clog2(N);" << endl;
  os << "\tlocalparam               MATRIX_SIZE=$clog2(M*N/P);" << endl;
  os << "\tlogic [VECTOR_SIZE-1:0]  addr_x;" << endl;
  os << "\tlogic [MATRIX_SIZE-1:0]  addr_w;" << endl;
  os << "\tlogic [P-1:0] f_sel;" << endl;
  os << "\tlogic                    wr_en_x, wr_en_w, clear_acc, en_acc;" << endl;

  // Instantiate datapath and control
  os <<"\n\t//Instantiate Control" << endl;
  os <<"\tcontrol_gen #(M,N,T,P)control1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .output_ready(output_ready), .output_valid(output_valid), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .wr_en_w(wr_en_w), .clear_acc(clear_acc), .en_acc(en_acc), .f_sel(f_sel));\n" << endl;

  os <<"\t//Instantiate Datapath" << endl;

   // Datapath generation for single and multiple layers
   if(L == 0){
      if (R == 1)
         os << "\tdatapath_gen_p3_relu #(M,N,T,P)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));" << endl;
      else
         os << "\tdatapath_gen_p3 #(M,N,T,P)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));" << endl;
   } 
   else if (L == 1 || L == 2 || L == 3){
      if (R == 1)
         os << "\tdatapath_gen_p3_relu #(M,N,T,P,L)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));" << endl;
      else 
         os << "\tdatapath_gen_p3 #(M,N,T,P,L)datapath1(.clk(clk), .reset(reset), .input_data(input_data), .addr_x(addr_x), .wr_en_x(wr_en_x), .addr_w(addr_w), .clear_acc(clear_acc), .en_acc(en_acc), .m_data_out_y(output_data), .f_sel(f_sel));" << endl;
   }
   os << "endmodule" << endl << endl;



   // At some point you will want to generate a ROM with values from the pre-stored constant values.
   // Here is code that demonstrates how to do this for the simple case where you want to put all of
   // the matrix values W in one ROM. This is probably what you will need for P=1, but you will want 
   // to change this for P>1. Please also see some examples of splitting these vectors in the Part 3
   // code.

  // Now, we generate the rom generator

   // Check there are enough values in the constant file.
   if (M*N != constVector.size()) {
      cout << "ERROR: constVector does not contain correct amount of data for the requested design" << endl;
      cout << "The design parameters requested require " << M*N << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }

   if (P > M){
      cout << "ERROR: P > M (" << P << " > " << M <<")" << endl;
      assert(false);
   }

   if (M % P != 0){
      cout << "ERROR: M/P should be an integer " << M <<"/" << P  << " != " << "integer" << endl;
      assert(false);
   }

   // Generate ROM
   // The L indicates if this rom is intended for use in a networked layer or not
   genROM(constVector, T, os, M, N, P, L);

}

// Part 3: Generate a hardware system with three layers interconnected.
// Layer 1: Input length: N, output length: M1
// Layer 2: Input length: M1, output length: M2
// Layer 3: Input length: M2, output length: M3
// B is the number of multipliers your overall design may use.
// Your goal is to build the fastest design that uses B or fewer multipliers
// constVector holds all the constants for your system (all three layers, in order)
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B_local, vector<int>& constVector, string modName, ofstream &os) {

   // ---------------------------------- Optimization ----------------------------------------
   // Here you will write code to figure out the best values to use for P1, P2, and P3, given
   // B. 

   // Updating global budget
	B = B_local; 

	layer layer1(1, N, M1);
	layer layer2(2, M1, M2);
	layer layer3(3, M2, M3);


	print_Cs(&layer1, &layer2, &layer3);

	network Network(&layer1, &layer2, &layer3);
	Network.optimize_ps();

	int P1 = Network.net.at(0)->P;
	int P2 = Network.net.at(1)->P;
	int P3 = Network.net.at(2)->P;

   cout << "Optimized P values: P1 =" << P1 << " P2 = " << P2 <<" P3 = " << P3 << endl;

   // ---------------------------------------- End of Optimization ---------------------------

   // Includes:
    os << "`include \"memory.sv\"" << endl;
    os << "`include \"controller.sv\"" << endl;

    if(R == 1){
         os << "`include \"datapath_gen_p3_relu.sv\"" << endl; // also contains genSatMac_p3 and sat_mac_p3
         os << endl;
    }
    else {
        os << "`include \"datapath_gen_p3.sv\"" << endl; // also contains genSatMac_p3 and sat_mac_p3
        os << endl;
    }
   // Output top-level module
   os << "\nmodule " << modName << "(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" << endl;

   // parameters
   os << "\tparameter M1 = " << M1 <<";" << endl;
   os << "\tparameter M2 = " << M2 <<";" << endl;
   os << "\tparameter M3 = " << M3 <<";" << endl;
   os << "\tparameter N = " << N <<";" << endl;
   os << "\tparameter T = " << T <<";" << endl;
   os << "\tparameter P1 = " << P1 <<";" << endl;
   os << "\tparameter P2 = " << P2 <<";" << endl;
   os << "\tparameter P3 = " << P3 <<";" << endl;

   // Port declarations
   os << "\n\t// Top level ports" << endl;
   os << "\tinput                 clk, reset, input_valid, output_ready;" << endl;
   os << "\tinput signed [T-1:0]  input_data;" << endl;
   os << "\toutput signed [T-1:0] output_data;" << endl;
   os << "\toutput logic          output_valid, input_ready;" << endl;

   os << "\n\t// Internal signals (connect the three layers)" << endl;
   os <<"\tlogic [T-1:0] L1toL2_output_data,L2toL3_output_data;" << endl;
   os <<"\tlogic L1toL2_output_valid, L2toL3_output_valid, L1toL2_output_ready,L2toL3_output_ready;" << endl;
  
   // generate the three layer modules on the same file, this names are needed on the top level
   string subModName1 = "l1_fc_" + to_string(M1) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P1);

   string subModName2 = "l2_fc_" + to_string(M2) + "_" + to_string(M1) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P2);

   string subModName3 = "l3_fc3_" + to_string(M3) + "_" + to_string(M2) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P3);

   // Instantiate the three layers
   os << "\t// Instantiate the three layers" << endl;
   os << "\n\t" << subModName1 << " #(M1, N, T, P1, 1)layer_1(.clk(clk), .reset(reset), .input_valid(input_valid), .input_ready(input_ready), .input_data(input_data), .output_valid(L1toL2_output_valid), .output_ready(L1toL2_output_ready),.output_data(L1toL2_output_data));\n" << endl;

   os << "\t" << subModName2 <<  " #(M2, M1, T, P2, 2)layer_2(.clk(clk), .reset(reset), .input_valid(L1toL2_output_valid), .input_ready(L1toL2_output_ready), .input_data(L1toL2_output_data), .output_valid(L2toL3_output_valid), .output_ready(L2toL3_output_ready), .output_data(L2toL3_output_data));\n" << endl;

   os << "\t" << subModName3 <<  " #(M3, M2, T, P3, 3)layer_3(.clk(clk), .reset(reset), .input_valid(L2toL3_output_valid), .input_ready(L2toL3_output_ready), .input_data(L2toL3_output_data), .output_valid(output_valid), .output_ready(output_ready), .output_data(output_data));\n" << endl;

   os << "\nendmodule" << endl;
   
   // -------------------------------------------------------------------------
   // Split up constVector for the three layers
   // layer 1's W matrix is M1 x N
   int start = 0;
   int stop = M1*N;
   vector<int> constVector1(&constVector[start], &constVector[stop]);

   // layer 2's W matrix is M2 x M1
   start = stop;
   stop = start+M2*M1;
   vector<int> constVector2(&constVector[start], &constVector[stop]);

   // layer 3's W matrix is M3 x M2
   start = stop;
   stop = start+M3*M2;
   vector<int> constVector3(&constVector[start], &constVector[stop]);

   if (stop > constVector.size()) {
      os << "ERROR: constVector does not contain enough data for the requested design" << endl;
      os << "The design parameters requested require " << stop << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }
   // --------------------------------------------------------------------------

   // Generate the three layers (number denotes L, the position in the layer)
   genFCLayer(M1, N, T, R, P1, 1, constVector1, subModName1, os);
   genFCLayer(M2, M1, T, R, P2, 2, constVector2, subModName2, os);
   genFCLayer(M3, M2, T, R, P3, 3, constVector3, subModName3, os);

}


void printUsage() {
  cout << "Usage: ./gen MODE ARGS" << endl << endl;

  cout << "   Mode 1 (Part 1): Produce one neural network layer (unparallelized)" << endl;
  cout << "      ./gen 1 M N T R const_file" << endl << endl;

  cout << "   Mode 2 (Part 2): Produce one neural network layer (parallelized)" << endl;
  cout << "      ./gen 2 M N T R P const_file" << endl << endl;

  cout << "   Mode 3 (Part 3): Produce a system with three interconnected layers" << endl;
  cout << "      ./gen 3 N M1 M2 M3 T R B const_file" << endl;
}