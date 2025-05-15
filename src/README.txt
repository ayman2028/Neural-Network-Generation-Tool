Neural Network Generation Tool
==========================

This tool generates SystemVerilog code for neural network hardware implementations.

Every generated module (either from mode 1, 2, or 3) will include the following helper files:
- controller.sv - State machine to control the neural network operation
- memory.sv - Memory module for storing input vectors
- datapath_gen_p3.sv (if non-ReLU) or datapath_gen_p3_relu.sv (if ReLU) - Datapath for computation

Key features:
1. Interactive mode for parameter input via console
2. Default weight generation (all 1's) instead of requiring manual input files
3. Organized directory structure for output files
4. Automatic generation of supporting SystemVerilog files

The program supports three modes:
- Mode 1: Generate a single unparallelized neural network layer
- Mode 2: Generate a single parallelized neural network layer
- Mode 3: Generate a three-layer neural network with optimized parallelism

For Mode 3, the program optimizes the parallelism factors (P1, P2, P3) based on the given
multiplier budget (B) to minimize the critical path delay.

Testmode files stayed the same.

The helper files have been included in the src and hdl folders