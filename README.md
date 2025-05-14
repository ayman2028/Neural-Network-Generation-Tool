# Neural-Network-Generation-Tool

A C++ program that generates SystemVerilog code for implementing neural network hardware.

## Features

- Generate SystemVerilog code for neural network hardware implementations
- Three operation modes:
  - Mode 1: Generate a single unparallelized neural network layer
  - Mode 2: Generate a single parallelized neural network layer
  - Mode 3: Generate a three-layer neural network with optimized parallelism
- Interactive mode for entering parameters directly through console prompts
- Default weight generation (all 1's) instead of requiring manual input
- Organized output directory structure for each neural network configuration
- Automatic generation of supporting SystemVerilog files (controller.sv, memory.sv, datapath modules)

## Usage

### Interactive Mode
```
./neural_net_gen
```
or
```
./neural_net_gen interactive
```

### Command Line Mode
```
# Mode 1: Single unparallelized layer
./neural_net_gen 1 M N T R const_file
./neural_net_gen 1 M N T R console    # Use default weights

# Mode 2: Single parallelized layer
./neural_net_gen 2 M N T R P const_file
./neural_net_gen 2 M N T R P console  # Use default weights

# Mode 3: Three-layer network
./neural_net_gen 3 N M1 M2 M3 T R B const_file
./neural_net_gen 3 N M1 M2 M3 T R B console  # Use default weights
```

Where:
- M, M1, M2, M3: Output dimensions
- N: Input dimension
- T: Bit width
- R: ReLU activation (1 = yes, 0 = no)
- P: Parallelism factor (must be a factor of M)
- B: Multiplier budget for the three-layer network
- const_file: File containing weight values
- console: Use default weight values (all 1's)

## Output

The program creates a directory structure with all necessary SystemVerilog files:
- Main neural network module (.sv)
- controller.sv
- memory.sv
- datapath_gen_p3.sv (or datapath_gen_p3_relu.sv for ReLU activation)

## Building

```
cd src
g++ main.cc -o neural_net_gen
```