NEURAL NETWORK GENERATION TOOL - GENERATOR MODULE DOCUMENTATION
================================================================

OVERVIEW
--------
The generator module is a Python wrapper around the C++ network generation tool.
It handles calling the C++ executable, managing output files, creating ZIPs, and 
integrating with the Django web application.

STRUCTURE
---------
generator/
├── __init__.py              # Package initialization, exports public API
├── generator.py             # Main generator module with all functions
├── main.exe                 # Copied C++ executable from /src/
├── test_generator.py        # Unit tests for module setup
├── test_manual.py           # Integration test with mock NetworkConfig
├── outputs/                 # Generated files directory (created at runtime)
│   ├── network_*_*.zip      # Generated ZIP files
│   └── nn_*/                # Generated network directories
└── const_*.txt              # Generated constants files (temporary)


KEY COMPONENTS
--------------

1. generator.py
   Main module containing:
   
   - get_generator_base_path()
     Returns the absolute path to the generator folder
     Used for locating the executable and output directories
   
   - get_executable_path(executable_name='main')
     Returns full path to C++ executable
     Auto-detects .exe extension on Windows
     Raises GenerationError if executable not found
   
   - generate_network(network_config, output_dir=None)
     Main function that orchestrates the generation process
     Takes a NetworkConfig model instance
     Returns path to generated ZIP file
     
     Handles three modes:
     - Mode 1/2: Single layer network (requires M, N, T, R, constants)
     - Mode 3: Three-layer network (requires N, M1, M2, M3, T, R, B, constants)
     
     Process:
     1. Creates output directory
     2. Generates temporary constants file with dummy values
     3. Constructs command with proper arguments for C++ tool
     4. Executes main.exe with timeout
     5. Zips generated files
     6. Returns ZIP path
   
   - create_output_zip(network_config, output_dir)
     Creates a ZIP file from all generated outputs
     Excludes other ZIP files to avoid nested ZIPs
     Names ZIP as: network_{id}_{mode}_{input_size}.zip
   
   - cleanup_outputs(output_dir=None, keep_zips=True)
     Optional cleanup function
     Can preserve just ZIP files or remove everything

2. __init__.py
   - Makes generator a Python package
   - Exports public API for clean imports
   - Allows: from generator import generate_network
   - Rather than: from generator.generator import generate_network


COMMAND-LINE ARGUMENTS
----------------------

Mode 1/2 (Single Layer):
  main.exe MODE M N T R CONST_FILE
  
  MODE:       1 or 2
  M:          Output dimension (number of neurons)
  N:          Input dimension
  T:          Bit width (e.g., 16 for 16-bit)
  R:          ReLU activation (0 or 1)
  CONST_FILE: Path to constants/weights file

  Example: main.exe 1 10 10 16 0 const_file.txt

Mode 3 (Three-Layer Network):
  main.exe MODE N M1 M2 M3 T R B CONST_FILE
  
  MODE:       3
  N:          Input dimension
  M1, M2, M3: Layer dimensions
  T:          Bit width
  R:          ReLU activation
  B:          Batch size
  CONST_FILE: Path to constants file

  Example: main.exe 3 10 10 10 1 16 0 1 const_file.txt


OUTPUT FILES
------------
After generation, the tool creates:
- nn_{params}/ folder with generated Verilog files
- network_{id}_{mode}_{input_size}.zip containing all outputs

The ZIP file path is stored in NetworkConfig.generated_files_path


TESTING
-------

Unit Tests (test_generator.py):
  python generator/test_generator.py
  
  Tests:
  - Generator base path exists
  - main.exe executable is found
  - outputs/ directory can be created
  - Executable is callable
  
  Expected Result: 3/4 passing (executable timeout is expected without args)

Integration Test (test_manual.py):
  python generator/test_manual.py
  
  Tests:
  - Creates mock NetworkConfig
  - Calls generate_network()
  - Verifies ZIP file is created
  - Shows file size
  
  Expected Result: ✓ Generation successful


INTEGRATION WITH DJANGO
-----------------------

In views.py:
  from generator import generate_network, GenerationError
  
  try:
      zip_path = generate_network(network_config)
      network_config.generated_files_path = zip_path
      network_config.save()
  except GenerationError as e:
      # Handle error
      pass

This will be called:
1. On network creation (CreateView)
2. On demand from detail view (button click)


CURRENT LIMITATIONS & NOTES
---------------------------

1. Constants File
   Currently generates dummy constants (0-255 repeating)
   In production, should accept actual weight values from user

2. Mode Parameters
   For modes 1/2, M is derived from input_size (not ideal)
   Consider adding explicit M parameter to NetworkConfig

3. Error Handling
   Catches subprocess errors but doesn't log details
   Consider adding logging for debugging

4. Cleanup
   Generated directories are kept in outputs/ folder
   Use cleanup_outputs() periodically to free space

5. Timeout
   Set to 300 seconds (5 minutes)
   Adjust based on typical generation time


DOCKER COMPATIBILITY
--------------------

The generator folder is Docker-ready:
- main.exe is bundled with code
- Paths are relative and portable
- No external dependencies
- Just copy /backend/ folder into container

In Dockerfile:
  COPY backend/ /app/backend/
  
Generator will work automatically!


FUTURE IMPROVEMENTS
-------------------

1. Parallel generation (for multiple requests)
2. Progress tracking (for long-running generations)
3. Weight input validation
4. Logging and metrics
5. Cache generated networks by config
6. Background tasks (Celery) for large generations
7. Different executable versions per mode
8. User-provided constants files


VERSION HISTORY
---------------

Version 1.0 (Current):
- Working generator module
- Support for modes 1, 2, 3
- ZIP file creation
- Test scripts
- Docker-ready

Created: December 21, 2025
