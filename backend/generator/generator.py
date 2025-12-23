"""
Generator module for Neural Network Generation Tool
Handles calling C++ generation tools and managing output files
"""

import os
import subprocess
import shutil
import zipfile
from pathlib import Path


class GenerationError(Exception):
    """Custom exception for generation errors"""
    pass


def get_generator_base_path():
    """Get the base path for the generator (this module's directory)"""
    return Path(__file__).parent.absolute()


def get_executable_path(executable_name='main'):
    """
    Get the full path to the C++ executable
    
    Args:
        executable_name: Name of the executable (default: 'main')
    
    Returns:
        Path object pointing to the executable
    
    Raises:
        GenerationError: If executable not found
    """
    # Try with and without .exe extension
    exe_path = get_generator_base_path() / executable_name
    
    if not exe_path.exists() and not executable_name.endswith('.exe'):
        # Try with .exe extension
        exe_path = get_generator_base_path() / (executable_name + '.exe')
    
    if not exe_path.exists():
        raise GenerationError(f"Executable not found: {exe_path}")
    
    return exe_path


def generate_network(network_config, output_dir=None):
    """
    Generate network files using the C++ tool
    
    Args:
        network_config: NetworkConfig model instance
        output_dir: Directory to store generated files (optional)
    
    Returns:
        Path to generated ZIP file
    
    Raises:
        GenerationError: If generation fails
    """
    if output_dir is None:
        output_dir = get_generator_base_path() / 'outputs'
    
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Build command based on mode
    try:
        exe_path = get_executable_path('main')
        
        # Command construction based on mode
        # Modes 1/2 require: mode M N T R const_file
        # Mode 3 requires: mode N M1 M2 M3 T R B const_file
        
        if network_config.mode in [1, 2]:
            # For mode 1/2, we need M and N values
            # Since NetworkConfig doesn't have M separately for modes 1/2,
            # we use input_size as N and create a dummy M
            M = network_config.input_size  # Using input_size as M
            N = network_config.input_size  # Using input_size as N
            
            # Create a dummy constants file
            const_file_path = output_dir / f"const_{network_config.id}.txt"
            with open(const_file_path, 'w') as f:
                # Generate dummy constants (M*N values)
                for i in range(M * N):
                    f.write(f"{i % 256}\n")
            
            cmd = [
                str(exe_path),
                str(network_config.mode),
                str(M),
                str(N),
                str(network_config.T),
                str(int(network_config.R)),
                str(const_file_path)
            ]
        
        elif network_config.mode == 3:
            # Mode 3: Three-layer network
            # C++ command format: mode N M1 M2 M3 T R B const_file
            # Where B is the multiplier budget (optimization parameter)
            
            # Use input_size as N (input dimension to first layer)
            N = network_config.input_size
            # For now, use input_size for all layer dimensions (can be extended later)
            M1 = network_config.input_size  # Output size of layer 1
            M2 = network_config.input_size  # Output size of layer 2
            M3 = 1                          # Output size of layer 3 (final output)
            
            # Get B (multiplier budget) from model, or use default of 10
            # B controls optimization: higher B = more parallelism across layers
            # Default of 10 works well for 3-layer networks
            B = network_config.B if network_config.B else 10
            
            # Create a dummy constants file (weights for the network)
            # In production, these would come from trained weights
            total_constants = N*M1 + M1*M2 + M2*M3
            const_file_path = output_dir / f"const_{network_config.id}.txt"
            with open(const_file_path, 'w') as f:
                for i in range(total_constants):
                    f.write(f"{i % 256}\n")
            
            # Construct C++ command with B parameter passed to C++ generator
            cmd = [
                str(exe_path),
                str(network_config.mode),
                str(N),
                str(M1),
                str(M2),
                str(M3),
                str(network_config.T),
                str(int(network_config.R)),
                str(B),  # Pass user-specified or default multiplier budget
                str(const_file_path)
            ]
        
        else:
            raise GenerationError(f"Unsupported mode: {network_config.mode}")
        
        # Run the executable
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300,  # 5 minute timeout
            cwd=str(output_dir)
        )
        
        if result.returncode != 0:
            raise GenerationError(f"Generation failed: {result.stderr}")
        
        # Create ZIP of generated files
        zip_path = create_output_zip(network_config, output_dir)
        
        return zip_path
    
    except subprocess.TimeoutExpired:
        raise GenerationError("Generation process timed out")
    except Exception as e:
        raise GenerationError(f"Generation error: {str(e)}")


def create_output_zip(network_config, output_dir):
    """
    Create a ZIP file from generated outputs for this specific network
    
    Args:
        network_config: NetworkConfig model instance
        output_dir: Directory containing generated files
    
    Returns:
        Path to the created ZIP file
        
    Raises:
        GenerationError: If required files (SV files) are not found
    """
    output_dir = Path(output_dir)
    
    # Create zip filename based on network config
    zip_filename = f"network_{network_config.id}_{network_config.mode}_{network_config.input_size}.zip"
    zip_path = output_dir / zip_filename
    
    # Build the expected subdirectory pattern for this network
    # The C++ generator creates directories like nn_N_M1_M2_M3_T_R or nn_M_N_T_R for modes 1/2
    # We need to find the subdirectory that was created for this network
    
    # Look for subdirectories in the output_dir
    subdirs = [d for d in output_dir.iterdir() if d.is_dir() and d.name.startswith('nn_')]
    
    # Verify that at least one subdirectory exists (contains SV files)
    sv_files = list(output_dir.rglob('*.sv'))
    
    if not sv_files:
        raise GenerationError(
            f"No SystemVerilog (.sv) files found in {output_dir}. "
            f"C++ generator may have failed to create output files. "
            f"Available files: {list(output_dir.glob('*'))}"
        )
    
    print(f"Found {len(sv_files)} SystemVerilog files to include in ZIP:")
    for sv_file in sv_files:
        print(f"  - {sv_file.relative_to(output_dir)}")
    
    # Create ZIP file with only this network's files
    files_added = 0
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        # Add files specific to this network:
        # 1. The SV files (in subdirectories)
        for file in output_dir.rglob('*.sv'):
            arcname = file.relative_to(output_dir)
            zipf.write(file, arcname=arcname)
            files_added += 1
            print(f"  Added to ZIP: {arcname}")
        
        # 2. The const file for this network (const_<id>.txt)
        const_file = output_dir / f"const_{network_config.id}.txt"
        if const_file.exists():
            zipf.write(const_file, arcname=const_file.name)
            files_added += 1
            print(f"  Added to ZIP: {const_file.name}")
        
        # 3. The cost.txt file (generic output from C++)
        cost_file = output_dir / "cost.txt"
        if cost_file.exists():
            zipf.write(cost_file, arcname=cost_file.name)
            files_added += 1
            print(f"  Added to ZIP: cost.txt")
    
    if files_added == 0:
        raise GenerationError(f"No files were added to ZIP {zip_path}")
    
    print(f"ZIP created successfully with {files_added} files: {zip_path}")
    
    # Verify ZIP contents
    with zipfile.ZipFile(zip_path, 'r') as zipf:
        zip_contents = zipf.namelist()
        sv_in_zip = [f for f in zip_contents if f.endswith('.sv')]
        if not sv_in_zip:
            raise GenerationError(
                f"ZIP file created but contains no .sv files! "
                f"ZIP contains: {zip_contents}"
            )
        print(f"Verified: ZIP contains {len(sv_in_zip)} .sv files")
    
    return str(zip_path)


def cleanup_outputs(output_dir=None, keep_zips=True):
    """
    Clean up generated files (optional)
    
    Args:
        output_dir: Directory to clean (default: generator outputs)
        keep_zips: If True, keep only ZIP files (default: True)
    """
    if output_dir is None:
        output_dir = get_generator_base_path() / 'outputs'
    
    output_dir = Path(output_dir)
    
    if not output_dir.exists():
        return
    
    if keep_zips:
        # Remove all non-ZIP files
        for file in output_dir.glob('*'):
            if file.is_file() and not file.name.endswith('.zip'):
                file.unlink()
    else:
        # Remove everything
        shutil.rmtree(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
