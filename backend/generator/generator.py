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
            # Mode 3: N M1 M2 M3 T R B
            # We'll use input_size for N and set other dimensions
            N = network_config.input_size
            M1 = network_config.input_size
            M2 = network_config.input_size
            M3 = 1
            B = 1
            
            # Create a dummy constants file
            total_constants = N*M1 + M1*M2 + M2*M3
            const_file_path = output_dir / f"const_{network_config.id}.txt"
            with open(const_file_path, 'w') as f:
                for i in range(total_constants):
                    f.write(f"{i % 256}\n")
            
            cmd = [
                str(exe_path),
                str(network_config.mode),
                str(N),
                str(M1),
                str(M2),
                str(M3),
                str(network_config.T),
                str(int(network_config.R)),
                str(B),
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
    Create a ZIP file from generated outputs
    
    Args:
        network_config: NetworkConfig model instance
        output_dir: Directory containing generated files
    
    Returns:
        Path to the created ZIP file
    """
    output_dir = Path(output_dir)
    
    # Create zip filename based on network config
    zip_filename = f"network_{network_config.id}_{network_config.mode}_{network_config.input_size}.zip"
    zip_path = output_dir / zip_filename
    
    # Create ZIP file
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for file in output_dir.glob('*'):
            if file.is_file() and not file.name.endswith('.zip'):
                zipf.write(file, arcname=file.name)
    
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
