# Building the Generator - Platform-Specific Instructions

## Overview

The Neural Network Generator is a C++ tool that must be compiled before use. This document covers building for Windows, macOS, and Linux.

## Prerequisites

### Windows
- Visual Studio Build Tools **OR** MinGW with g++, make, and C++11 support
- Python 3.9+
- pip

### macOS
- Xcode Command Line Tools: `xcode-select --install`
- Python 3.9+
- pip (usually included with Python)

### Linux
- Build essentials: `sudo apt-get install build-essential g++ make`
- Python 3.9+
- pip

## Building Locally

### macOS & Linux

```bash
# From project root, run:
chmod +x build.sh
./build.sh
```

This will:
1. Compile the C++ code from `/src/`
2. Copy the executable to `/backend/generator/main`
3. Verify it works

### Windows

```cmd
# From project root, run:
build.bat
```

This will:
1. Compile the C++ code from `/src/`
2. Copy the executable to `/backend/generator/main.exe`
3. Display completion status

## Building for Docker

The Dockerfile automatically compiles the C++ tool for Linux when building the image:

```bash
docker-compose build
```

The build process:
1. Uses multi-stage Dockerfile
2. Compiles from `/src/` in Linux environment
3. Places executable at `/app/generator/main`
4. No local compilation needed

## Verifying the Build

After building locally, verify the executable exists:

**macOS/Linux:**
```bash
ls -lh backend/generator/main
```

**Windows:**
```cmd
dir backend\generator\main.exe
```

The executable should be executable (chmod +x on Unix) and reasonably small (~1-5 MB).

## Platform-Specific Notes

### macOS
- Requires Xcode Command Line Tools
- The compiled executable will be named `main` (no extension)
- Stored at `/backend/generator/main`
- Generator code auto-detects this path

### Windows
- Requires g++ (can use MinGW or Visual Studio tools)
- The compiled executable will be named `main.exe`
- Stored at `/backend/generator/main.exe`
- Generator code auto-detects this path

### Linux / Docker
- Uses standard GCC toolchain
- Dockerfile compiles automatically
- No pre-compiled executable needed in repo
- Executable placed at `/app/generator/main` in container

## Troubleshooting

### "Command not found: make"
**Solution:** Install build tools
- macOS: `xcode-select --install`
- Linux: `sudo apt-get install build-essential`
- Windows: Install MinGW with make

### "g++: command not found"
**Solution:** Install C++ compiler
- macOS: `xcode-select --install`
- Linux: `sudo apt-get install g++`
- Windows: Install MinGW or Visual Studio Build Tools

### Compilation errors
1. Ensure you're in the correct directory (`src/`)
2. Check that `main.cc` exists
3. Look at the Makefile for any special requirements
4. Check C++ standard (Makefile uses `-std=c++11`)

### Docker build fails
```bash
# Rebuild with verbose output
docker-compose build --no-cache --progress=plain
```

## CI/CD Integration

For automated builds, use:

**GitHub Actions example:**
```yaml
- name: Build Generator
  run: |
    if [ "$RUNNER_OS" == "macOS" ]; then
      chmod +x build.sh && ./build.sh
    elif [ "$RUNNER_OS" == "Linux" ]; then
      chmod +x build.sh && ./build.sh
    else
      build.bat
    fi
```

## Next Steps

1. Run the appropriate build script for your OS
2. Verify the executable was created
3. Test the Django application locally
4. Commit the compiled executable (Windows/macOS) or rebuild in Docker

