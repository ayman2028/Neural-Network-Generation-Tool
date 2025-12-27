#!/bin/bash
# Build script for Neural Network Generator - Works on macOS and Linux

set -e  # Exit on error

echo "Building Neural Network Generator..."

# Navigate to src directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"
BACKEND_GENERATOR_DIR="$SCRIPT_DIR/backend/generator"

if [ ! -d "$SRC_DIR" ]; then
    echo "Error: src directory not found at $SRC_DIR"
    exit 1
fi

echo "Compiling C++ generator from $SRC_DIR..."
cd "$SRC_DIR"

# Run make to build the generator
make clean 2>/dev/null || true  # Clean previous builds
make generator

# Check if compilation was successful
if [ ! -f "gen" ]; then
    echo "Error: Compilation failed. 'gen' executable not found."
    exit 1
fi

echo "✓ Compilation successful"

# Copy to backend/generator as 'main'
echo "Copying executable to $BACKEND_GENERATOR_DIR..."
cp gen "$BACKEND_GENERATOR_DIR/main"
chmod +x "$BACKEND_GENERATOR_DIR/main"

echo "✓ Executable ready at $BACKEND_GENERATOR_DIR/main"

# Verify it works
echo ""
echo "Testing executable..."
if "$BACKEND_GENERATOR_DIR/main" > /dev/null 2>&1 || [ $? -eq 1 ]; then
    echo "✓ Executable is functional"
else
    echo "⚠ Warning: Could not verify executable (this may be normal if the tool requires arguments)"
fi

echo ""
echo "Build complete! Ready for development."
