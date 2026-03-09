#!/bin/bash
# About: Setup script for OpenVINS integration
# Installs OpenVINS from fork or original repo to /opt/openvins
# Run with: sudo ./setup_openvins.sh

set -e  # Exit on error

# Configuration
OPENVINS_DIR="/opt/openvins"
REPO_URL="${1:-https://github.com/rpng/open_vins.git}"  # Default to original, can pass fork URL
BRANCH="${2:-master}"  # Default branch

echo "=========================================="
echo "OpenVINS Installation Script"
echo "=========================================="
echo "Target directory: $OPENVINS_DIR"
echo "Repository: $REPO_URL"
echo "Branch: $BRANCH"
echo ""

# Check for sudo
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run with sudo"
    echo "Usage: sudo ./setup_openvins.sh [repo_url] [branch]"
    exit 1
fi

# Install dependencies
echo "Step 1/5: Installing dependencies..."
apt-get update
apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    || { echo "Failed to install dependencies"; exit 1; }

echo "✓ Dependencies installed"
echo ""

# Clone repository
echo "Step 2/5: Cloning OpenVINS repository..."
mkdir -p $OPENVINS_DIR
cd $OPENVINS_DIR

if [ -d "open_vins" ]; then
    echo "Repository exists, updating..."
    cd open_vins
    git fetch origin
    git checkout $BRANCH
    git pull origin $BRANCH
else
    echo "Cloning fresh repository..."
    git clone --recursive --branch $BRANCH $REPO_URL open_vins
    cd open_vins
fi

echo "✓ Repository ready at $(pwd)"
echo ""

# Build ov_core
echo "Step 3/5: Building ov_core library..."
cd ov_core
mkdir -p build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install
cd ../..

echo "✓ ov_core built and installed"
echo ""

# Build ov_msckf
echo "Step 4/5: Building ov_msckf library..."
cd ov_msckf
mkdir -p build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install
cd ../..

echo "✓ ov_msckf built and installed"
echo ""

# Update linker cache
echo "Step 5/5: Updating system configuration..."
ldconfig

# Create environment setup script
ENV_FILE="/etc/profile.d/openvins.sh"
cat > $ENV_FILE << 'EOF'
# OpenVINS environment variables
export OpenVINS_DIR=/opt/openvins/open_vins
export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH}
EOF
chmod +x $ENV_FILE

echo "✓ System configuration updated"
echo ""

# Verify installation
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo "Libraries installed:"
echo "  - /usr/local/lib/libov_core.so"
echo "  - /usr/local/lib/libov_msckf.so"
echo "  - /usr/local/include/open_vins/"
echo ""
echo "Environment configured in: $ENV_FILE"
echo ""
echo "To use in new terminals, run:"
echo "  source /etc/profile.d/openvins.sh"
echo ""
echo "Or add to your ~/.bashrc:"
echo "  export CMAKE_PREFIX_PATH=/usr/local:\${CMAKE_PREFIX_PATH}"
echo ""
echo "To verify:"
echo "  ls /usr/local/lib/libov_*.so"
echo "  ls /usr/local/include/open_vins/"
echo ""

# List installed files
ls -lh /usr/local/lib/libov_*.so 2>/dev/null || echo "Warning: Libraries not found in expected location"
ls -d /usr/local/include/open_vins/ 2>/dev/null || echo "Warning: Headers not found in expected location"
