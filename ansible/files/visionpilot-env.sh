#!/bin/bash
# VisionPilot Environment Setup
# This file is managed by Ansible - do not edit manually

# Models directory
export VISIONPILOT_MODELS_DIR="/opt/visionpilot/models"

# OpenCV (ensure Ubuntu 4.5.4 is used)
export OpenCV_DIR="/usr/lib/aarch64-linux-gnu/cmake/opencv4"

# CUDA paths (if needed by applications)
export CUDA_HOME="/usr/local/cuda"
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"

# TensorRT paths
export TENSORRT_ROOT="/usr"
export LD_LIBRARY_PATH="${TENSORRT_ROOT}/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"