#!/bin/bash
# Environment setup for ONNX Runtime C++

export ONNXRUNTIME_ROOT="/home/jetson/vision-pilot/onnxruntime"
export LD_LIBRARY_PATH="/home/jetson/vision-pilot/onnxruntime/lib:${LD_LIBRARY_PATH}"
export PKG_CONFIG_PATH="/home/jetson/vision-pilot/onnxruntime/lib/pkgconfig:${PKG_CONFIG_PATH}"

echo "ONNX Runtime C++ environment configured:"
echo "  ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT}"
echo "  Libraries: Using pip-installed ONNX Runtime 1.19.0"
