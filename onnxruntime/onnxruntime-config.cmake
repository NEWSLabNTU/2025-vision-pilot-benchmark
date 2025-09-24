# ONNX Runtime CMake Configuration
# Using pip-installed libraries (1.19.0) with AAR headers

set(ONNXRUNTIME_ROOT_PATH "/home/jetson/vision-pilot/onnxruntime")
set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_ROOT_PATH}/include")
set(ONNXRUNTIME_LIB_DIRS "${ONNXRUNTIME_ROOT_PATH}/lib")
set(ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so")

# For find_package compatibility
set(onnxruntime_FOUND TRUE)
set(onnxruntime_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})
set(onnxruntime_LIBRARIES ${ONNXRUNTIME_LIBRARIES})
