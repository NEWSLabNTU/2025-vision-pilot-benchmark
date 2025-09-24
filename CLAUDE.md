# VisionPilot Project Context

## Project Overview
This is a VisionPilot installation and automation system for AGX Orin running Jetson Linux 36.3 with ROS2 Humble. The project provides automated installation of AI perception components from Autoware.foundation.

## System Architecture
- **Main build system**: Makefile with targets for setup, build, clean, test, validation
- **Automation**: Ansible playbooks in `ansible/` directory
- **Dependencies**: pip-based installations with NVIDIA-provided wheels for ARM64
- **AI Models**: Downloaded to local `models/` directory with optional TensorRT compilation
- **Scripts**: Located in `scripts/` directory for model management

## Key Commands
```bash
# Primary workflow
make setup          # Install prerequisites using Ansible (interactive TensorRT prompt)
make build          # Build ROS2 packages
make test           # Run basic tests
make validate       # Validate installation

# Model management
make download-models        # Download ONNX models only
make compile-tensorrt       # Compile TensorRT engines (10-15 min)
make tensorrt-engines       # Alias for compile-tensorrt

# Running pipelines
make run-scene-seg VIDEO=/path/to/video.mp4
make run-depth VIDEO=/path/to/video.mp4
```

## Important Technical Details

### OpenCV Version Management
- Uses Ubuntu OpenCV 4.5.4 (NOT NVIDIA 4.8.0)
- APT preferences configured with source-based pinning: `Pin: origin "Ubuntu"`
- Located in `ansible/files/opencv-preferences`

### Dependencies Installation Method
- **ONNX Runtime GPU**: NVIDIA-provided wheel for JetPack 6.0 (1.19.0) with CUDA support
- **NumPy compatibility**: Uses system NumPy 1.21.5, uninstalls user NumPy to avoid conflicts
- **Installation method**: `pip install --user --no-deps` to prevent dependency conflicts
- **gdown**: `gdown==5.2.0` via pip for Google Drive downloads
- **TensorRT**: Uses system package `nvidia-tensorrt` at `/usr/src/tensorrt/bin/trtexec`

### TensorRT Compilation
- **Interactive prompt**: During `make setup`, user can choose to compile (default: no)
- **Compilation time**: 10-15 minutes for all 3 models (SceneSeg, Scene3D, DomainSeg)
- **Fixed batch size**: Uses 1x3x320x640 to avoid reshape conflicts
- **Optimized flags**: `--memPoolSize=workspace:4096 --fp16 --skipInference --noTF32`
- **Performance gain**: 2-4x speedup over ONNX Runtime

### Build Configuration
- ROS2 packages: sensors, models, visualization
- Build only requires OpenCV_DIR (no ONNX_ROOT or TORCH_ROOT)
- Located in `autoware-pov/VisionPilot/ROS2/` (git submodule)

### File Structure
```
/home/jetson/vision-pilot/
├── Makefile                    # Main build system
├── autoware-pov/              # VisionPilot source (git submodule)
├── models/                    # ONNX models and TensorRT engines (local)
├── scripts/                   # Model download/compilation scripts
│   ├── download_models.py     # Downloads ONNX models
│   ├── compile_tensorrt_engines.py  # Compiles TensorRT engines
│   └── models_manifest.json   # Model metadata
└── ansible/                   # Ansible automation
    ├── site.yml              # Master playbook
    ├── install_prerequisites.yml  # Main installation
    ├── validate_installation.yml  # Installation validation
    └── files/                # Config files (opencv-preferences, etc.)
```

## Current Status
- ✅ Prerequisites installation fully automated with Ansible
- ✅ ONNX Runtime GPU working with proper NumPy compatibility
- ✅ TensorRT compilation fixed and working (interactive prompt added)
- ✅ Local directory structure (no `/opt/visionpilot/` usage)
- ✅ All 3 AI models download and can be compiled to TensorRT engines
- ✅ Graceful fallback from TensorRT to ONNX Runtime

## Recent Fixes Applied
- **ONNX Runtime installation**: Fixed NumPy 2.x compatibility by using `--no-deps` and system NumPy
- **TensorRT compilation**: Fixed batch size reshape conflicts, updated to modern flags
- **Model dimensions**: Corrected hardcoded dimensions from 480x640 to 320x640
- **Interactive setup**: Added user prompt for TensorRT compilation with default "no"
- **PATH configuration**: Fixed TensorRT PATH to use `/usr/src/tensorrt/bin/trtexec`

## Lint/Typecheck Commands
Based on the ROS2 nature of this project, typical commands would be:
- `colcon build` (already integrated in make build)
- Check project-specific linting in package.xml or CMakeLists.txt files

## Performance Notes
- **With TensorRT engines (FP16)**: Scene/Domain Segmentation 40-60 FPS, Depth Estimation 25-40 FPS
- **With ONNX Runtime (FP32)**: Scene/Domain Segmentation 15-25 FPS, Depth Estimation 10-20 FPS
- **TensorRT engines**: Provide 2-4x performance improvement, optional during setup