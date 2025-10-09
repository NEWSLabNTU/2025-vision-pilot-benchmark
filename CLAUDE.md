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

# Running pipelines (direct execution)
make run-scene-seg VIDEO=/path/to/video.mp4
make run-depth VIDEO=/path/to/video.mp4

# Running as systemd services
make run-web-service VIDEO=/path/to/video.mp4 PIPELINE=scene_seg
make run-simple-service VIDEO=/path/to/video.mp4 PIPELINE=scene_seg
make stop-vision-services     # Stop all VisionPilot services
make status-vision-services   # Check service status
make logs-vision-services SERVICE=vision-pilot-web  # View service logs

# Test video available at: video/8358-208052058_tiny.mp4
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
├── Makefile                    # Main build system with systemd service targets
├── autoware-pov/              # VisionPilot source (git submodule)
│   └── VisionPilot/ROS2/
│       └── vision_pilot_launch/  # Custom launch package with XML files
├── models/                    # ONNX models and TensorRT engines (local)
├── scripts/                   # Model download/compilation scripts
│   ├── download_models.py     # Downloads ONNX models
│   ├── compile_tensorrt_engines.py  # Compiles TensorRT engines
│   └── models_manifest.json   # Model metadata
├── onnxruntime/               # ONNX Runtime C++ headers and libraries
│   └── capi_dir/              # Symlink to pip-installed ONNX Runtime
├── video/                     # Test videos (see video/README.md for sources)
└── ansible/                   # Ansible automation
    ├── site.yml              # Master playbook
    ├── install_prerequisites.yml  # Main installation
    ├── validate_installation.yml  # Installation validation
    ├── setup_onnxruntime_symlink.yml  # ONNX Runtime symlink setup
    └── files/                # Config files (opencv-preferences, etc.)
```

## Current Status
- ✅ Prerequisites installation fully automated with Ansible
- ✅ ONNX Runtime GPU working with proper NumPy compatibility
- ✅ TensorRT compilation fixed and working (interactive prompt added)
- ✅ Local directory structure (no `/opt/visionpilot/` usage)
- ✅ All 3 AI models download and can be compiled to TensorRT engines
- ✅ Graceful fallback from TensorRT to ONNX Runtime
- ✅ ROS2 launch package `vision_pilot_launch` with web interface integration
- ✅ Systemd service management via ros2systemd integration
- ✅ Test videos available in `video/` directory (see `video/README.md` for sources)
- ⚠️ **PENDING**: Systemd user logging requires group membership fix (see Known Issues)

## Recent Fixes Applied
- **ONNX Runtime installation**: Fixed NumPy 2.x compatibility by using `--no-deps` and system NumPy
- **TensorRT compilation**: Fixed batch size reshape conflicts, updated to modern flags
- **Model dimensions**: Corrected hardcoded dimensions from 480x640 to 320x640
- **Interactive setup**: Added user prompt for TensorRT compilation with default "no"
- **PATH configuration**: Fixed TensorRT PATH to use `/usr/src/tensorrt/bin/trtexec`
- **Launch package creation**: Added `vision_pilot_launch` ROS2 package with XML launch files
- **Systemd integration**: Added ros2systemd support with Makefile targets for service management
- **ONNX Runtime C++ headers**: Fixed missing headers using AAR extraction method

## Lint/Typecheck Commands
Based on the ROS2 nature of this project, typical commands would be:
- `colcon build` (already integrated in make build)
- Check project-specific linting in package.xml or CMakeLists.txt files

## Known Issues

### Systemd User Service Logging
**Problem**: `journalctl --user` shows "No journal files were found" even after configuring persistent logging.

**Root Cause**: User is not in the `systemd-journal` group, which is required to access journal files.

**Solution**: Add user to systemd-journal group (requires sudo):
```bash
sudo usermod -a -G systemd-journal $USER
# Then log out and log back in, or reboot
```

**Verification**: After relogging, check with:
```bash
groups $USER  # Should include systemd-journal
journalctl --user --no-pager --lines=5  # Should show logs
```

**Alternative**: For immediate testing without reboot, use `newgrp systemd-journal` after adding to group.

## Performance Notes
- **With TensorRT engines (FP16)**: Scene/Domain Segmentation 40-60 FPS, Depth Estimation 25-40 FPS
- **With ONNX Runtime (FP32)**: Scene/Domain Segmentation 15-25 FPS, Depth Estimation 10-20 FPS
- **TensorRT engines**: Provide 2-4x performance improvement, optional during setup

## Launch Package Integration
- **vision_pilot_launch**: Custom ROS2 package with XML launch files
- **vision_pilot_web.launch.xml**: Full pipeline with Foxglove Bridge, web services, visualization
- **vision_pilot_simple.launch.xml**: Basic pipeline with minimal web interface
- **ros2systemd integration**: Automatic systemd service creation for persistent background execution
- **Web interfaces**:
  - Foxglove: `ws://ROBOT_IP:8765` (WebSocket)
  - Video streams: `http://ROBOT_IP:8080` (HTTP)