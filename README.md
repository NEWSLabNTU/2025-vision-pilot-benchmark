# VisionPilot Installation and Build System

Automated installation and build system for VisionPilot from Autoware on Jetson Linux 36.3.

## Quick Start

```bash
# Install all prerequisites
make setup

# Build VisionPilot ROS2 packages
make build

# Run basic tests
make test

# Get help
make help
```

## Requirements

- Jetson Linux 36.3
- CUDA 12.x (must be pre-installed)
- Internet connection for downloads
- sudo access for system packages

## What This Does

The automated system uses Ansible playbooks (located in the `ansible/` directory):

1. **Installs Prerequisites**
   - Configures OpenCV 4.5.4 (Ubuntu repo version, not NVIDIA 4.8.0)
   - Downloads and installs ONNX Runtime 1.19.2 with CUDA support
   - Downloads and installs LibTorch 2.1.0
   - Downloads AI models (SceneSeg, Scene3D, DomainSeg)

2. **Builds VisionPilot**
   - Sources ROS2 Humble environment
   - Builds sensors, models, and visualization packages
   - Configures all paths automatically

3. **Validates Installation**
   - Checks all dependencies
   - Verifies model downloads
   - Tests basic functionality

## Available Commands

| Command | Description |
|---------|-------------|
| `make help` | Show available targets |
| `make setup` | Install prerequisites using Ansible |
| `make build` | Build ROS2 packages |
| `make clean` | Remove build artifacts |
| `make rebuild` | Clean and rebuild |
| `make validate` | Validate installation |
| `make test` | Run basic tests |
| `make check-deps` | Check dependencies |
| `make download-models` | Download AI models only |
| `make compile-tensorrt` | Compile TensorRT engines |

## Running VisionPilot

```bash
# Scene segmentation pipeline
make run-scene-seg VIDEO=/path/to/video.mp4

# Depth estimation pipeline
make run-depth VIDEO=/path/to/video.mp4

# List active topics
make list-topics
```

## File Structure

```
/home/jetson/vision-pilot/
├── Makefile                    # Main build system
├── autoware-pov/              # VisionPilot source code (submodule)
├── ansible/                   # Ansible playbooks and configuration
└── README.md                  # This file

/opt/visionpilot/
├── models/                    # AI model files
├── scripts/                   # Standalone scripts
└── validation_report.txt      # Installation validation

/etc/profile.d/
└── visionpilot.sh            # Environment variables
```

## Performance on AGX Orin

**With TensorRT Engines (FP16)**:
- Scene/Domain Segmentation: 40-60 FPS
- Depth Estimation: 25-40 FPS

**With ONNX Runtime (FP32)**:
- Scene/Domain Segmentation: 15-25 FPS
- Depth Estimation: 10-20 FPS

**Note**: TensorRT engines are automatically compiled during installation for ~2-4x performance improvement.

## Troubleshooting

### Installation Issues
```bash
# Check what's missing
make check-deps

# Re-run setup
make setup

# Validate installation
make validate
```

### Build Issues
```bash
# Clean and rebuild
make rebuild

# Check ROS2 workspace
cd autoware-pov/VisionPilot/ROS2
source install/setup.bash
ros2 pkg list | grep -E "(sensors|models|visualization)"
```

### OpenCV Version Conflicts
The system automatically configures APT to prefer Ubuntu OpenCV 4.5.4 over NVIDIA 4.8.0. Check:
```bash
dpkg -l | grep libopencv-dev
cat /etc/apt/preferences.d/opencv-preferences
```

## Advanced Usage

For detailed Ansible configuration and troubleshooting, see [ansible/README.md](ansible/README.md).

## License

This build system is provided as-is for VisionPilot setup on Jetson platforms.