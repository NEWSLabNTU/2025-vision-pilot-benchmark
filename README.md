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

## Test Videos and Datasets

VisionPilot works best with highway or street driving videos. Since the Autoware POV Working Group does not provide specific test videos, you can obtain suitable test footage from these sources:

### Free Stock Video Platforms

**Dashcam and Driving Videos:**
- **Pixabay**: https://pixabay.com/videos/search/dashcam/
  - Free dashcam videos, royalty-free, no attribution required
- **Pexels**: https://www.pexels.com/search/videos/dash%20cam/
  - 1,454+ free dash cam videos in 4K/HD
- **Videvo**: https://www.videvo.net/stock-video-footage/dashcam/
  - 428+ free dashcam videos in 4K/HD
- **POV Driving**: https://pixabay.com/videos/search/pov%20driving/
  - Point-of-view driving videos

### Academic Datasets (Research-Quality)

**Automotive Computer Vision Datasets:**
- **KITTI Dataset**: http://www.cvlibs.net/datasets/kitti/
  - Industry standard: 6 hours of highway/urban traffic scenarios
  - High-resolution RGB, stereo cameras, 3D laser scanner
  - Karlsruhe city, rural areas, and highways
- **BDD100K**: https://bair.berkeley.edu/blog/2018/05/30/bdd/
  - 100,000 videos, 40 seconds each, 720p at 30fps
  - Diverse US locations with GPS/IMU data
- **TU-DAT Dataset**: https://github.com/pavana27/TU-DAT
  - 280 real-world and simulated traffic videos
  - Traffic CCTV footage and BeamNG.drive simulations
- **Carolinas Highway Dataset (CHD)**:
  - 1.6 million frames, highway-based videos
  - 338,000 vehicle trajectories across Carolina highways

### Video Requirements

For optimal VisionPilot performance, use videos with:
- **Content**: Highway/street driving, clear road lanes and traffic
- **Quality**: 720p or higher resolution
- **Duration**: 30 seconds to 5 minutes recommended
- **Lighting**: Good visibility conditions
- **Format**: MP4 (H.264), AVI, MOV, or MKV

### Usage Example

```bash
# Download a test video (save to test_videos/ directory)
make download-test-video

# Run VisionPilot with your video
make run-web-service VIDEO=/path/to/your/video.mp4 PIPELINE=scene_seg

# Access web interface
# - Foxglove: https://app.foxglove.dev/ (connect to ws://ROBOT_IP:8765)
# - Video streams: http://ROBOT_IP:8080
```

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