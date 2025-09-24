# VisionPilot Prerequisites Ansible Installation

This directory contains Ansible playbooks to automate the installation of VisionPilot prerequisites on Jetson Linux 36.3.

## Prerequisites

- Jetson Linux 36.3
- CUDA 12.x pre-installed (not automated to prevent system breakage)
- sudo access
- Internet connection

## Quick Start

```bash
# Install prerequisites
make setup

# Build VisionPilot
make build

# Run tests
make test
```

## Manual Ansible Commands

```bash
# Navigate to ansible directory
cd ansible

# Install Ansible if not present
sudo apt update && sudo apt install -y ansible

# Run complete installation
ansible-playbook -i inventory.ini site.yml

# Run only specific parts
ansible-playbook -i inventory.ini site.yml --tags install  # Prerequisites only
ansible-playbook -i inventory.ini site.yml --tags models   # Models only (using gdown 5.2.0)
ansible-playbook -i inventory.ini site.yml --tags validate # Validation only

# Run with verbose output
ansible-playbook -i inventory.ini site.yml -v

# Run with check mode (dry run)
ansible-playbook -i inventory.ini site.yml --check
```

## Available Make Targets

```bash
make help           # Show available targets
make setup          # Install prerequisites using Ansible
make build          # Build ROS2 packages
make clean          # Remove build artifacts
make rebuild        # Clean and rebuild
make validate       # Validate installation
make test           # Run basic tests
make check-deps     # Check dependencies
make download-models # Download AI models only
make run-scene-seg VIDEO=/path/to/video.mp4  # Run segmentation
make run-depth VIDEO=/path/to/video.mp4      # Run depth estimation
make list-topics    # List ROS2 topics
```

## What Gets Installed

### System Packages
- **OpenCV 4.5.4** (Ubuntu repo version, not NVIDIA 4.8.0)
  - Configured via APT pinning in `/etc/apt/preferences.d/opencv-preferences`
- Build tools (cmake, build-essential, etc.)
- Download tools (wget, curl, gdown)

### AI/ML Libraries
- **onnxruntime-gpu==1.19.2** via pip with CUDA support
- **gdown==5.2.0** via pip for reliable Google Drive downloads
  - Pinned versions for future-proofing
- **nvidia-tensorrt** system package for optimized inference

### AI Models
All models downloaded to `/opt/visionpilot/models/`:
- **SceneSeg_FP32.onnx** - Object segmentation (185MB)
- **Scene3D_FP32.onnx** - Depth estimation (185MB)
- **DomainSeg_FP32.onnx** - Construction zone segmentation (185MB)
- **SceneSeg_INT8.onnx** - Optimized INT8 version (optional)

### Environment Configuration
- Environment variables set in `/etc/profile.d/visionpilot.sh`
- Standalone scripts in `/opt/visionpilot/scripts/`
- System commands: `visionpilot-download-models`, `visionpilot-compile-engines`

## Important Notes

### OpenCV Version Management
The playbook specifically configures APT to prefer Ubuntu's OpenCV 4.5.4 over NVIDIA's 4.8.0 to avoid dependency conflicts. This is done through APT pinning with priority 1001.

### CUDA Requirements
CUDA must be pre-installed. The playbook will check for CUDA but will NOT install it to prevent potential system issues. Install CUDA manually if not present.

### Google Drive Downloads with Enhanced Features
Models are hosted on Google Drive. The system uses `gdown 5.2.0` with:
- **Retry Logic**: 3 attempts per model with exponential backoff
- **Checksum Validation**: SHA256 checksums generated and verified
- **Size Validation**: File size verification against expected values
- **Bulk Download**: JSON manifest-driven batch processing
- **Filtering Options**: Download by category, format, or specific models
- **Progress Tracking**: Real-time download and validation progress

#### Advanced Model Download Options:
```bash
# Download only required ONNX models (default)
visionpilot-download-models --required-only --format onnx_fp32

# Download all models for a specific category
visionpilot-download-models --category segmentation

# Download specific models
visionpilot-download-models --models SceneSeg_FP32.onnx Scene3D_FP32.onnx

# Compile TensorRT engines for better performance
visionpilot-compile-engines --precision fp16
```

## Validation

After installation, the validation playbook checks:
- CUDA version (12.x required)
- OpenCV version (4.5.4 required, not 4.8.0)
- ONNX Runtime GPU with CUDA providers
- gdown 5.2.0 installation
- Model files size and presence
- TensorRT availability and engines
- Environment setup script

View the validation report:
```bash
cat /opt/visionpilot/validation_report.txt
```

## Building VisionPilot After Installation

```bash
# Simple build using Makefile
make build

# Or clean and rebuild
make rebuild
```

## Troubleshooting

### OpenCV Conflicts
If you see OpenCV 4.8.0 being installed:
```bash
# Check APT preferences
cat /etc/apt/preferences.d/opencv-preferences

# Force reinstall 4.5.4
sudo apt-get install libopencv-dev=4.5.4+dfsg-9ubuntu4
```

### Download Failures
If model downloads fail:
```bash
# Manually run download script
visionpilot-download-models --required-only

# Or use gdown directly
cd /opt/visionpilot/models
gdown <file_id> -O <filename>
```

### Permission Issues
```bash
# Fix permissions
sudo chown -R $USER:$USER /opt/visionpilot
```

## File Structure

```
/opt/visionpilot/
├── models/
│   ├── SceneSeg_FP32.onnx
│   ├── Scene3D_FP32.onnx
│   ├── DomainSeg_FP32.onnx
│   ├── SceneSeg_FP32.fp16.engine  # TensorRT engines
│   ├── models_manifest.json
│   └── master_checksums.sha256
├── scripts/
│   ├── download_models.py
│   └── compile_tensorrt_engines.py
└── validation_report.txt

/usr/local/bin/
├── visionpilot-download-models -> /opt/visionpilot/scripts/download_models.py
└── visionpilot-compile-engines -> /opt/visionpilot/scripts/compile_tensorrt_engines.py

/etc/
├── apt/preferences.d/
│   └── opencv-preferences
└── profile.d/
    └── visionpilot.sh
```

## License

These installation scripts are provided as-is for VisionPilot setup on Jetson platforms.