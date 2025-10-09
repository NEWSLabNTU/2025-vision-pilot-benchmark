# VisionPilot on AGX Orin: Setup and Performance Benchmark

Automated installation, build system, and performance benchmarking for VisionPilot from Autoware on NVIDIA Jetson AGX Orin (Jetson Linux 36.3).

## Quick Start

### Prerequisites

- Jetson Linux 36.3
- CUDA 12.x (must be pre-installed)
- Internet connection for downloads
- sudo access for system packages

### Installation

```bash
# 1. Install all prerequisites using Ansible
make setup

# 2. Build VisionPilot ROS2 packages
make build

# 3. Verify installation
make test
```

### Run VisionPilot

Start VisionPilot as a systemd service with the included test video:

```bash
# Start the web service (includes Foxglove interface)
make start-web VIDEO=$PWD/video/8358-208052058.mp4

# Check service status
make status-web

# View live logs
make logs-web

# Access the web interface:
#   Foxglove Studio: https://app.foxglove.dev/ (connect to ws://ROBOT_IP:8765)
#   Video streams: http://ROBOT_IP:8080

# Stop the service when done
make stop-web
```

**Note:** For more test videos and datasets, see [video/README.md](video/README.md).

## Available Commands

Run `make help` to see all available targets.

### Setup and Build
```bash
make setup              # Install prerequisites using Ansible
make build              # Build ROS2 packages
make clean              # Remove build artifacts
make test               # Run basic tests
```

### Systemd Service Management (Recommended)

**Web Service** (full interface with Foxglove):
```bash
make start-web VIDEO=/path/to/video.mp4 [PIPELINE=scene_seg]
make stop-web
make status-web
make logs-web
make remove-web
```

**Simple Service** (minimal interface):
```bash
make start-simple VIDEO=/path/to/video.mp4 [PIPELINE=scene_seg]
make stop-simple
make status-simple
make logs-simple
make remove-simple
```

### Direct Pipeline Execution

For testing or debugging without systemd:
```bash
make run-pipeline VIDEO=/path/to/video.mp4 PIPELINE=scene_seg    # Generic
make run-scene-seg VIDEO=/path/to/video.mp4                      # Scene segmentation
make run-domain-seg VIDEO=/path/to/video.mp4                     # Domain segmentation
make run-depth VIDEO=/path/to/video.mp4                          # Depth estimation
```

**Available Pipelines**: `scene_seg`, `domain_seg`, `scene_3d`

## File Structure

```
/home/jetson/2025-vision-pilot-benchmark/
├── Makefile                    # Main build system
├── README.md                   # This file
├── ansible/                    # Ansible playbooks and configuration
├── autoware-pov/               # VisionPilot source code (git submodule)
│   └── VisionPilot/ROS2/       # ROS2 packages
├── models/                     # AI models (ONNX and TensorRT engines)
├── onnxruntime/                # ONNX Runtime C++ headers
├── scripts/                    # Model download/compilation scripts
└── video/                      # Test videos
```

## Performance

For detailed performance benchmarks on AGX Orin, see [models/statistic.md](models/statistic.md).

## Advanced Usage

For detailed Ansible configuration and troubleshooting, see [ansible/README.md](ansible/README.md).

## License

Copyright 2025 NEWSLAB, National Taiwan University

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
