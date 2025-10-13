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

### Configuration Management
```bash
# Generate ROS2 config files for different backends and precisions
make config BACKEND=tensorrt PRECISION=fp16     # TensorRT FP16 (fastest)
make config BACKEND=tensorrt PRECISION=fp32     # TensorRT FP32 (balanced)
make config BACKEND=onnxruntime PRECISION=fp32  # ONNX Runtime FP32

# View current config
cat autoware-pov/VisionPilot/ROS2/models/config/autoseg.yaml
cat autoware-pov/VisionPilot/ROS2/models/config/auto3d.yaml
```

**Default:** TensorRT FP32 (configured during `make build`)

**Notes:**
- TensorRT supports both FP16 (faster) and FP32 (more accurate)
- ONNX Runtime only supports FP32 (no FP16 ONNX models available)
- Config changes require service restart: `make stop-web && make start-web ...`

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
make run-pipeline VIDEO=/path/to/video.mp4 PIPELINE=scene_seg
```

**Available Pipelines**: `scene_seg`, `domain_seg`, `scene_3d`

**Note:** Direct execution runs in the foreground. Use Ctrl+C to stop. For background execution, use systemd services above.

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

## Performance Benchmarking

### View Benchmark Results

For detailed performance benchmarks on AGX Orin, see [docs/BENCHMARK_RESULTS.md](docs/BENCHMARK_RESULTS.md).

### Pre-compiled Models

Pre-compiled TensorRT engines for AGX Orin 64GB (optimized, ready to use):

- **Download:** [AGX Orin 64G Pre-compiled Models](https://newslabn.csie.ntu.edu.tw/drive/d/s/15KS5zS0saqd5CvLHFfungJMsOitkC3V/x9WpnVsIh3tjeD_TEIRlLMKsaXUfdelS-e76AXV8xpgw)
- Includes FP16 and FP32 engines for all three models (SceneSeg, DomainSeg, Scene3D)
- Extract to `models/` directory to skip the 10-15 minute TensorRT compilation step
- Achieves the performance shown in [docs/BENCHMARK_RESULTS.md](docs/BENCHMARK_RESULTS.md)

### Run Your Own Benchmarks

#### Quick Start (Makefile)

```bash
# Run TensorRT benchmark (6 tests: 3 models × 2 precisions)
make benchmark VIDEO=video/8358-208052058.mp4 DURATION=10
```

**Output:** CSV results in `benchmark_results/<timestamp>/benchmark_results.csv`

#### Advanced Benchmarking (Python Script)

For more control and backend comparison:

```bash
# Test both TensorRT and ONNX Runtime (12 tests - recommended)
# Default: TensorRT (10s init + 10s runtime), ONNX Runtime (120s init + 30s runtime)
python3 scripts/run_benchmark.py video/8358-208052058.mp4

# Test only TensorRT (6 tests)
python3 scripts/run_benchmark.py video/8358-208052058.mp4 --backend tensorrt

# Test only ONNX Runtime with custom timing (6 tests)
python3 scripts/run_benchmark.py video/8358-208052058.mp4 \
  --backend onnxruntime \
  --onnxruntime-init 120 \
  --onnxruntime-duration 60

# Full customization for both backends
python3 scripts/run_benchmark.py video/8358-208052058.mp4 \
  --tensorrt-init 10 --tensorrt-duration 15 \
  --onnxruntime-init 120 --onnxruntime-duration 45
```

**Available Options:**
- `--backend`: `tensorrt`, `onnxruntime`, or `both` (default: `both`)
- `--duration`: Default runtime duration for all backends (default: 10s)
- `--tensorrt-init`: TensorRT initialization period (default: 10s)
- `--tensorrt-duration`: TensorRT runtime duration (default: same as `--duration`)
- `--onnxruntime-init`: ONNX Runtime initialization period (default: 120s)
- `--onnxruntime-duration`: ONNX Runtime runtime duration (default: max(30, `--duration`))

**Why different settings for ONNX Runtime?**
- ONNX Runtime requires ~60-70 seconds to initialize CUDA provider
- Minimum 30 seconds runtime needed to collect meaningful data
- See [scripts/README.md](scripts/README.md) for detailed timing recommendations

#### What Gets Tested

**Backends:**
- TensorRT (with FP16 and FP32 engines)
- ONNX Runtime (FP32 only, no FP16 ONNX models available)

**Models:**
- `scene_seg`: Scene segmentation
- `domain_seg`: Domain segmentation
- `scene_3d`: Depth estimation

**Total tests:** 6 (TensorRT only) or 12 (both backends)

#### Collected Metrics

For each combination, the benchmark collects:
- **Inference FPS** and **latency** (ms)
- **Mask generation time** (for segmentation models)
- **Visualization FPS** and **latency**
- **CPU usage**: Average during runtime, peak during init + runtime
- **Memory usage**: Average during runtime, peak during init + runtime

**Output format:** CSV with metadata (crash-safe incremental writing)

#### Example Results

```
tensorrt/scene_seg fp16: Inf: 32.8 FPS (30.64 ms) | CPU: 63.4% (peak: 150%) | Mem: 585 MB (peak: 952 MB)
onnxruntime/scene_seg fp32: Inf: 8.5 FPS (117.15 ms) | CPU: 47.5% (peak: 150%) | Mem: 952 MB (peak: 955 MB)
```

For complete documentation, see [scripts/README.md](scripts/README.md).

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
