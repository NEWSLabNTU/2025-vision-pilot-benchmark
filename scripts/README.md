# VisionPilot Scripts

This directory contains utility scripts for VisionPilot configuration and benchmarking.

## Config Generation

### `generate_configs.py`

Generates ROS2 config files from templates for different backend/precision combinations.

**Usage:**
```bash
# Via Makefile (recommended)
make config BACKEND=tensorrt PRECISION=fp16
make config BACKEND=tensorrt PRECISION=fp32
make config BACKEND=onnxruntime PRECISION=fp32

# Direct invocation
python3 scripts/generate_configs.py \
    --backend tensorrt \
    --precision fp16 \
    --models-dir /path/to/models \
    --output-dir autoware-pov/VisionPilot/ROS2/models/config \
    --templates-dir scripts/config_templates
```

**Parameters:**
- `BACKEND`: `tensorrt` or `onnxruntime` (default: `tensorrt`)
- `PRECISION`: `fp16` or `fp32` (default: `fp32`)

**Output:**
- `autoware-pov/VisionPilot/ROS2/models/config/autoseg.yaml`
- `autoware-pov/VisionPilot/ROS2/models/config/auto3d.yaml`

## Benchmarking

### `run_benchmark.py`

Python script that runs comprehensive benchmarks across all backend, model, and precision combinations.

**Tested Combinations:**
- **Backends**: `tensorrt`, `onnxruntime` (or both)
- **Models**: `scene_seg`, `domain_seg`, `scene_3d`
- **Precisions**: `fp16`, `fp32`

Default: 12 tests (2 backends × 3 models × 2 precisions)

**Usage:**
```bash
# Via Makefile (recommended) - tests TensorRT only
make benchmark VIDEO=video/8358-208052058.mp4 DURATION=10

# Test both backends with defaults (12 tests)
# TensorRT: 10s init + 10s runtime
# ONNX Runtime: 120s init + 30s runtime (auto-increased)
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

**Parameters:**
- `video_path`: Path to video file (required, positional)
- `--duration`: Default runtime duration for all backends (default: 10s)
- `--backend`: Backend to test: `tensorrt`, `onnxruntime`, or `both` (default: `both`)
- `--tensorrt-init`: TensorRT initialization period in seconds (default: 10)
- `--tensorrt-duration`: TensorRT runtime duration in seconds (default: same as `--duration`)
- `--onnxruntime-init`: ONNX Runtime initialization period in seconds (default: 120)
- `--onnxruntime-duration`: ONNX Runtime runtime duration in seconds (default: max(30, `--duration`))

**Why different settings for ONNX Runtime?**

ONNX Runtime requires significantly longer initialization time:
- **CUDA Provider Setup**: ~60-70 seconds to initialize
- **Model Loading**: Additional time to load ONNX models
- **First Inference Delay**: First logs appear at frame 100, not frame 1
- **Recommended minimum**: 120s init + 30s runtime for reliable data collection

**What it does:**
1. Creates a summary file immediately with header information
2. For each backend/model/precision combination:
   - Generates appropriate config files
   - Stops any running services
   - Starts VisionPilot with the specified configuration
   - Monitors CPU and memory usage during execution
   - Collects logs for the specified duration
   - Extracts performance metrics (FPS, latency, CPU, memory)
   - **Writes result immediately to summary file** (crash-safe)
   - Stops the service

3. Writes footer with completion status

**Crash Recovery:**
Results are written incrementally after each test completes. If the benchmark crashes or is interrupted, you can still view partial results in the summary file.

**Performance Metrics Collected:**

All metrics are averaged over the last 10 reported frames:

- **Inference FPS**: Frames per second (inference throughput)
- **Inference Latency**: Inference latency in milliseconds per frame
- **Mask Time**: Time to generate mask from tensor (ms) - for segmentation models only
- **Mask FPS**: Mask generation throughput (calculated from mask time)
- **Visualization Latency**: Time to render visualization (ms)
- **Visualization FPS**: Visualization rendering throughput
- **CPU Average**: Average CPU % during runtime (excludes initialization)
- **CPU Peak**: Peak CPU % during entire period (includes initialization)
- **Memory Average**: Average memory MB during runtime (excludes initialization)
- **Memory Peak**: Peak memory MB during entire period (includes initialization)

**Note**: The latency values shown in logs (e.g., "Frame 100", "Frame 200") represent the latency for that specific frame, not an average over 100 frames. The benchmark script averages the last 10 frame measurements.

**Timing Phases:**
1. **Initialization Phase**: Model loading, CUDA setup (duration: `--*-init`)
   - Peak CPU/memory includes this phase
2. **Runtime Phase**: Active inference and monitoring (duration: `--*-duration`)
   - Average CPU/memory calculated from this phase only

**Output Format: CSV**

Results are saved in CSV format for easy analysis in spreadsheets or scripts:

- `benchmark_results/TIMESTAMP/` - Directory with all results
  - `benchmark_results.csv` - CSV file with all metrics (updated after each test)
  - `tensorrt_scene_seg_fp16.log` - Individual test logs
  - `tensorrt_scene_seg_fp32.log`
  - `tensorrt_domain_seg_fp16.log`
  - `tensorrt_domain_seg_fp32.log`
  - `tensorrt_scene_3d_fp16.log`
  - `tensorrt_scene_3d_fp32.log`
  - `onnxruntime_scene_seg_fp16.log` (if testing both backends)
  - `onnxruntime_scene_seg_fp32.log`
  - ... (and so on for ONNX Runtime)

**CSV Columns:**
```
backend,pipeline,precision,status,inference_fps,inference_latency_ms,mask_time_ms,mask_fps,viz_latency_ms,viz_fps,cpu_avg,cpu_peak,mem_avg_mb,mem_peak_mb
```

**Example CSV output:**
```csv
# VisionPilot Benchmark Results
# Timestamp: 2025-10-09 19:31:45
# Video: /home/jetson/.../video/8358-208052058.mp4
# Runtime duration: 10s (TensorRT), 30s (ONNX Runtime)
# Init period: 10s (TensorRT), 120s (ONNX Runtime)
# Note: CPU/Memory averages are runtime only; peaks include initialization
# Backends: tensorrt, onnxruntime
#
backend,pipeline,precision,status,inference_fps,inference_latency_ms,mask_time_ms,mask_fps,viz_latency_ms,viz_fps,cpu_avg,cpu_peak,mem_avg_mb,mem_peak_mb
tensorrt,scene_seg,fp16,SUCCESS,32.8,30.64,0.75,1333.33,21.89,45.97,63.4,150.2,584.5,952.3
tensorrt,scene_seg,fp32,SUCCESS,18.2,54.90,0.59,1694.92,10.35,96.62,45.1,129.8,373.0,925.7
tensorrt,domain_seg,fp16,SUCCESS,31.8,31.45,0.73,1369.86,8.75,114.29,58.2,138.5,501.0,943.2
onnxruntime,scene_seg,fp32,SUCCESS,8.5,117.15,1.39,719.42,33.09,32.97,47.5,149.7,952.0,954.6
...
#
# Completed: 12/12 tests
```

**Console output (human-readable):**
```
tensorrt/scene_seg fp16: Inf: 32.8 FPS (30.64 ms) | Mask: 1333.3 FPS (0.75 ms) | Viz: 46.0 FPS (21.89 ms) | CPU: 63.4% (peak: 150.2%) | Mem: 585 MB (peak: 952 MB)
onnxruntime/scene_seg fp32: Inf: 8.5 FPS (117.15 ms) | Mask: 719.4 FPS (1.39 ms) | Viz: 33.0 FPS (33.09 ms) | CPU: 47.5% (peak: 149.7%) | Mem: 952 MB (peak: 955 MB)
```

## Config Templates

### `config_templates/`

Contains YAML template files used by `generate_configs.py`:

- `autoseg.yaml.template` - Template for scene_seg and domain_seg models
- `auto3d.yaml.template` - Template for scene_3d model

**Template variables:**
- `{MODEL_PATH}` - Absolute path to models directory
- `{BACKEND}` - Backend name (tensorrt/onnxruntime)
- `{PRECISION}` - Precision (fp16/fp32)

## Quick Reference

```bash
# Generate configs
make config BACKEND=tensorrt PRECISION=fp16

# Run single test
make start-web VIDEO=video/test.mp4 PIPELINE=scene_seg

# Run TensorRT benchmark via Makefile (6 tests, 10s each)
make benchmark VIDEO=video/test.mp4

# Run both backends (12 tests: TensorRT + ONNX Runtime)
python3 scripts/run_benchmark.py video/test.mp4 --backend both

# Run longer benchmark (30s per test)
python3 scripts/run_benchmark.py video/test.mp4 --duration 30 --backend both

# Run only ONNX Runtime backend (6 tests)
python3 scripts/run_benchmark.py video/test.mp4 --backend onnxruntime
```
