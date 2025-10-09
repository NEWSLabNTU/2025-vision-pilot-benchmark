# VisionPilot Benchmark Results

> **Last updated:** 2025-10-09
> **Test duration:** 10 seconds per configuration
> **Video:** 8358-208052058.mp4
> **Backend:** TensorRT
> **Environment:** Jetson AGX Orin (JetPack 6.0, L4T 36.3)
> **Raw data:** [benchmark_results/20251009_161647/](../benchmark_results/20251009_161647/)

---

## Performance Summary

| Model          | Precision | Inf Lat (ms) | Inf FPS | Mask (ms) | Viz Lat (ms) | CPU (%) | Mem (MB) |
|:---------------|:----------|-------------:|--------:|----------:|-------------:|--------:|---------:|
| **domain_seg** | FP16      |        32.38 |   31.25 |      0.29 |        23.52 |   63.82 |      572 |
| **scene_3d**   | FP16      |        34.09 |   29.70 |       N/A |         9.24 |   72.42 |      647 |
| **scene_seg**  | FP16      |        32.96 |   30.38 |      0.71 |        21.10 |   64.53 |      581 |
| **domain_seg** | FP32      |        57.07 |   17.55 |      0.23 |        22.62 |   46.24 |      455 |
| **scene_3d**   | FP32      |        54.17 |   18.50 |       N/A |        13.61 |   55.02 |      528 |
| **scene_seg**  | FP32      |        52.62 |   19.00 |      0.76 |        21.71 |   48.61 |      456 |

### Column Explanations

**Model & Precision:**
- **Model**: AI pipeline - `scene_seg` (scene segmentation), `domain_seg` (domain segmentation), `scene_3d` (depth estimation)
- **Precision**: `FP16` (16-bit, faster) or `FP32` (32-bit, higher precision)

**Inference Metrics:**
- **Inf Lat (ms)**: Inference latency per frame (lower is better)
- **Inf FPS**: Inference throughput (higher is better, calculated as 1000/latency)

**Post-Processing Metrics:**
- **Mask (ms)**: Time to convert output tensor to visualization mask (segmentation only, N/A for depth)
- **Viz Lat (ms)**: Time to render and encode visualization for display/streaming

Note: Mask and visualization steps execute after inference completes, so their latencies add to total pipeline time.

**Resource Usage:**
- **CPU (%)**: Average CPU utilization of `models_node_exe` process
- **Mem (MB)**: Resident memory (RAM) usage, excludes GPU VRAM

---

## Key Findings

**Performance:**
- FP16 provides ~1.7-1.8x faster inference (29-31 FPS vs 17-19 FPS)
- All FP16 models achieve real-time performance (>29 FPS)
- Post-processing is fast: mask generation 0.23-0.76ms, visualization 9-24ms

**Resource Trade-offs:**
- FP16: Higher CPU (+30-40%) and memory (+120-140 MB) for faster inference
- FP32: Lower resource usage, suitable for 15-20 FPS applications

**Recommendations:**
- **Real-time (30 FPS)**: Use FP16
- **Resource-constrained**: Use FP32
- **High accuracy**: Use FP32

---

## Reproduce Benchmarks

```bash
# Run full benchmark suite (6 tests, 10s each)
make benchmark VIDEO=video/8358-208052058.mp4 DURATION=10

# Results saved to: benchmark_results/<timestamp>/
```

See `scripts/README.md` for detailed documentation.

## Raw Data

The results in this document are generated from:
- **CSV Summary:** [benchmark_results/20251009_161647/benchmark_results.csv](../benchmark_results/20251009_161647/benchmark_results.csv)
- **Individual Logs:** [benchmark_results/20251009_161647/](../benchmark_results/20251009_161647/)
  - `scene_seg_fp16.log`, `scene_seg_fp32.log`
  - `domain_seg_fp16.log`, `domain_seg_fp32.log`
  - `scene_3d_fp16.log`, `scene_3d_fp32.log`
