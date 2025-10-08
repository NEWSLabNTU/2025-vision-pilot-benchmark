# Inference Latency & Visualization Performance Summary

This document summarizes the measured inference and visualization latencies  
for all segmentation and 3D scene models under **FP16** and **FP32** precision.

> **Frame range analyzed:** 100–15000  
> **Samples per log:** n = 150  
> **Environment:** Jetson Orin (TensorRT engine logs)

---

## Inference Performance

| Model | Precision | Latency (ms) | Throughput (FPS) | Mask Time (ms) |
|:------|:----------|-------------:|-----------------:|---------------:|
| **domain_seg** | FP16 | 33.73 ± 2.79 | 29.85 ± 2.45 | 0.73 ± 0.22 |
| **scene_3d** | FP16 | 33.63 ± 2.55 | 29.90 ± 2.14 | N/A |
| **scene_seg** | FP16 | 34.59 ± 2.90 | 29.12 ± 2.49 | 1.62 ± 0.50 |
| **domain_seg** | FP32 | 58.72 ± 2.94 | 17.07 ± 0.83 | 0.59 ± 0.22 |
| **scene_3d** | FP32 | 58.89 ± 3.11 | 17.03 ± 0.85 | N/A |
| **scene_seg** | FP32 | 58.61 ± 2.97 | 17.10 ± 0.83 | 1.59 ± 0.55 |

## Visualization Performance

| Model | Precision | Latency (ms) | Throughput (FPS) |
|:------|:----------|-------------:|-----------------:|
| **domain_seg** | FP16 | 8.75 ± 2.18 | 122.19 ± 33.69 |
| **scene_3d** | FP16 | 4.22 ± 1.07 | 255.67 ± 79.73 |
| **scene_seg** | FP16 | 8.76 ± 2.26 | 121.60 ± 29.83 |
| **domain_seg** | FP32 | 10.35 ± 2.53 | 102.76 ± 26.57 |
| **scene_3d** | FP32 | 4.23 ± 1.25 | 259.46 ± 84.36 |
| **scene_seg** | FP32 | 9.71 ± 1.87 | 107.10 ± 22.10 |

## CPU, GPU, Mem Utlization

| Model          | Precision | CPU Utilization | GPU Utilization |     Memory Usage |
| :------------- | :-------- | --------------: | --------------: | ---------------: |
| **domain_seg** | FP32      |       43 ~ 47 % |        **99 %** | 0.6 % (~0.38 GB) |
| **scene_seg**  | FP32      |       42 ~ 49 % |        **99 %** | 0.6 % (~0.38 GB) |
| **scene_3d**   | FP32      |       44 ~ 46 % |        **99 %** | 0.6 % (~0.38 GB) |
| **domain_seg** | FP16      |       56 ~ 60 % |        **88 %** | 0.8 % (~0.50 GB) |
| **scene_seg**  | FP16      |       57 ~ 66 % |        **74 %** | 0.8 % (~0.50 GB) |
| **scene_3d**   | FP16      |       53 ~ 56 % |        **82 %** | 0.8 % (~0.50 GB) |
