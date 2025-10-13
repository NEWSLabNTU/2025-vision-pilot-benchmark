#!/usr/bin/env python3
"""
Run VisionPilot benchmarks for all backend/model/precision combinations.

This script automatically tests all combinations of backends (TensorRT, ONNX Runtime),
models (scene_seg, domain_seg, scene_3d), and precisions (fp16, fp32), collecting
performance metrics including FPS, latency, CPU usage, and memory usage.
"""

import argparse
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import psutil


class Colors:
    """ANSI color codes for terminal output."""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color


class BenchmarkRunner:
    """Runs VisionPilot benchmarks across all model/precision combinations."""

    def __init__(self, video_path: Path, duration: int = 10, backends: List[str] = None,
                 tensorrt_init: int = 10, onnxruntime_init: int = 120,
                 tensorrt_duration: int = None, onnxruntime_duration: int = None):
        self.video_path = video_path.resolve()
        self.default_duration = duration
        self.backends = backends if backends else ["tensorrt"]
        self.pipelines = ["scene_seg", "domain_seg", "scene_3d"]
        self.precisions = ["fp16", "fp32"]
        self.tensorrt_init = tensorrt_init
        self.onnxruntime_init = onnxruntime_init
        # Backend-specific durations (default to 3x init period if not specified)
        self.tensorrt_duration = tensorrt_duration if tensorrt_duration is not None else duration
        self.onnxruntime_duration = onnxruntime_duration if onnxruntime_duration is not None else max(duration, 30)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_dir = Path("benchmark_results") / self.timestamp
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.summary_file = self.results_dir / "benchmark_results.csv"

    def log_info(self, msg: str) -> None:
        """Log info message."""
        print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")

    def log_success(self, msg: str) -> None:
        """Log success message."""
        print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {msg}")

    def log_warning(self, msg: str) -> None:
        """Log warning message."""
        print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {msg}")

    def log_error(self, msg: str) -> None:
        """Log error message."""
        print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")

    def run_command(self, cmd: List[str], silent: bool = False) -> Tuple[int, str, str]:
        """Run a shell command and return exit code, stdout, stderr."""
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Command timed out"
        except Exception as e:
            return -1, "", str(e)

    def generate_config(self, backend: str, precision: str) -> bool:
        """Generate config files for the specified backend and precision."""
        self.log_info(f"Generating config: BACKEND={backend} PRECISION={precision}")
        rc, _, _ = self.run_command([
            "make", "config",
            f"BACKEND={backend}",
            f"PRECISION={precision}"
        ], silent=True)
        return rc == 0

    def stop_service(self) -> None:
        """Stop any running VisionPilot services."""
        self.run_command(["make", "stop-web"], silent=True)
        time.sleep(2)

    def start_service(self, pipeline: str) -> bool:
        """Start VisionPilot service with specified pipeline."""
        self.log_info(f"Starting service: {pipeline}")
        rc, _, _ = self.run_command([
            "make", "start-web",
            f"VIDEO={self.video_path}",
            f"PIPELINE={pipeline}"
        ], silent=True)
        return rc == 0

    def get_service_pid(self, max_retries: int = 10, retry_delay: float = 1.0) -> Optional[int]:
        """Get PID of the models_node_exe process with retries.

        Args:
            max_retries: Maximum number of retries (default: 10)
            retry_delay: Delay between retries in seconds (default: 1.0)

        Returns:
            PID of the process or None if not found
        """
        for attempt in range(max_retries):
            try:
                result = subprocess.run(
                    ["pgrep", "-f", "models_node_exe"],
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0 and result.stdout.strip():
                    pid = int(result.stdout.strip().split()[0])
                    self.log_info(f"Found models_node_exe process: PID {pid}")
                    return pid
            except Exception:
                pass

            if attempt < max_retries - 1:
                time.sleep(retry_delay)

        return None

    def monitor_resources(self, duration: int, init_period: int = 10, sample_interval: float = 0.1) -> Dict[str, float]:
        """Monitor CPU and memory usage during initialization and runtime.

        Args:
            duration: Duration to monitor in seconds (runtime/benchmark period)
            init_period: Initialization period in seconds (default: 10)
            sample_interval: Sampling interval in seconds (default: 0.1 = 10 samples/sec)

        Returns:
            Dictionary with:
            - cpu_avg: Average CPU during runtime only (after init)
            - cpu_peak: Peak CPU during entire period (init + runtime)
            - mem_avg_mb: Average memory during runtime only (after init)
            - mem_peak_mb: Peak memory during entire period (init + runtime)
        """
        pid = self.get_service_pid()
        if not pid:
            self.log_warning("Could not find models_node_exe process for resource monitoring")
            return {"cpu_avg": 0.0, "cpu_peak": 0.0, "mem_avg_mb": 0.0, "mem_peak_mb": 0.0}

        try:
            process = psutil.Process(pid)
            init_cpu_samples = []
            init_mem_samples = []
            runtime_cpu_samples = []
            runtime_mem_samples = []

            # Calculate number of samples for each period
            init_samples = int(init_period / sample_interval)
            runtime_samples = int(duration / sample_interval)

            # Monitor initialization period
            self.log_info(f"Monitoring initialization period: {init_period}s at {1/sample_interval:.0f} samples/sec...")
            for i in range(init_samples):
                try:
                    cpu_percent = process.cpu_percent(interval=sample_interval)
                    mem_info = process.memory_info()
                    mem_mb = mem_info.rss / (1024 * 1024)  # Convert to MB

                    init_cpu_samples.append(cpu_percent)
                    init_mem_samples.append(mem_mb)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    break

            # Monitor runtime period
            self.log_info(f"Monitoring runtime period: {duration}s at {1/sample_interval:.0f} samples/sec...")
            for i in range(runtime_samples):
                try:
                    cpu_percent = process.cpu_percent(interval=sample_interval)
                    mem_info = process.memory_info()
                    mem_mb = mem_info.rss / (1024 * 1024)  # Convert to MB

                    runtime_cpu_samples.append(cpu_percent)
                    runtime_mem_samples.append(mem_mb)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    break

            # Calculate metrics
            # Peak CPU and memory across entire period (init + runtime)
            all_cpu_samples = init_cpu_samples + runtime_cpu_samples
            all_mem_samples = init_mem_samples + runtime_mem_samples

            cpu_peak = max(all_cpu_samples) if all_cpu_samples else 0.0
            mem_peak_mb = max(all_mem_samples) if all_mem_samples else 0.0

            # Average CPU and memory during runtime only
            if runtime_cpu_samples and runtime_mem_samples:
                cpu_avg = sum(runtime_cpu_samples) / len(runtime_cpu_samples)
                mem_avg_mb = sum(runtime_mem_samples) / len(runtime_mem_samples)
            else:
                cpu_avg = 0.0
                mem_avg_mb = 0.0

            return {
                "cpu_avg": cpu_avg,
                "cpu_peak": cpu_peak,
                "mem_avg_mb": mem_avg_mb,
                "mem_peak_mb": mem_peak_mb
            }
        except Exception as e:
            self.log_warning(f"Resource monitoring error: {e}")

        return {"cpu_avg": 0.0, "cpu_peak": 0.0, "mem_avg_mb": 0.0, "mem_peak_mb": 0.0}

    def collect_logs(self, log_file: Path, duration: int, init_period: int = 10) -> None:
        """Collect logs from systemd journal.

        Args:
            log_file: Path to save logs
            duration: Runtime duration in seconds
            init_period: Initialization period in seconds (default: 10)
        """
        try:
            # Collect logs from entire period: init + runtime + 5s buffer
            total_time = init_period + duration + 5
            result = subprocess.run(
                ["journalctl", "--user", "-u", "ros2-vision-pilot-web",
                 "--since", f"{total_time} seconds ago"],
                capture_output=True,
                text=True,
                timeout=10
            )
            with open(log_file, 'w') as f:
                f.write(result.stdout)
        except Exception as e:
            self.log_warning(f"Failed to collect logs: {e}")

    def extract_metrics(self, log_file: Path) -> Dict[str, Optional[float]]:
        """Extract performance metrics from log file."""
        metrics = {
            "fps": None,
            "latency_ms": None,
            "mask_time_ms": None,
            "mask_fps": None,
            "viz_latency_ms": None,
            "viz_fps": None
        }

        try:
            with open(log_file, 'r') as f:
                lines = f.readlines()

            # Find last 10 inference latency lines
            inference_lines = [l for l in lines if "Inference Latency" in l][-10:]

            # Find last 10 mask time lines
            mask_lines = [l for l in lines if "Get mask time" in l][-10:]

            # Find last 10 visualization latency lines
            viz_lines = [l for l in lines if "Visualization Latency" in l][-10:]

            # Extract inference metrics
            if inference_lines:
                fps_values = []
                latency_values = []

                for line in inference_lines:
                    # Extract FPS: "... (15.8 FPS)"
                    fps_match = re.search(r'\(([0-9.]+) FPS\)', line)
                    if fps_match:
                        fps_values.append(float(fps_match.group(1)))

                    # Extract latency: "Latency: 63.24 ms"
                    latency_match = re.search(r'Latency: ([0-9.]+) ms', line)
                    if latency_match:
                        latency_values.append(float(latency_match.group(1)))

                if fps_values:
                    metrics["fps"] = sum(fps_values) / len(fps_values)
                if latency_values:
                    metrics["latency_ms"] = sum(latency_values) / len(latency_values)

            # Extract mask time metrics
            if mask_lines:
                mask_values = []

                for line in mask_lines:
                    # Extract mask time: "Get mask time: 0.68 ms"
                    mask_match = re.search(r'Get mask time: ([0-9.]+) ms', line)
                    if mask_match:
                        mask_values.append(float(mask_match.group(1)))

                if mask_values:
                    avg_mask_time = sum(mask_values) / len(mask_values)
                    metrics["mask_time_ms"] = avg_mask_time
                    if avg_mask_time > 0:
                        metrics["mask_fps"] = 1000.0 / avg_mask_time

            # Extract visualization metrics
            if viz_lines:
                viz_latency_values = []
                viz_fps_values = []

                for line in viz_lines:
                    # Extract viz latency: "Visualization Latency: 21.46 ms"
                    viz_latency_match = re.search(r'Visualization Latency: ([0-9.]+) ms', line)
                    if viz_latency_match:
                        viz_latency_values.append(float(viz_latency_match.group(1)))

                    # Extract viz FPS: "... (46.6 FPS)"
                    viz_fps_match = re.search(r'\(([0-9.]+) FPS\)', line)
                    if viz_fps_match:
                        viz_fps_values.append(float(viz_fps_match.group(1)))

                if viz_latency_values:
                    metrics["viz_latency_ms"] = sum(viz_latency_values) / len(viz_latency_values)
                if viz_fps_values:
                    metrics["viz_fps"] = sum(viz_fps_values) / len(viz_fps_values)

        except Exception as e:
            self.log_warning(f"Failed to extract metrics: {e}")

        return metrics

    def run_benchmark(self, backend: str, pipeline: str, precision: str) -> Dict[str, any]:
        """Run a single benchmark test."""
        result = {
            "backend": backend,
            "pipeline": pipeline,
            "precision": precision,
            "status": "FAILED",
            "fps": None,
            "latency_ms": None,
            "mask_time_ms": None,
            "mask_fps": None,
            "viz_latency_ms": None,
            "viz_fps": None,
            "cpu_avg": None,
            "cpu_peak": None,
            "mem_avg_mb": None,
            "mem_peak_mb": None
        }

        # ONNX Runtime only supports FP32 (no FP16 ONNX models available)
        if backend == "onnxruntime" and precision == "fp16":
            self.log_warning(f"Skipping {backend}/{pipeline}/{precision}: ONNX Runtime only supports FP32")
            result["status"] = "SKIPPED"
            return result

        # Determine initialization period and runtime duration based on backend
        # ONNX Runtime needs longer init time due to CUDA provider setup
        init_period = self.onnxruntime_init if backend == "onnxruntime" else self.tensorrt_init
        duration = self.onnxruntime_duration if backend == "onnxruntime" else self.tensorrt_duration

        # Generate config
        if not self.generate_config(backend, precision):
            self.log_error("Failed to generate config")
            return result

        # Stop existing services
        self.log_info("Stopping existing services...")
        self.stop_service()

        # Start service
        if not self.start_service(pipeline):
            self.log_error("Failed to start service")
            return result

        # Monitor resources during initialization and benchmark
        # Peak CPU/memory includes init period, but averages are runtime only
        self.log_info(f"Monitoring resources: initialization ({init_period}s) + runtime ({duration}s)...")
        resource_metrics = self.monitor_resources(duration, init_period=init_period)

        # Stop service
        self.log_info("Stopping service...")
        self.stop_service()

        # Collect logs
        log_file = self.results_dir / f"{backend}_{pipeline}_{precision}.log"
        self.collect_logs(log_file, duration, init_period=init_period)

        # Extract performance metrics
        self.log_info("Extracting performance metrics...")
        perf_metrics = self.extract_metrics(log_file)

        # Combine results
        result.update({
            "status": "SUCCESS" if perf_metrics["fps"] else "NO_DATA",
            "fps": perf_metrics["fps"],
            "latency_ms": perf_metrics["latency_ms"],
            "mask_time_ms": perf_metrics["mask_time_ms"],
            "mask_fps": perf_metrics["mask_fps"],
            "viz_latency_ms": perf_metrics["viz_latency_ms"],
            "viz_fps": perf_metrics["viz_fps"],
            "cpu_avg": resource_metrics["cpu_avg"],
            "cpu_peak": resource_metrics["cpu_peak"],
            "mem_avg_mb": resource_metrics["mem_avg_mb"],
            "mem_peak_mb": resource_metrics["mem_peak_mb"]
        })

        return result

    def format_result_csv(self, result: Dict[str, any]) -> str:
        """Format a benchmark result as CSV row."""
        return (f"{result['backend']},"
               f"{result['pipeline']},"
               f"{result['precision']},"
               f"{result['status']},"
               f"{result['fps'] if result['fps'] else ''},"
               f"{result['latency_ms'] if result['latency_ms'] else ''},"
               f"{result['mask_time_ms'] if result['mask_time_ms'] else ''},"
               f"{result['mask_fps'] if result['mask_fps'] else ''},"
               f"{result['viz_latency_ms'] if result['viz_latency_ms'] else ''},"
               f"{result['viz_fps'] if result['viz_fps'] else ''},"
               f"{result['cpu_avg'] if result['cpu_avg'] else ''},"
               f"{result['cpu_peak'] if result['cpu_peak'] else ''},"
               f"{result['mem_avg_mb'] if result['mem_avg_mb'] else ''},"
               f"{result['mem_peak_mb'] if result['mem_peak_mb'] else ''}")

    def format_result_human(self, result: Dict[str, any]) -> str:
        """Format a benchmark result as human-readable string."""
        backend = result["backend"]
        pipeline = result["pipeline"]
        precision = result["precision"]

        if result["status"] == "SUCCESS" and result["fps"]:
            parts = [
                f"{backend}/{pipeline} {precision}:",
                f"Inf: {result['fps']:.1f} FPS ({result['latency_ms']:.2f} ms)"
            ]
            if result['mask_time_ms']:
                parts.append(f"Mask: {result['mask_fps']:.1f} FPS ({result['mask_time_ms']:.2f} ms)")
            if result['viz_latency_ms']:
                parts.append(f"Viz: {result['viz_fps']:.1f} FPS ({result['viz_latency_ms']:.2f} ms)")
            parts.append(f"CPU: {result['cpu_avg']:.1f}% (peak: {result['cpu_peak']:.1f}%)")
            parts.append(f"Mem: {result['mem_avg_mb']:.0f} MB (peak: {result['mem_peak_mb']:.0f} MB)")
            return " | ".join(parts)
        else:
            return f"{backend}/{pipeline} {precision}: {result['status']}"

    def write_csv_header(self) -> None:
        """Write CSV header to file."""
        with open(self.summary_file, 'w') as f:
            # Write metadata as comments
            f.write(f"# VisionPilot Benchmark Results\n")
            f.write(f"# Timestamp: {datetime.now()}\n")
            f.write(f"# Video: {self.video_path}\n")
            f.write(f"# Runtime duration: {self.tensorrt_duration}s (TensorRT), {self.onnxruntime_duration}s (ONNX Runtime)\n")
            f.write(f"# Init period: {self.tensorrt_init}s (TensorRT), {self.onnxruntime_init}s (ONNX Runtime)\n")
            f.write(f"# Note: CPU/Memory averages are runtime only; peaks include initialization\n")
            f.write(f"# Backends: {', '.join(self.backends)}\n")
            f.write("#\n")
            # Write CSV header
            f.write("backend,pipeline,precision,status,inference_fps,inference_latency_ms,")
            f.write("mask_time_ms,mask_fps,viz_latency_ms,viz_fps,cpu_avg,cpu_peak,mem_avg_mb,mem_peak_mb\n")
            f.flush()

    def append_result(self, result: Dict[str, any]) -> None:
        """Append a single result to the CSV file."""
        with open(self.summary_file, 'a') as f:
            f.write(self.format_result_csv(result) + "\n")
            f.flush()  # Ensure data is written immediately

    def write_csv_footer(self, total_completed: int, total_tests: int) -> None:
        """Write CSV footer as comment."""
        with open(self.summary_file, 'a') as f:
            f.write(f"#\n")
            f.write(f"# Completed: {total_completed}/{total_tests} tests\n")
            if total_completed < total_tests:
                f.write("# WARNING: Benchmark did not complete all tests.\n")

    def run_all(self) -> None:
        """Run benchmarks for all combinations."""
        # Validate video file
        if not self.video_path.exists():
            self.log_error(f"Video file not found: {self.video_path}")
            sys.exit(1)

        self.log_info(f"Video: {self.video_path}")
        self.log_info(f"Runtime duration: TensorRT={self.tensorrt_duration}s, ONNX Runtime={self.onnxruntime_duration}s")
        self.log_info(f"Init period: TensorRT={self.tensorrt_init}s, ONNX Runtime={self.onnxruntime_init}s")
        self.log_info(f"Backends: {', '.join(self.backends)}")
        self.log_info(f"Results will be saved to: {self.results_dir}")
        print()

        # Write CSV header
        self.write_csv_header()
        self.log_info(f"Results CSV: {self.summary_file}")
        print()

        # Run benchmarks
        total_tests = len(self.backends) * len(self.pipelines) * len(self.precisions)
        current_test = 0
        completed_tests = 0

        try:
            for backend in self.backends:
                for pipeline in self.pipelines:
                    for precision in self.precisions:
                        current_test += 1

                        self.log_info("=" * 60)
                        self.log_info(f"Test {current_test}/{total_tests}: {backend}/{pipeline} with {precision}")
                        self.log_info("=" * 60)

                        result = self.run_benchmark(backend, pipeline, precision)

                        # Write result immediately after test completes
                        self.append_result(result)
                        completed_tests += 1

                        if result["status"] == "SUCCESS":
                            self.log_success(self.format_result_human(result))
                        else:
                            self.log_warning(self.format_result_human(result))

                        self.log_info(f"Result saved to: {self.summary_file}")
                        print()

        except KeyboardInterrupt:
            self.log_warning("\nBenchmark interrupted by user")
        except Exception as e:
            self.log_error(f"Benchmark error: {e}")
        finally:
            # Cleanup
            self.log_info("Stopping all services...")
            self.stop_service()

            # Write footer
            self.write_csv_footer(completed_tests, total_tests)

            # Display summary
            self.log_success("=" * 60)
            if completed_tests == total_tests:
                self.log_success("Benchmark Complete!")
            else:
                self.log_warning(f"Benchmark Partial Complete ({completed_tests}/{total_tests} tests)")
            self.log_success("=" * 60)
            print()
            print("Results CSV:")
            with open(self.summary_file, 'r') as f:
                print(f.read())
            self.log_info(f"Full logs saved to: {self.results_dir}/")
            self.log_info(f"CSV file: {self.summary_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Run VisionPilot benchmarks for all model/precision/backend combinations'
    )
    parser.add_argument(
        'video_path',
        type=Path,
        help='Path to the video file'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=10,
        help='Default runtime duration in seconds per test (default: 10)'
    )
    parser.add_argument(
        '--backend',
        choices=['tensorrt', 'onnxruntime', 'both'],
        default='both',
        help='Backend to test: tensorrt, onnxruntime, or both (default: both)'
    )
    parser.add_argument(
        '--tensorrt-init',
        type=int,
        default=10,
        help='Initialization period in seconds for TensorRT (default: 10)'
    )
    parser.add_argument(
        '--onnxruntime-init',
        type=int,
        default=120,
        help='Initialization period in seconds for ONNX Runtime (default: 120)'
    )
    parser.add_argument(
        '--tensorrt-duration',
        type=int,
        default=None,
        help='Runtime duration in seconds for TensorRT tests (default: same as --duration)'
    )
    parser.add_argument(
        '--onnxruntime-duration',
        type=int,
        default=None,
        help='Runtime duration in seconds for ONNX Runtime tests (default: max(30, --duration))'
    )

    args = parser.parse_args()

    # Determine which backends to test
    if args.backend == 'both':
        backends = ['tensorrt', 'onnxruntime']
    else:
        backends = [args.backend]

    runner = BenchmarkRunner(
        video_path=args.video_path,
        duration=args.duration,
        backends=backends,
        tensorrt_init=args.tensorrt_init,
        onnxruntime_init=args.onnxruntime_init,
        tensorrt_duration=args.tensorrt_duration,
        onnxruntime_duration=args.onnxruntime_duration
    )
    runner.run_all()


if __name__ == '__main__':
    main()
