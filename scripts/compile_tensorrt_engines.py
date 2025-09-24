#!/usr/bin/env python3
"""
TensorRT Engine Compiler for VisionPilot
Compiles ONNX models to TensorRT engines for optimized inference on Jetson
"""
import os
import sys
import subprocess
import argparse
import json
from pathlib import Path
from typing import List, Dict, Any

def check_tensorrt_tools():
    """Check if TensorRT tools are available"""
    try:
        result = subprocess.run(['trtexec', '--help'],
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            return True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    print("ERROR: trtexec not found. Please install TensorRT.")
    print("On Jetson, TensorRT should be pre-installed.")
    print("Check if /usr/src/tensorrt/bin/trtexec exists")
    return False

def get_jetson_info():
    """Get Jetson device information for TensorRT optimization"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip('\x00')

        # Determine compute capability
        if 'AGX Orin' in model:
            compute_capability = '8.7'
            max_batch_size = 8
        elif 'Orin NX' in model or 'Orin Nano' in model:
            compute_capability = '8.7'
            max_batch_size = 4
        elif 'AGX Xavier' in model:
            compute_capability = '7.2'
            max_batch_size = 4
        elif 'Xavier NX' in model:
            compute_capability = '7.2'
            max_batch_size = 2
        else:
            print(f"WARNING: Unknown Jetson model: {model}")
            compute_capability = '8.7'  # Default to latest
            max_batch_size = 4

        return {
            'model': model,
            'compute_capability': compute_capability,
            'max_batch_size': max_batch_size
        }
    except Exception as e:
        print(f"Warning: Could not detect Jetson model: {e}")
        return {
            'model': 'Unknown Jetson',
            'compute_capability': '8.7',
            'max_batch_size': 4
        }

def compile_onnx_to_engine(onnx_path: Path, engine_path: Path,
                          precision: str = 'fp16',
                          max_batch_size: int = 4,
                          workspace_size: int = 4096) -> bool:
    """Compile ONNX model to TensorRT engine"""

    if engine_path.exists():
        print(f"Engine already exists: {engine_path}")
        return True

    print(f"Compiling {onnx_path.name} to TensorRT engine...")
    print(f"  Precision: {precision}")
    print(f"  Max batch size: {max_batch_size}")
    print(f"  Workspace size: {workspace_size}MB")

    cmd = [
        'trtexec',
        f'--onnx={onnx_path}',
        f'--saveEngine={engine_path}',
        f'--memPoolSize=workspace:{workspace_size}',
        '--noTF32',  # Disable TF32 for better compatibility
        '--skipInference'  # Skip inference testing to speed up compilation
    ]

    # Add precision flags
    if precision == 'fp16':
        cmd.append('--fp16')
    elif precision == 'int8':
        cmd.append('--int8')
        # Note: INT8 requires calibration dataset which we don't have here
        print("WARNING: INT8 precision requires calibration dataset")

    # Add batch size optimization (use fixed batch size to avoid reshape conflicts)
    cmd.extend([
        f'--minShapes=input:1x3x320x640',
        f'--optShapes=input:1x3x320x640',
        f'--maxShapes=input:1x3x320x640'
    ])

    try:
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)  # 30 min timeout

        if result.returncode == 0:
            if engine_path.exists():
                size_mb = engine_path.stat().st_size / (1024 * 1024)
                print(f"✓ Successfully compiled engine ({size_mb:.1f}MB): {engine_path}")
                return True
            else:
                print(f"✗ Engine compilation completed but file not found: {engine_path}")
                return False
        else:
            print(f"✗ Engine compilation failed:")
            print(f"STDOUT: {result.stdout}")
            print(f"STDERR: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print(f"✗ Engine compilation timed out after 30 minutes")
        return False
    except Exception as e:
        print(f"✗ Engine compilation error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Compile ONNX models to TensorRT engines')
    parser.add_argument('--models-dir', default='/opt/visionpilot/models',
                      help='Directory containing ONNX models')
    parser.add_argument('--precision', choices=['fp32', 'fp16', 'int8'], default='fp16',
                      help='TensorRT precision mode')
    parser.add_argument('--workspace', type=int, default=4096,
                      help='TensorRT workspace size in MB')
    parser.add_argument('--models', nargs='+',
                      help='Specific ONNX models to compile')
    parser.add_argument('--force', action='store_true',
                      help='Force recompilation even if engine exists')
    parser.add_argument('--dry-run', action='store_true',
                      help='Show what would be compiled without actually doing it')

    args = parser.parse_args()

    if not check_tensorrt_tools():
        sys.exit(1)

    models_dir = Path(args.models_dir)
    if not models_dir.exists():
        print(f"ERROR: Models directory not found: {models_dir}")
        sys.exit(1)

    # Get Jetson device info
    jetson_info = get_jetson_info()
    print(f"Detected: {jetson_info['model']}")
    print(f"Compute Capability: {jetson_info['compute_capability']}")
    print(f"Max Batch Size: {jetson_info['max_batch_size']}")

    # Find ONNX models
    if args.models:
        onnx_files = [models_dir / name for name in args.models if name.endswith('.onnx')]
        onnx_files = [f for f in onnx_files if f.exists()]
    else:
        onnx_files = list(models_dir.glob("*.onnx"))

    if not onnx_files:
        print("No ONNX models found to compile")
        return

    print(f"\nFound {len(onnx_files)} ONNX models:")
    for f in onnx_files:
        size_mb = f.stat().st_size / (1024 * 1024)
        print(f"  {f.name}: {size_mb:.1f}MB")

    if args.dry_run:
        print("\nDry run mode - showing what would be compiled:")

    compiled_count = 0
    failed_count = 0

    for onnx_path in onnx_files:
        # Generate engine filename
        engine_name = f"{onnx_path.stem}.{args.precision}.engine"
        engine_path = models_dir / engine_name

        if engine_path.exists() and not args.force:
            print(f"\n✓ Engine exists (use --force to recompile): {engine_name}")
            continue

        if args.dry_run:
            print(f"\nWould compile: {onnx_path.name} -> {engine_name}")
            continue

        print(f"\n{'='*60}")
        success = compile_onnx_to_engine(
            onnx_path=onnx_path,
            engine_path=engine_path,
            precision=args.precision,
            max_batch_size=jetson_info['max_batch_size'],
            workspace_size=args.workspace
        )

        if success:
            compiled_count += 1
        else:
            failed_count += 1

    if not args.dry_run:
        print(f"\n{'='*60}")
        print("COMPILATION SUMMARY")
        print(f"{'='*60}")
        print(f"Successfully compiled: {compiled_count}")
        print(f"Failed: {failed_count}")

        if failed_count > 0:
            print(f"\n⚠ {failed_count} models failed to compile")
            print("This may be due to:")
            print("- Insufficient memory (try smaller --workspace)")
            print("- Unsupported ONNX operators")
            print("- TensorRT version compatibility")

        # List all engines
        engine_files = list(models_dir.glob("*.engine"))
        if engine_files:
            print(f"\nAvailable TensorRT engines:")
            total_size = 0
            for engine in engine_files:
                size_mb = engine.stat().st_size / (1024 * 1024)
                total_size += size_mb
                print(f"  {engine.name}: {size_mb:.1f}MB")
            print(f"Total engine size: {total_size:.1f}MB")

    print(f"\n🚀 TensorRT compilation complete!")

if __name__ == "__main__":
    main()