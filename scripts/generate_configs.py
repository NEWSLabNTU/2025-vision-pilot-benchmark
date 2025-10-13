#!/usr/bin/env python3
"""
Generate VisionPilot config files for benchmarking.

This script generates autoseg.yaml and auto3d.yaml config files
based on backend (tensorrt/onnxruntime) and precision (fp16/fp32) settings.
"""

import argparse
import os
import sys
from pathlib import Path


def generate_config(template_path: Path, output_path: Path, models_dir: Path,
                   backend: str, precision: str) -> None:
    """Generate a config file from template."""

    # Read template
    with open(template_path, 'r') as f:
        content = f.read()

    # For TensorRT with FP16, use .fp16.engine files
    # For TensorRT with FP32, use .fp32.engine files (fallback to ONNX if not exists)
    # For ONNX Runtime, always use .onnx files (only FP32 available)
    if backend == 'tensorrt':
        if precision == 'fp16':
            # Replace model paths for TensorRT FP16 engines
            content = content.replace('/SceneSeg_FP32.onnx', '/SceneSeg_FP32.onnx.fp16.engine')
            content = content.replace('/DomainSeg_FP32.onnx', '/DomainSeg_FP32.onnx.fp16.engine')
            content = content.replace('/Scene3D_FP32.onnx', '/Scene3D_FP32.onnx.fp16.engine')
        else:  # fp32
            # Replace model paths for TensorRT FP32 engines
            content = content.replace('/SceneSeg_FP32.onnx', '/SceneSeg_FP32.onnx.fp32.engine')
            content = content.replace('/DomainSeg_FP32.onnx', '/DomainSeg_FP32.onnx.fp32.engine')
            content = content.replace('/Scene3D_FP32.onnx', '/Scene3D_FP32.onnx.fp32.engine')
    # For ONNX Runtime, keep .onnx files (already in template)

    # Substitute variables
    content = content.replace('{MODEL_PATH}', str(models_dir))
    content = content.replace('{BACKEND}', backend)
    content = content.replace('{PRECISION}', precision)

    # Write output
    with open(output_path, 'w') as f:
        f.write(content)

    print(f"Generated: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate VisionPilot config files for benchmarking'
    )
    parser.add_argument(
        '--backend',
        choices=['tensorrt', 'onnxruntime'],
        default='tensorrt',
        help='Backend to use (tensorrt or onnxruntime)'
    )
    parser.add_argument(
        '--precision',
        choices=['fp16', 'fp32'],
        default='fp32',
        help='Precision to use (fp16 or fp32)'
    )
    parser.add_argument(
        '--models-dir',
        type=Path,
        required=True,
        help='Absolute path to models directory'
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        required=True,
        help='Output directory for generated config files'
    )
    parser.add_argument(
        '--templates-dir',
        type=Path,
        required=True,
        help='Directory containing config templates'
    )

    args = parser.parse_args()

    # Validate paths
    if not args.models_dir.exists():
        print(f"Error: Models directory does not exist: {args.models_dir}", file=sys.stderr)
        sys.exit(1)

    if not args.templates_dir.exists():
        print(f"Error: Templates directory does not exist: {args.templates_dir}", file=sys.stderr)
        sys.exit(1)

    # Create output directory if it doesn't exist
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Generate configs
    templates = [
        ('autoseg.yaml.template', 'autoseg.yaml'),
        ('auto3d.yaml.template', 'auto3d.yaml')
    ]

    print(f"Generating configs with backend={args.backend}, precision={args.precision}")

    for template_name, output_name in templates:
        template_path = args.templates_dir / template_name
        output_path = args.output_dir / output_name

        if not template_path.exists():
            print(f"Warning: Template not found: {template_path}", file=sys.stderr)
            continue

        generate_config(
            template_path=template_path,
            output_path=output_path,
            models_dir=args.models_dir,
            backend=args.backend,
            precision=args.precision
        )


if __name__ == '__main__':
    main()
