#!/usr/bin/env python3
"""
VisionPilot Model Downloader
Downloads AI models from Google Drive with validation and checksum support
"""
import json
import os
import sys
import hashlib
import argparse
from pathlib import Path
from typing import List, Dict, Any

try:
    import gdown
except ImportError:
    print("Error: gdown not installed. Run: pip install gdown==5.2.0")
    sys.exit(1)

def calculate_file_hash(filepath: Path, algorithm: str = 'sha256') -> str:
    """Calculate file hash with progress indication for large files"""
    hash_func = getattr(hashlib, algorithm)()
    file_size = filepath.stat().st_size
    processed = 0

    with open(filepath, 'rb') as f:
        while chunk := f.read(4096):
            hash_func.update(chunk)
            processed += len(chunk)
            if file_size > 50 * 1024 * 1024:  # Show progress for files > 50MB
                progress = (processed / file_size) * 100
                print(f"\rCalculating {algorithm.upper()}: {progress:.1f}%", end='', flush=True)

    if file_size > 50 * 1024 * 1024:
        print()  # New line after progress

    return hash_func.hexdigest()

def validate_existing_file(filepath: Path, expected_size_mb: int, checksum_file: Path = None) -> bool:
    """Validate existing file by size and checksum if available"""
    if not filepath.exists():
        return False

    # Size validation
    actual_size_mb = filepath.stat().st_size / (1024 * 1024)
    if expected_size_mb > 0:
        size_diff_percent = abs(actual_size_mb - expected_size_mb) / expected_size_mb * 100
        if size_diff_percent > 15:  # Allow 15% variance for size check
            print(f"Size mismatch: {actual_size_mb:.1f}MB vs expected {expected_size_mb}MB")
            return False

    # Checksum validation if available
    if checksum_file and checksum_file.exists():
        try:
            with open(checksum_file, 'r') as f:
                stored_checksum = f.read().strip().split()[0]

            print("Validating existing file checksum...")
            actual_checksum = calculate_file_hash(filepath, 'sha256')

            if stored_checksum == actual_checksum:
                print("Checksum validation passed")
                return True
            else:
                print("Checksum validation failed - will re-download")
                return False
        except Exception as e:
            print(f"Checksum validation error: {e}")
            return False

    return True

def download_single_model(model: Dict[str, Any], models_dir: Path, retry_count: int = 3) -> bool:
    """Download a single model with validation and retry logic"""
    filename = model['name']
    file_id = model['file_id']
    description = model['description']
    required = model.get('required', True)
    expected_size_mb = model.get('size_mb', 0)
    category = model.get('category', 'unknown')
    format_type = model.get('format', 'unknown')

    filepath = models_dir / filename
    checksum_file = filepath.with_suffix(filepath.suffix + '.sha256')

    print(f"\n{'='*80}")
    print(f"Model: {filename}")
    print(f"Description: {description}")
    print(f"Category: {category} | Format: {format_type}")
    print(f"Required: {'Yes' if required else 'No'}")
    print(f"Expected size: ~{expected_size_mb}MB")
    print(f"{'='*80}")

    # Check if file exists and is valid
    if validate_existing_file(filepath, expected_size_mb, checksum_file):
        file_size_mb = filepath.stat().st_size / (1024 * 1024)
        print(f"✓ Valid file exists ({file_size_mb:.1f}MB) - skipping download")
        return True

    # Download the file
    url = f"https://drive.google.com/uc?id={file_id}"

    for attempt in range(retry_count):
        try:
            print(f"Downloading (attempt {attempt + 1}/{retry_count})...")

            # Remove partial/corrupted file
            if filepath.exists():
                filepath.unlink()

            output_path = gdown.download(url, str(filepath), quiet=False)

            if output_path and Path(output_path).exists():
                # Validate download
                actual_size_mb = filepath.stat().st_size / (1024 * 1024)
                print(f"Download complete ({actual_size_mb:.1f}MB)")

                # Generate and save checksum
                print("Generating checksum...")
                checksum = calculate_file_hash(filepath, 'sha256')
                print(f"SHA256: {checksum}")

                with open(checksum_file, 'w') as f:
                    f.write(f"{checksum}  {filename}\n")

                print("✓ Download successful")
                return True

        except Exception as e:
            print(f"✗ Attempt {attempt + 1} failed: {e}")
            if attempt < retry_count - 1:
                print("Retrying in 3 seconds...")
                import time
                time.sleep(3)

    # All attempts failed
    error_msg = f"Failed to download {filename} after {retry_count} attempts"
    if required:
        print(f"✗ ERROR: {error_msg}")
        return False
    else:
        print(f"⚠ WARNING: {error_msg} (optional model)")
        return True

def filter_models(models: List[Dict], args) -> List[Dict]:
    """Filter models based on command line arguments"""
    filtered = models.copy()

    if args.required_only:
        filtered = [m for m in filtered if m.get('required', True)]

    if args.category:
        filtered = [m for m in filtered if m.get('category') in args.category]

    if args.format:
        filtered = [m for m in filtered if m.get('format') in args.format]

    if args.models:
        filtered = [m for m in filtered if m['name'] in args.models]

    return filtered

def main():
    parser = argparse.ArgumentParser(description='Download VisionPilot AI models')
    parser.add_argument('--required-only', action='store_true',
                      help='Download only required models')
    parser.add_argument('--category', nargs='+',
                      choices=['segmentation', 'depth'],
                      help='Filter by category')
    parser.add_argument('--format', nargs='+',
                      choices=['onnx_fp32', 'onnx_int8', 'pytorch_weights', 'pytorch_traced'],
                      help='Filter by format')
    parser.add_argument('--models', nargs='+',
                      help='Specific model names to download')
    parser.add_argument('--manifest', default='models/models_manifest.json',
                      help='Models manifest file')
    parser.add_argument('--retry', type=int, default=3,
                      help='Number of retry attempts per model')

    args = parser.parse_args()

    # Use environment variable or default to local models directory
    models_dir = Path(os.environ.get("VISIONPILOT_MODELS_DIR", "models"))
    manifest_file = Path(args.manifest)

    print("VisionPilot Model Downloader")
    print("=" * 80)
    print(f"Models directory: {models_dir}")
    print(f"gdown version: {gdown.__version__}")
    print(f"Manifest: {manifest_file}")

    if not manifest_file.exists():
        print(f"ERROR: Models manifest not found: {manifest_file}")
        sys.exit(1)

    # Load models manifest
    with open(manifest_file, 'r') as f:
        manifest = json.load(f)

    all_models = manifest.get('models', [])
    if not all_models:
        print("No models found in manifest")
        return

    # Filter models based on arguments
    models_to_download = filter_models(all_models, args)

    if not models_to_download:
        print("No models match the specified criteria")
        return

    print(f"Total models in manifest: {len(all_models)}")
    print(f"Models to download: {len(models_to_download)}")

    # Download models
    failed_required = []
    successful_downloads = []

    for i, model in enumerate(models_to_download, 1):
        print(f"\n[{i}/{len(models_to_download)}] Processing {model['name']}...")

        if download_single_model(model, models_dir, args.retry):
            successful_downloads.append(model['name'])
        elif model.get('required', True):
            failed_required.append(model['name'])

    # Final report
    print(f"\n{'='*80}")
    print("DOWNLOAD SUMMARY")
    print(f"{'='*80}")

    downloaded_files = list(models_dir.glob("*.onnx")) + list(models_dir.glob("*.pth")) + list(models_dir.glob("*.pt"))
    if downloaded_files:
        total_size = sum(f.stat().st_size for f in downloaded_files) / (1024 * 1024)
        print(f"\nDownloaded files ({len(downloaded_files)}):")
        for f in sorted(downloaded_files):
            size_mb = f.stat().st_size / (1024 * 1024)
            print(f"  ✓ {f.name}: {size_mb:.1f}MB")
        print(f"\nTotal size: {total_size:.1f}MB")

    if successful_downloads:
        print(f"\n✓ Successfully processed: {len(successful_downloads)} models")

    if failed_required:
        print(f"\n✗ Failed required models: {', '.join(failed_required)}")
        sys.exit(1)

    print(f"\n🎉 All required models downloaded successfully!")

if __name__ == "__main__":
    main()