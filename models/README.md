# VisionPilot Models Downloader Using tensorRT

This folder contains the VisionPilot AI models download scripts and manifest, allowing you to automatically download required or optional models to the `models/` directory.

## Directory Structure

```
models/
├── models_setup.sh        # Main model download script
├── models_manifest.json   # JSON manifest listing all models
├── convert_bs.sh          # Script to convert ONNX models
└── (Downloaded .onnx model files)
```

---

## Dependencies

* **Python 3**
* **gdown** (to download files from Google Drive)
* **jq** (to parse JSON in shell scripts)

If missing, `download_models.sh` will attempt to install them automatically:

```bash
sudo apt install jq
pip3 install --user gdown
```

---

## Usage

1. **Ensure the JSON manifest is in the `models/` folder**
   File name must be `models_manifest.json`.

2. **Make the script executable**

3. **Download required models only (`required=true`)**

```bash
./models_setup.sh
```

4. **Download including optional models (`required=false`)**

```bash
./models_setup.sh 1
```

5. **Check download status**
   Existing files are automatically skipped; no duplicate downloads.

---

## Batch Size Conversion

Due to **tensorRT** do not accept the dynamic dimension input, we have to fix the hyperparameter of the models.

Using `convert_bs.sh`:

```bash
# Default batch=1
./convert_bs.sh

# Specify batch=2
./convert_bs.sh 2
```

Converted models will be saved back into `models/` with a `_bsX` suffix, e.g., `SceneSeg_FP32_bs2.onnx`.

---

## `auto3D.yaml`, `autoseg.yaml` setting

They are usually in the folder `/2025-vision-pilot-benchmark/autoware-pov/VisionPilot/ROS2/models/config`

- `auto3D.yaml`
    ```yaml
    scene3d_model:
        ros__parameters:
            model_path: "path/to/2025-vision-pilot-benchmark/models/Scene3D_FP32_bs1.onnx" # set the model to "_bsX" 
            backend: "tensorrt" # set the backend to "tensorrt" 
            precision: "fp32"
            model_type: "depth"
            input_topic: "/sensors/video/image_raw"
            output_topic: "/auto3d/scene_3d/depth_map"
            gpu_backend: "cuda"
    ```
- `autoseg.yaml`
    ```yaml
    scene_seg_model:
        ros__parameters:
            model_path: "/path/to/2025-vision-pilot-benchmark/models/SceneSeg_FP32_bs1.onnx" # set the model to "_bsX" 
            backend: "tensorrt" # set the backend to "tensorrt" 
            precision: "fp32"
            model_type: "segmentation"
            input_topic: "/sensors/video/image_raw"
            output_topic: "/autoseg/scene_seg/mask"
            gpu_backend: "cuda"

    domain_seg_model:
        ros__parameters:
            model_path: "/path/to/2025-vision-pilot-benchmark/models/DomainSeg_FP32_bs1.onnx" # set the model to "_bsX" 
            backend: "tensorrt" # set the backend to "tensorrt" 
            precision: "fp32"
            model_type: "segmentation"
            input_topic: "/sensors/video/image_raw"
            output_topic: "/autoseg/domain_seg/mask"
            gpu_backend: "cuda"
    ```

---

## Notes

1. Google Drive downloads require a stable internet connection; each model is approximately 185MB.
2. The download script can be safely re-run; it will skip files that already exist.
3. It is recommended to install `jq` and `gdown`, or use the automatic installation in the script.