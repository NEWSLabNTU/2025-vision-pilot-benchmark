set -e

echo -e ""

BATCH=${1:-1}

cd "$(dirname "$0")"

SRC_DIR="models"

echo "Converting ONNX models to batch=${BATCH}..."

python3 - <<PY
import onnx, os

src = "."
batch_size = int("${BATCH}")

models = [
    "SceneSeg_FP32.onnx",
    "Scene3D_FP32.onnx",
    "DomainSeg_FP32.onnx",
]

for fname in models:
    in_path = os.path.join(src, fname)
    if not os.path.exists(in_path):
        print(f"File not found: {in_path}, skip.")
        continue
    model = onnx.load(in_path)
    for inp in model.graph.input:
        inp.type.tensor_type.shape.dim[0].dim_value = batch_size
    out_name = fname.replace(".onnx", f"_bs{batch_size}.onnx")
    out_path = os.path.join(src, out_name)
    onnx.save(model, out_path)
    print(f"Converted {fname} -> {out_path} (batch={batch_size})")
PY

echo -e "\e[1;36mAll models processed, new files saved in $SRC_DIR/\n\e[0m"