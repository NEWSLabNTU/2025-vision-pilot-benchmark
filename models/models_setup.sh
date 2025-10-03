#!/bin/bash
set -e

echo -e ""

cd "$(dirname "$0")"

MANIFEST="models_manifest.json"

DOWNLOAD_OPTIONAL=${1:-0}  # According to the "required" in "models_manifest.json"

if ! command -v jq >/dev/null 2>&1; then
    echo "⚡ jq not found, installing..."
    sudo apt update
    sudo apt install -y jq
fi

if ! command -v gdown >/dev/null 2>&1; then
    echo "⚡ gdown not found, installing..."
    pip3 install --user gdown
    export PATH="$HOME/.local/bin:$PATH"
fi

echo "----> Downloading models to $(pwd) ..."

jq -c '.models[]' "$MANIFEST" | while read -r model; do
    NAME=$(echo "$model" | jq -r '.name')
    FILE_ID=$(echo "$model" | jq -r '.file_id')
    REQUIRED=$(echo "$model" | jq -r '.required')

    if [ "$REQUIRED" = "false" ] && [ "$DOWNLOAD_OPTIONAL" -eq 0 ]; then
        echo "- Skipping optional model $NAME"
        continue
    fi

    if [ -f "$NAME" ]; then
        echo "- File already exists, skipping $NAME"
        continue
    fi

    echo "⬇Downloading $NAME ..."
    gdown "https://drive.google.com/uc?id=$FILE_ID" -O "$NAME"
done

echo -e "\e[1;36mAll requested models downloaded to $(pwd)/\n\e[0m"
