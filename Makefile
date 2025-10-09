# VisionPilot Build System for Jetson Linux 36.3 with ROS2 Humble
# Variables: VIDEO (video file path), PIPELINE (scene_seg/domain_seg/scene_3d)

ANSIBLE_DIR := ansible
ROS_SETUP := /opt/ros/humble/setup.sh
WORKSPACE_DIR := autoware-pov/VisionPilot/ROS2
MODELS_DIR := models
BUILD_TYPE := Release
ONNXRUNTIME_ROOTDIR := $(shell pwd)/onnxruntime
PIPELINE ?= scene_seg

# Convert VIDEO to absolute path
ifneq ($(VIDEO),)
VIDEO_ABS := $(abspath $(VIDEO))
else
VIDEO_ABS :=
endif

.PHONY: help
help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-25s %s\n", $$1, $$2}'

.PHONY: setup
setup: ## Install prerequisites using Ansible
	@cd $(ANSIBLE_DIR) && ansible-playbook -i inventory.ini site.yml --ask-become-pass

.PHONY: prepare
prepare: ## Install ROS2 dependencies with rosdep
	@. $(ROS_SETUP) && rosdep update && \
	cd $(WORKSPACE_DIR) && rosdep install --from-paths . --ignore-src -r -y

.PHONY: build
build: ## Build ROS2 packages
	@cd $(WORKSPACE_DIR) && . $(ROS_SETUP) && \
	colcon build --symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DONNXRUNTIME_ROOTDIR=$(ONNXRUNTIME_ROOTDIR)

.PHONY: clean
clean: ## Remove build artifacts
	@rm -rf $(WORKSPACE_DIR)/{build,install,log}
	@find $(WORKSPACE_DIR) -type f -name "*.pyc" -delete 2>/dev/null || true
	@find $(WORKSPACE_DIR) -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true

.PHONY: test
test: ## Run basic tests
	@echo "CUDA: $$(nvcc --version 2>/dev/null | grep release || echo 'Not found')"
	@echo "OpenCV: $$(dpkg -l | grep libopencv-dev | awk '{print $$3}' || echo 'Not found')"
	@echo "ONNX Runtime: $$(python3 -c 'import onnxruntime as ort; print(ort.__version__)' 2>/dev/null || echo 'Not found')"

.PHONY: run-pipeline
run-pipeline: ## Run pipeline (VIDEO=path PIPELINE=scene_seg/domain_seg/scene_3d)
	@[ "$(VIDEO)" ] || { echo "Usage: make run-pipeline VIDEO=<path> PIPELINE=<type>"; exit 1; }
	@[ "$(PIPELINE)" ] || { echo "Usage: make run-pipeline VIDEO=<path> PIPELINE=<type>"; exit 1; }
	@cd $(WORKSPACE_DIR) && . $(ROS_SETUP) && . install/setup.sh && \
		ros2 launch models run_pipeline.launch.py pipeline:=$(PIPELINE) video_path:=$(VIDEO_ABS)

.PHONY: start-web
start-web: ## Start web service (VIDEO=path [PIPELINE=scene_seg])
	@[ "$(VIDEO)" ] || { echo "Usage: make start-web VIDEO=<path> [PIPELINE=scene_seg]"; exit 1; }
	@cd $(WORKSPACE_DIR) && . $(ROS_SETUP) && . install/setup.sh && \
	ros2 systemd launch --name vision-pilot-web --replace \
		--description "VisionPilot Web Interface" \
		--source $(WORKSPACE_DIR)/install/setup.sh \
		vision_pilot_launch vision_pilot_web.launch.xml \
		video_path:=$(VIDEO_ABS) pipeline:=$(PIPELINE)
	@echo "Foxglove: ws://$$(hostname -I | awk '{print $$1}'):8765"
	@echo "Video: http://$$(hostname -I | awk '{print $$1}'):8080"

.PHONY: stop-web
stop-web: ## Stop web service
	@-ros2 systemd stop vision-pilot-web 2>/dev/null

.PHONY: status-web
status-web: ## Show web service status
	@ros2 systemd status vision-pilot-web 2>/dev/null || echo "Not running"

.PHONY: logs-web
logs-web: ## Show web service logs
	@ros2 systemd logs vision-pilot-web

.PHONY: remove-web
remove-web: ## Remove web service
	@-ros2 systemd remove vision-pilot-web 2>/dev/null

.PHONY: start-simple
start-simple: ## Start simple service (VIDEO=path [PIPELINE=scene_seg])
	@[ "$(VIDEO)" ] || { echo "Usage: make start-simple VIDEO=<path> [PIPELINE=scene_seg]"; exit 1; }
	@cd $(WORKSPACE_DIR) && . $(ROS_SETUP) && . install/setup.sh && \
	ros2 systemd launch --name vision-pilot-simple --replace \
		--description "VisionPilot Simple Service" \
		--source $(WORKSPACE_DIR)/install/setup.sh \
		vision_pilot_launch vision_pilot_simple.launch.xml \
		video_path:=$(VIDEO_ABS) pipeline:=$(PIPELINE)

.PHONY: stop-simple
stop-simple: ## Stop simple service
	@-ros2 systemd stop vision-pilot-simple 2>/dev/null

.PHONY: status-simple
status-simple: ## Show simple service status
	@ros2 systemd status vision-pilot-simple 2>/dev/null || echo "Not running"

.PHONY: logs-simple
logs-simple: ## Show simple service logs
	@ros2 systemd logs vision-pilot-simple

.PHONY: remove-simple
remove-simple: ## Remove simple service
	@-ros2 systemd remove vision-pilot-simple 2>/dev/null

.PHONY: fix-onnxruntime
fix-onnxruntime: ## Fix onnxruntime capi_dir symlink
	@echo "Finding onnxruntime 1.19.0 installation..."
	@ONNX_PATH=$$(python3 -c "import onnxruntime as ort; import os; print(os.path.dirname(ort.__file__))" 2>/dev/null) && \
	ONNX_VER=$$(python3 -c "import onnxruntime as ort; print(ort.__version__)" 2>/dev/null) && \
	if [ -z "$$ONNX_PATH" ]; then \
		echo "Error: onnxruntime not found"; exit 1; \
	fi && \
	if [ "$$ONNX_VER" != "1.19.0" ]; then \
		echo "Warning: Found onnxruntime $$ONNX_VER (expected 1.19.0)"; \
	fi && \
	if [ ! -d "$$ONNX_PATH/capi" ]; then \
		echo "Error: capi directory not found at $$ONNX_PATH/capi"; exit 1; \
	fi && \
	mkdir -p onnxruntime && \
	ln -sf "$$ONNX_PATH/capi" onnxruntime/capi_dir && \
	echo "Symlink created: onnxruntime/capi_dir -> $$ONNX_PATH/capi"

.DEFAULT_GOAL := help
