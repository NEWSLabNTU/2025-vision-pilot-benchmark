# VisionPilot Build System
# For Jetson Linux 36.3 with ROS2 Humble

# Paths
ANSIBLE_DIR := ansible
ROS_SETUP := /opt/ros/humble/setup.sh
VISIONPILOT_ENV := setup-env.sh
WORKSPACE_DIR := autoware-pov/VisionPilot/ROS2
MODELS_DIR := models

# Build configuration
PACKAGES := sensors models visualization
BUILD_TYPE := Release

# Colors for output
RESET := \033[0m
BOLD := \033[1m
GREEN := \033[32m
YELLOW := \033[33m
BLUE := \033[34m

.PHONY: help
help: ## Show this help message
	@echo "$(BOLD)VisionPilot Build System$(RESET)"
	@echo ""
	@echo "$(BOLD)Usage:$(RESET)"
	@echo "  make [target]"
	@echo ""
	@echo "$(BOLD)Targets:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-15s$(RESET) %s\n", $$1, $$2}'
	@echo ""
	@echo "$(BOLD)Examples:$(RESET)"
	@echo "  make setup      # Install prerequisites"
	@echo "  make build      # Build ROS2 packages"
	@echo "  make clean      # Clean build artifacts"

.PHONY: setup
setup: install-ansible ## Install prerequisites using Ansible
	@echo "$(BOLD)Installing VisionPilot prerequisites...$(RESET)"
	@if [ ! -f /usr/bin/ansible-playbook ]; then \
		echo "Installing Ansible..."; \
		sudo apt update && sudo apt install -y ansible; \
	fi
	@cd $(ANSIBLE_DIR) && ansible-playbook -i inventory.ini site.yml --ask-become-pass
	@echo "$(GREEN)Setup complete. Source environment: source $(VISIONPILOT_ENV)$(RESET)"

.PHONY: install-ansible
install-ansible: ## Install Ansible if not present
	@if ! command -v ansible-playbook >/dev/null 2>&1; then \
		echo "Installing Ansible..."; \
		sudo apt update && sudo apt install -y ansible; \
	else \
		echo "Ansible already installed"; \
	fi

.PHONY: build
build: ## Build ROS2 packages
	@echo "$(BOLD)Building VisionPilot ROS2 packages...$(RESET)"
	@if [ ! -d "$(WORKSPACE_DIR)" ]; then \
		echo "$(YELLOW)Error: $(WORKSPACE_DIR) not found$(RESET)"; \
		echo "Please clone the VisionPilot repository first"; \
		exit 1; \
	fi
	@cd $(WORKSPACE_DIR) && \
		. $(ROS_SETUP) && \
		colcon build --packages-select $(PACKAGES) \
			--cmake-args \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	@echo "$(GREEN)Build complete$(RESET)"
	@echo "Source the workspace: cd $(WORKSPACE_DIR) && source install/setup.sh"

.PHONY: clean
clean: ## Remove build artifacts
	@echo "$(BOLD)Cleaning build artifacts...$(RESET)"
	@if [ -d "$(WORKSPACE_DIR)/build" ]; then \
		rm -rf $(WORKSPACE_DIR)/build; \
		echo "Removed build directory"; \
	fi
	@if [ -d "$(WORKSPACE_DIR)/install" ]; then \
		rm -rf $(WORKSPACE_DIR)/install; \
		echo "Removed install directory"; \
	fi
	@if [ -d "$(WORKSPACE_DIR)/log" ]; then \
		rm -rf $(WORKSPACE_DIR)/log; \
		echo "Removed log directory"; \
	fi
	@find $(WORKSPACE_DIR) -name "*.pyc" -delete 2>/dev/null || true
	@find $(WORKSPACE_DIR) -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo "$(GREEN)Clean complete$(RESET)"

.PHONY: validate
validate: ## Validate installation
	@echo "$(BOLD)Validating installation...$(RESET)"
	@cd $(ANSIBLE_DIR) && ansible-playbook -i inventory.ini validate_installation.yml --ask-become-pass
	@if [ -f "validation_report.txt" ]; then \
		cat validation_report.txt; \
	fi

.PHONY: test
test: ## Run basic tests
	@echo "$(BOLD)Running tests...$(RESET)"
	@echo ""
	@echo "1. Checking CUDA..."
	@nvcc --version | grep release || echo "CUDA not found"
	@echo ""
	@echo "2. Checking OpenCV..."
	@dpkg -l | grep libopencv-dev | awk '{print "OpenCV version: " $$3}' || echo "OpenCV not found"
	@echo ""
	@echo "3. Checking ROS2 packages..."
	@if [ -d "$(WORKSPACE_DIR)/install" ]; then \
		cd $(WORKSPACE_DIR) && \
		bash -c ". $(ROS_SETUP) && . install/setup.sh && ros2 pkg list | grep -E '(sensors|models|visualization)'" || echo "Packages not built"; \
	else \
		echo "Packages not built"; \
	fi
	@echo ""
	@echo "4. Checking models..."
	@ls -lh $(MODELS_DIR)/*.onnx 2>/dev/null || echo "Models not found"
	@echo ""
	@echo "5. Checking ONNX Runtime GPU..."
	@python3 -c 'import onnxruntime as ort; print(f"ONNX Runtime {ort.__version__} with providers: {ort.get_available_providers()}")' 2>/dev/null || echo "ONNX Runtime GPU not found"

.PHONY: download-models
download-models: ## Download AI models only
	@echo "$(BOLD)Downloading AI models...$(RESET)"
	@cd $(ANSIBLE_DIR) && ansible-playbook -i inventory.ini models_download.yml --ask-become-pass

.PHONY: compile-tensorrt
compile-tensorrt: ## Compile ONNX models to TensorRT engines
	@echo "$(BOLD)Compiling TensorRT engines...$(RESET)"
	@cd $(ANSIBLE_DIR) && ansible-playbook -i inventory.ini compile_tensorrt.yml --ask-become-pass

.PHONY: tensorrt-engines
tensorrt-engines: compile-tensorrt ## Alias for compile-tensorrt

.PHONY: check-deps
check-deps: ## Check if all dependencies are installed
	@echo "$(BOLD)Checking dependencies...$(RESET)"
	@echo -n "CUDA: "; nvcc --version >/dev/null 2>&1 && echo "OK" || echo "NOT FOUND"
	@echo -n "ROS2 Humble: "; [ -f $(ROS_SETUP) ] && echo "OK" || echo "NOT FOUND"
	@echo -n "OpenCV: "; dpkg -l | grep -q libopencv-dev && echo "OK" || echo "NOT FOUND"
	@echo -n "Ansible: "; command -v ansible >/dev/null 2>&1 && echo "OK" || echo "NOT FOUND"
	@echo -n "Colcon: "; command -v colcon >/dev/null 2>&1 && echo "OK" || echo "NOT FOUND"

.PHONY: run-scene-seg
run-scene-seg: ## Run scene segmentation pipeline (requires video path)
	@if [ -z "$(VIDEO)" ]; then \
		echo "$(YELLOW)Usage: make run-scene-seg VIDEO=/path/to/video.mp4$(RESET)"; \
		exit 1; \
	fi
	@cd $(WORKSPACE_DIR) && \
		. $(ROS_SETUP) && \
		. install/setup.sh && \
		ros2 launch models run_pipeline.launch.py pipeline:=scene_seg video_path:=$(VIDEO)

.PHONY: run-depth
run-depth: ## Run depth estimation pipeline (requires video path)
	@if [ -z "$(VIDEO)" ]; then \
		echo "$(YELLOW)Usage: make run-depth VIDEO=/path/to/video.mp4$(RESET)"; \
		exit 1; \
	fi
	@cd $(WORKSPACE_DIR) && \
		. $(ROS_SETUP) && \
		. install/setup.sh && \
		ros2 launch models run_pipeline.launch.py pipeline:=scene_3d video_path:=$(VIDEO)

.PHONY: list-topics
list-topics: ## List ROS2 topics
	@cd $(WORKSPACE_DIR) && \
		. $(ROS_SETUP) && \
		. install/setup.sh 2>/dev/null && \
		ros2 topic list | grep -E "(autoseg|auto3d|sensors)" || echo "No topics found"

.DEFAULT_GOAL := help
