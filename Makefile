# VisionPilot Build System
# For Jetson Linux 36.3 with ROS2 Humble

# Paths
ANSIBLE_DIR := ansible
ROS_SETUP := /opt/ros/humble/setup.sh
VISIONPILOT_ENV := setup-env.sh
WORKSPACE_DIR := autoware-pov/VisionPilot/ROS2
MODELS_DIR := models

# Build configuration
BUILD_TYPE := Release
ONNXRUNTIME_ROOTDIR := $(shell pwd)/onnxruntime

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

.PHONY: prepare
prepare: ## Install ROS2 dependencies with rosdep
	@echo "$(BOLD)Preparing ROS2 workspace dependencies...$(RESET)"
	@if [ ! -d "$(WORKSPACE_DIR)" ]; then \
		echo "$(YELLOW)Error: $(WORKSPACE_DIR) not found$(RESET)"; \
		echo "Please clone the VisionPilot repository first"; \
		exit 1; \
	fi

	@echo "Updating rosdep..."
	@. $(ROS_SETUP) && \
	rosdep update

	@echo "Installing dependencies..."
	@cd $(WORKSPACE_DIR) && \
	. $(ROS_SETUP) && \
	rosdep install --from-paths . --ignore-src -r -y

	@echo "$(GREEN)ROS2 dependencies installed$(RESET)"

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
	colcon build \
		--symlink-install \
		--cmake-args \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DONNXRUNTIME_ROOTDIR=$(ONNXRUNTIME_ROOTDIR)

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
		. $(ROS_SETUP) && . install/setup.sh && ros2 pkg list | grep -E '(sensors|models|visualization)' || echo "Packages not built"; \
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

.PHONY: run-web-service
run-web-service: ## Run VisionPilot web service with systemd (requires VIDEO and PIPELINE)
	@if [ -z "$(VIDEO)" ]; then \
		echo "$(YELLOW)Usage: make run-web-service VIDEO=/path/to/video.mp4 PIPELINE=scene_seg$(RESET)"; \
		exit 1; \
	fi
	@echo "$(BOLD)Starting VisionPilot web service with systemd...$(RESET)"
	@cd $(WORKSPACE_DIR) && \
	. $(ROS_SETUP) && \
	. install/setup.sh && \
	ros2 systemd launch \
		--name vision-pilot-web \
		--replace \
		--description "VisionPilot Web Interface Service" \
		--source $(WORKSPACE_DIR)/install/setup.sh \
		vision_pilot_launch vision_pilot_web.launch.xml \
		video_path:=$(VIDEO) \
		pipeline:=$(or $(PIPELINE),scene_seg)
	@echo "$(GREEN)VisionPilot web service started!$(RESET)"
	@echo "Access via:"
	@echo "  - Foxglove: https://app.foxglove.dev/ (connect to ws://$(shell hostname -I | awk '{print $$1}'):8765)"
	@echo "  - Video streams: http://$(shell hostname -I | awk '{print $$1}'):8080"

.PHONY: run-simple-service
run-simple-service: ## Run simple VisionPilot service with systemd (requires VIDEO and PIPELINE)
	@if [ -z "$(VIDEO)" ]; then \
		echo "$(YELLOW)Usage: make run-simple-service VIDEO=/path/to/video.mp4 PIPELINE=scene_seg$(RESET)"; \
		exit 1; \
	fi
	@echo "$(BOLD)Starting simple VisionPilot service with systemd...$(RESET)"
	@cd $(WORKSPACE_DIR) && \
	. $(ROS_SETUP) && \
	. install/setup.sh && \
	ros2 systemd launch \
		--name vision-pilot-simple \
		--replace \
		--description "VisionPilot Simple Service" \
		--source $(WORKSPACE_DIR)/install/setup.sh \
		vision_pilot_launch vision_pilot_simple.launch.xml \
		video_path:=$(VIDEO) \
		pipeline:=$(or $(PIPELINE),scene_seg)
	@echo "$(GREEN)VisionPilot simple service started!$(RESET)"
	@echo "Access via:"
	@echo "  - Foxglove: https://app.foxglove.dev/ (connect to ws://$(shell hostname -I | awk '{print $$1}'):8765)"
	@echo "  - Video streams: http://$(shell hostname -I | awk '{print $$1}'):8080"

.PHONY: stop-vision-services
stop-vision-services: ## Stop all VisionPilot systemd services
	@echo "$(BOLD)Stopping VisionPilot services...$(RESET)"
	@-ros2 systemd stop vision-pilot-web 2>/dev/null || true
	@-ros2 systemd stop vision-pilot-simple 2>/dev/null || true
	@echo "$(GREEN)VisionPilot services stopped$(RESET)"

.PHONY: status-vision-services
status-vision-services: ## Show status of VisionPilot systemd services
	@echo "$(BOLD)VisionPilot services status:$(RESET)"
	@echo ""
	@echo "$(YELLOW)Web service:$(RESET)"
	@-ros2 systemd status vision-pilot-web 2>/dev/null || echo "  Not running"
	@echo ""
	@echo "$(YELLOW)Simple service:$(RESET)"
	@-ros2 systemd status vision-pilot-simple 2>/dev/null || echo "  Not running"
	@echo ""
	@echo "$(YELLOW)All ROS2 systemd services:$(RESET)"
	@-ros2 systemd list 2>/dev/null || echo "  None found"

.PHONY: logs-vision-services
logs-vision-services: ## Show logs for VisionPilot systemd services
	@echo "$(BOLD)VisionPilot service logs:$(RESET)"
	@echo ""
	@if [ "$(SERVICE)" ]; then \
		ros2 systemd logs $(SERVICE); \
	else \
		echo "$(YELLOW)Usage: make logs-vision-services SERVICE=vision-pilot-web$(RESET)"; \
		echo "Available services: vision-pilot-web, vision-pilot-simple"; \
	fi

.PHONY: remove-vision-services
remove-vision-services: ## Remove all VisionPilot systemd services
	@echo "$(BOLD)Removing VisionPilot services...$(RESET)"
	@-ros2 systemd remove vision-pilot-web 2>/dev/null || true
	@-ros2 systemd remove vision-pilot-simple 2>/dev/null || true
	@echo "$(GREEN)VisionPilot services removed$(RESET)"

.PHONY: rviz
rviz: ## Launch RViz2 for 3D visualization
	@echo "$(BOLD)Launching RViz2...$(RESET)"
	@if [ ! -d "$(WORKSPACE_DIR)" ]; then \
		echo "$(YELLOW)Error: $(WORKSPACE_DIR) not found$(RESET)"; \
		echo "Please build the workspace first"; \
		exit 1; \
	fi
	@cd $(WORKSPACE_DIR) && \
	. $(ROS_SETUP) && \
	. install/setup.sh && \
	rviz2

.DEFAULT_GOAL := help
