# List of examples (without .c extension)
EXAMPLES =  imu motor_sw_pwm rgb_led ultrasonic button_rgb main demo
 
# QNX target configuration
QNX_TARGET_USER ?= qnxuser
QNX_TARGET_PASS ?= qnxuser
QNX_ROOT_PASS ?= root
QNX_TARGET_IP ?= 172.20.10.8
QNX_TARGET_DIR ?= /data/home/$(QNX_TARGET_USER)
 
# Default target - builds in Docker container and deploys all examples
default:
	docker exec -it qnx-build bash -c 'cd /home/cameronmacdonald/qnxprojects/trilobot-main && source /home/cameronmacdonald/qnx800/qnxsdp-env.sh && make -f Makefile.docker'
	$(MAKE) deploy-all
 
# Deploy all examples to the QNX target
deploy-all:
	@echo "Deploying all examples to QNX target..."
	@for example in $(EXAMPLES); do \
		echo "Deploying $$example..."; \
		SSHPASS=$(QNX_TARGET_PASS) sshpass -e scp -o StrictHostKeyChecking=no build/$(CONFIG_NAME)/$$example $(QNX_TARGET_USER)@$(QNX_TARGET_IP):$(QNX_TARGET_DIR)/; \
	done
	@echo "All examples deployed successfully."
 
# Deploy a specific example to the QNX target
deploy-%:
	SSHPASS=$(QNX_TARGET_PASS) sshpass -e scp -o StrictHostKeyChecking=no build/$(CONFIG_NAME)/$* $(QNX_TARGET_USER)@$(QNX_TARGET_IP):$(QNX_TARGET_DIR)/
 
# Run a specific example on the QNX target
run-%:
	SSHPASS=$(QNX_TARGET_PASS) sshpass -e ssh -o StrictHostKeyChecking=no $(QNX_TARGET_USER)@$(QNX_TARGET_IP) "su -c 'cd $(QNX_TARGET_DIR) && ./$*' root"
 
# Deploy and run a specific example
deploy-run-%: deploy-%
	$(MAKE) run-$*
 
# Clean up build artifacts
clean:
	rm -rf build
	docker exec -it qnx-build bash -c 'cd /home/cameronmacdonald/qnxprojects/trilobot-main && rm -rf build'
 
# The actual build target
build: $(TARGETS)
 
# Build architecture/variant string, e.g. aarch64le, x86_64le, etc.
PLATFORM ?= aarch64le
 
# Build profile: possible values: release, debug, profile, coverage
BUILD_PROFILE ?= debug
 
# Derived variables for output directory and target binaries
CONFIG_NAME ?= $(PLATFORM)-$(BUILD_PROFILE)
OUTPUT_DIR = build/$(CONFIG_NAME)
 
# Compiler and linker definitions
CC = qcc -Vgcc_nto$(PLATFORM)
LD = $(CC)
 
# Include paths (adjust for your repo structure)
INCLUDES = -Icommon/rpi_i2c/public/ -Icommon/rpi_gpio/public/ -Icommon/system/gpio/ -Icommon/rpi_trilobot
 
# Build profile flags
CCFLAGS_release = -O2
CCFLAGS_debug   = -g -O0 -fno-builtin
#CCFLAGS_profile  = -g -O0 -finstrument-functions
#CCFLAGS_coverage = -g -O0 -ftest-coverage -fprofile-arcs
 
# Combine common flags with the profile-specific flags
CCFLAGS_all = -Wall -fmessage-length=0 $(CCFLAGS_$(BUILD_PROFILE)) $(INCLUDES)
 
# Dependency generation flags
DEPS = -Wp,-MMD,$(@:%.o=%.d),-MT,$@
 
# List of example source files (located in the examples folder)
SRCS = $(foreach name, $(EXAMPLES), examples/$(name).c)
 
# Object files for examples (placed in the build directory preserving folder structure)
OBJS = $(patsubst examples/%.c, $(OUTPUT_DIR)/examples/%.o, $(SRCS))
 
# Trilobot source file and its corresponding object file.
TRILOBOT_SRC = common/rpi_trilobot/trilobot.c
TRILOBOT_OBJ = $(OUTPUT_DIR)/common/rpi_trilobot/trilobot.o
 
# Pattern rule for compiling example .c files into .o files.
$(OUTPUT_DIR)/examples/%.o: examples/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(CCFLAGS_all) $<
 
# Pattern rule for compiling trilobot.c.
$(OUTPUT_DIR)/common/rpi_trilobot/%.o: common/rpi_trilobot/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(CCFLAGS_all) $<
 
# Build static libraries for rpi_i2c and rpi_gpio from the common folder.
librpi_i2c.a:
	$(MAKE) -C common/rpi_i2c
 
librpi_gpio.a:
	$(MAKE) -C common/rpi_gpio
 
# Variables for the static libraries with correct paths.
LIB_RPI_I2C = common/rpi_i2c/build/$(CONFIG_NAME)/librpi_i2c.a
LIB_RPI_GPIO = common/rpi_gpio/build/$(CONFIG_NAME)/librpi_gpio.a
 
# Library search paths and libraries.
LIBS = -Lcommon/rpi_i2c/build/$(CONFIG_NAME) -lrpi_i2c -lm \
       -Lcommon/rpi_gpio/build/$(CONFIG_NAME) -lrpi_gpio -lm
 
# Final binary targets: one binary per example.
TARGETS = $(foreach name, $(EXAMPLES), $(OUTPUT_DIR)/$(name))
 
# Linking rule: each example binary is linked from its object file, the trilobot object, and the static libraries.
$(OUTPUT_DIR)/%: $(OUTPUT_DIR)/examples/%.o $(TRILOBOT_OBJ) $(LIB_RPI_I2C) $(LIB_RPI_GPIO)
	$(LD) -o $@ $^ $(LIBS)
 
# Rebuild: clean then build.
rebuild: clean build
 
# Include auto-generated dependency files if present.
-include $(OBJS:.o=.d)
