#!/bin/bash

# Source QNX environment
source /opt/qnx800/qnxsdp-env.sh

# List of examples (without .c extension)
EXAMPLES="imu motor_sw_pwm rgb_led ultrasonic button_rgb"
 
# Build architecture/variant string
PLATFORM=aarch64le
 
# Build profile
BUILD_PROFILE=debug
 
# Output directory
CONFIG_NAME="${PLATFORM}-${BUILD_PROFILE}"
OUTPUT_DIR="build/${CONFIG_NAME}"
 
# Create output directory
mkdir -p ${OUTPUT_DIR}
 
# Build static libraries first
(cd common/rpi_i2c && make)
(cd common/rpi_gpio && make)
 
# Compile each example
for example in ${EXAMPLES}; do
    echo "Building ${example}..."
    qcc -Vgcc_nto${PLATFORM} -o ${OUTPUT_DIR}/${example} \
        examples/${example}.c \
        common/rpi_trilobot/trilobot.c \
        -Icommon/rpi_i2c/public/ \
        -Icommon/rpi_gpio/public/ \
        -Icommon/system/gpio/ \
        -Icommon/rpi_trilobot \
        -Wall -g -O0 -fno-builtin \
        -Lcommon/rpi_i2c/build/${CONFIG_NAME} -lrpi_i2c \
        -Lcommon/rpi_gpio/build/${CONFIG_NAME} -lrpi_gpio \
        -lm
done 