#!/bin/bash

# Define the source directory
SOURCE_DIR="$(dirname $0)"

KYLIN_LIB="${HOME}/libs/cross"

# Define the build directory
BUILD_DIR="${SOURCE_DIR}/arm_build"

echo "KYLIN_LIB: $KYLIN_LIB"
echo "SOURCE_DIR: $SOURCE_DIR"
echo "BUILD_DIR: $BUILD_DIR"

# Create the build directory
mkdir -p "$BUILD_DIR/"


# Run the docker container and mount the source and build directories
docker run -v "$SOURCE_DIR:/project" \
           -v "$KYLIN_LIB:/cross" \
           -v "$BUILD_DIR:/project/arm_build" \
           -w "/project/arm_build" \
           -it wqhot/gcc9_aarch64:v1.1 \
           /bin/bash -c "\
                git config --global --add safe.directory /project && \
                cmake -D BUILD_TYPE=RELEASE -D PLATFORM=ARM .. && \
                make -j4
                make package
              "
