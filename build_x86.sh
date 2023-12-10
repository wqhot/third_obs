#!/bin/bash

# Define the source directory
SOURCE_DIR="$(dirname $0)"

# Define the build directory
BUILD_DIR="${SOURCE_DIR}/build_11"

# Create the build directory
mkdir -p "$BUILD_DIR/"
# rm -rf "$BUILD_DIR/*"

# Run the docker container and mount the source and build directories
docker run -v "$SOURCE_DIR:/project" \
           -w "/project/build_11" \
           -it wqhot/gcc11:v1.0 \
           /bin/bash -c "\
                git config --global --add safe.directory /project && \
                cmake -D BUILD_TYPE=RELEASE /project && \
                make -j4 && \
                make package
           "
