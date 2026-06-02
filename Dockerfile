# Smartfin GPS+IMU offline pipeline — build & run without local toolchain setup.
#
# Build:
#   docker build -t smartfin-ble-client .
#
# Run fusion + plots on bundled fixtures:
#   docker run --rm -v "$(pwd)/docker-out:/workspace/docker-out" smartfin-ble-client
#
# Interactive shell with tools on PATH:
#   docker run --rm -it smartfin-ble-client bash

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV SMARTFIN_BUILD_DIR=/workspace/build
ENV PATH="/workspace/build:${PATH}"

RUN apt-get update -q \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        ninja-build \
        python3 \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Plot script + optional Python tests (matplotlib; scipy matches CI tests).
RUN pip3 install --no-cache-dir --break-system-packages \
        matplotlib \
        numpy \
        scipy

WORKDIR /workspace

# Dependencies change rarely — copy build files first for layer cache.
COPY CMakeLists.txt ./
COPY src ./src
COPY tests ./tests
COPY tools ./tools
COPY unprocessed ./unprocessed
COPY docs ./docs
COPY docker ./docker

RUN cmake -S . -B "${SMARTFIN_BUILD_DIR}" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DSMARTFIN_ENABLE_SIMPLEBLE=OFF \
    && cmake --build "${SMARTFIN_BUILD_DIR}" \
        --target fusion_export gps_inspect orient_ride_inspect \
                   gps_test ahrs_orient_test ekf_fusion_test

# Default: run paired fake GPS + IMU pipeline and write artifacts under docker-out/.
CMD ["/workspace/docker/run_pipeline.sh"]
