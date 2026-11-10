# Clean aarch64 Linux toolchain. No prebuilt MOOS or MAVMOOS artifacts.
FROM ubuntu@sha256:d1e2e92c075e5ca139d51a140fff46f84315c0fdce203eab2807c7e495eff4f9
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git ca-certificates curl pkg-config \
    python3 python3-venv python3-dev python3-pip \
    libproj-dev libssl-dev zlib1g-dev lsof procps \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
