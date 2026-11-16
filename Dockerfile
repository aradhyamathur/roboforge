# syntax=docker/dockerfile:1
#
# RoboForge in a container: the IDE (browser mode) and the Python SDK with all its
# dependencies: ROS 2, MuJoCo, pymavlink, Gymnasium, a C++ toolchain for C++
# controllers, the documentation tools and, optionally, PyTorch + stable-baselines3.
#
# Build:
#   docker build -t roboforge .                                  # with PyTorch (CPU build) + stable-baselines3
#   docker build -t roboforge --build-arg WITH_ML=0 .            # without them
#   docker build -t roboforge --build-arg TORCH_BACKEND=cu128 .  # PyTorch for an NVIDIA GPU (run with --gpus all)
#
# Run the IDE, then open http://127.0.0.1:7878 on the host:
#   docker run --rm -it --network host -v ~/roboforge-projects:/home/ubuntu/roboforge-projects roboforge
#
# The IDE listens on 127.0.0.1 only (see "Platform and security" in README.md), so use
# --network host: that also lets ROS 2 discovery and MAVLink's UDP reach the host as they would
# natively. Run an SDK app or a shell instead of the IDE:
#   docker run --rm -it --network host roboforge python3 -m roboforge run /opt/roboforge/sdk/python/examples/mujoco_ros_arm/app.yaml --headless
#   docker run --rm -it --network host roboforge bash
#
# MuJoCo renders off screen on the CPU (Mesa, EGL). For its viewer window, also pass
#   -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------- the IDE binary
FROM ros:${ROS_DISTRO}-ros-base AS ide
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates build-essential pkg-config \
    && rm -rf /var/lib/apt/lists/*
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable
WORKDIR /src
# The binary embeds the UI, the docs and the whole Python SDK with its examples, so it needs all of them.
COPY Cargo.toml Cargo.lock ./
COPY crates crates
COPY docs docs
COPY sdk sdk
RUN cargo build --release --locked -p roboforge && strip target/release/roboforge

# ---------------------------------------------------------------- the runtime
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG WITH_ML=1
# cpu (default, small) or a CUDA variant such as cu128; `auto` picks one from the build machine's driver.
ARG TORCH_BACKEND=cpu
ENV DEBIAN_FRONTEND=noninteractive

# Tools the IDE's SDK Manager and run configurations use, a C++ toolchain for C++ controllers,
# and Mesa for rendering without a GPU.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates tar bzip2 xz-utils unzip \
        build-essential g++ cmake ninja-build pkg-config \
        python3-dev \
        libegl1 libgl1 libgles2 libglx-mesa0 libgl1-mesa-dri libegl-mesa0 libosmesa6 \
        libglfw3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# The Python environment: the system Python (so ROS 2's rclpy stays importable) plus the pinned packages.
ENV VIRTUAL_ENV=/opt/roboforge/venv
COPY requirements.txt requirements-ml.txt /opt/roboforge/
RUN uv venv --python /usr/bin/python3 --system-site-packages $VIRTUAL_ENV \
    && uv pip install --python $VIRTUAL_ENV/bin/python -r /opt/roboforge/requirements.txt \
    && if [ "$WITH_ML" = "1" ]; then uv pip install --python $VIRTUAL_ENV/bin/python --torch-backend "$TORCH_BACKEND" -r /opt/roboforge/requirements-ml.txt; fi \
    && rm -rf /root/.cache

# The repository (SDK, examples, docs, tests) and the IDE.
COPY . /opt/roboforge/src
RUN uv pip install --python $VIRTUAL_ENV/bin/python --no-deps -e /opt/roboforge/src/sdk/python \
    && chown -R ubuntu:ubuntu /opt/roboforge
COPY --from=ide /src/target/release/roboforge /usr/local/bin/roboforge

ENV PATH=$VIRTUAL_ENV/bin:$PATH \
    ROS_DISTRO=${ROS_DISTRO} \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl \
    EGL_PLATFORM=surfaceless \
    ROBOFORGE_SDK=/opt/roboforge/src/sdk/python
RUN ln -s /opt/roboforge/src/sdk/python /opt/roboforge/sdk

# Every command runs with ROS 2 sourced.
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nexec "$@"\n' "$ROS_DISTRO" > /usr/local/bin/roboforge-entrypoint \
    && chmod +x /usr/local/bin/roboforge-entrypoint \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ubuntu/.bashrc

# Ubuntu's own non-root user (uid 1000, like most first users on a Linux host, so mounted folders stay yours).
USER ubuntu
WORKDIR /home/ubuntu
# The IDE's "light SDK environment" is the one above, so new SDK projects need no setup step.
RUN mkdir -p /home/ubuntu/.roboforge/envs /home/ubuntu/roboforge-projects \
    && ln -s $VIRTUAL_ENV /home/ubuntu/.roboforge/envs/sdk-venv \
    && python3 -m roboforge doctor

EXPOSE 7878
ENTRYPOINT ["/usr/local/bin/roboforge-entrypoint"]
CMD ["roboforge", "--browser", "--no-open", "--port", "7878"]
