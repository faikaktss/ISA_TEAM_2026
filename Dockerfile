FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install "numpy<2"

RUN python3 -m pip install \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    --no-deps

RUN python3 -m pip install \
    ultralytics pyserial \
    --ignore-installed

WORKDIR /workspace
