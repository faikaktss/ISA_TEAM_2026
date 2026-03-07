FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get remove -y python3-numpy && \
    python3 -m pip install "numpy<2" --break-system-packages

RUN python3 -m pip install \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    --break-system-packages --no-deps

RUN python3 -m pip install \
    ultralytics pyserial \
    --break-system-packages \
    --ignore-installed

WORKDIR /workspace