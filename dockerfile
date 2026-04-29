
# Minimal environment for running the kompass-core CPU benchmark.
# No SYCL, no AdaptiveCpp — those are only needed for the GPU build.
# For the CPU-native build (cmake .. -DFORCE_CPU_BUILD=ON) the stock
# Ubuntu toolchain is all you need.
 
FROM ubuntu:22.04
 
ENV DEBIAN_FRONTEND=noninteractive
 
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    # Build toolchain: GCC/G++, make, cmake, git
    build-essential \
    cmake \
    git \
    # Kompass C++ library dependencies
    libompl-dev \
    libfcl-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libboost-all-dev \
    libopencv-dev \    
    python3 \
    pip \
    sudo \
    # ROS 2 and Nav2 dependencies
    ros-humble-ros-core \
    ros-humble-nav2-mppi-controller \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install matplotlib
 
ARG USER_ID=1000
ARG GROUP_ID=1000

RUN groupadd -g ${GROUP_ID} developer && \
    useradd -u ${USER_ID} -g developer -m -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER developer

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
 

