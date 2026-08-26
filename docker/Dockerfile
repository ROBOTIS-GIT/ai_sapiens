FROM ros:jazzy

WORKDIR /root

ARG ONNXRUNTIME_VERSION=1.23.2
ARG TARGETARCH

RUN set -eux; \
    case "$TARGETARCH" in \
      amd64) ORT_ARCH=x64 ;; \
      arm64) ORT_ARCH=aarch64 ;; \
      *) echo "Unsupported TARGETARCH: $TARGETARCH"; exit 1 ;; \
    esac && \
    curl -sL https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}.tgz -o /tmp/ort.tgz && \
    mkdir -p /tmp/onnxruntime && \
    tar -xzf /tmp/ort.tgz -C /tmp/onnxruntime --strip-components=1 && \
    cp -r /tmp/onnxruntime/include /usr/local/include/onnxruntime && \
    cp -r /tmp/onnxruntime/lib/* /usr/local/lib/ && \
    cd /usr/local/lib && \
    if [ ! -f libonnxruntime.so ] && [ -f libonnxruntime.so.* ]; then \
        ln -sf $(ls libonnxruntime.so.* | head -1) libonnxruntime.so; \
    fi && \
    ldconfig && \
    rm -rf /tmp/onnxruntime /tmp/ort.tgz

ARG MUJOCO_VERSION=3.5.0

RUN set -eux; \
    case "$TARGETARCH" in \
      amd64) MJ_ARCH=x86_64 ;; \
      arm64) MJ_ARCH=aarch64 ;; \
      *) echo "Unsupported TARGETARCH: $TARGETARCH"; exit 1 ;; \
    esac && \
    curl -sL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MJ_ARCH}.tar.gz -o /tmp/mujoco.tgz && \
    mkdir -p /tmp/mujoco && \
    tar -xzf /tmp/mujoco.tgz -C /tmp/mujoco --strip-components=1 && \
    cp -r /tmp/mujoco/include/mujoco /usr/local/include/mujoco && \
    cp -a /tmp/mujoco/lib/. /usr/local/lib/ && \
    ldconfig && \
    rm -rf /tmp/mujoco /tmp/mujoco.tgz

RUN apt-get update && apt-get install -y --no-install-recommends libglfw3-dev && \
    rm -rf /var/lib/apt/lists/*

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src

COPY . ${COLCON_WS}/src/ai_sapiens

ENV ROS_DISTRO=jazzy

RUN cd ${COLCON_WS} && \
    apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2" -y && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install ros-jazzy-rmw-zenoh-cpp -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1'" >> ~/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
    echo "alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'">> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
