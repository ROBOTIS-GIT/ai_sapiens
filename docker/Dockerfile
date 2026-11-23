FROM ros:jazzy

WORKDIR /root

ARG ONNXRUNTIME_VERSION

COPY scripts/install_onnxruntime.sh /tmp/install_onnxruntime.sh
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates unzip && \
    bash /tmp/install_onnxruntime.sh "$ONNXRUNTIME_VERSION" && \
    rm -rf /var/lib/apt/lists/* /tmp/install_onnxruntime.sh

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
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --parallel-workers 1'" >> ~/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
    echo "alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'">> ~/.bashrc
