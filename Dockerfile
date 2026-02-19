# ── Base image: official ROS 2 Humble (multi-arch: works on x86 AND ARM64) ──
FROM ros:humble-ros-base-jammy

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── Copy source code ─────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY src/ src/

# ── Install ROS package dependencies from package.xml ────────────────────────
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble

# ── Build the package ─────────────────────────────────────────────────────────
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf build/ log/

# ── Source overlay on every container start ───────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
