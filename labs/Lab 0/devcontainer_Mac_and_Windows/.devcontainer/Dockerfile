# .devcontainer/Dockerfile

FROM ros:humble

# Set default shell to bash so the Fluxbox terminal knows what to launch
ENV SHELL=/bin/bash

# Added colcon build tools, ROS humble desktop, and DDS
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    python3-transforms3d \
    python3-colcon-common-extensions \
    python3-pip \
    curl \
    unzip \
    wget \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install rosbags and data science libraries
RUN pip3 install rosbags scipy matplotlib numpy

# Add Zenoh repository and install 1.8.0
RUN apt-get update && apt-get install -y gnupg curl && \
    mkdir -p /etc/apt/keyrings && \
    curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | tee /etc/apt/sources.list.d/zenoh.list > /dev/null && \
    apt-get update && \
    apt-get install -y zenoh-bridge-ros2dds=1.8.0 && \
    rm -rf /var/lib/apt/lists/*

# Convenience script for Zenoh
RUN echo '#!/bin/bash\nzenoh-bridge-ros2dds -e tcp/host.docker.internal:7447 "$@"' > /usr/local/bin/connect-zenoh && \
    chmod +x /usr/local/bin/connect-zenoh

# Convenience script to start noVNC on demand
RUN echo '#!/bin/bash\n\
if [ -f /usr/local/share/desktop-init.sh ]; then\n\
    nohup /usr/local/share/desktop-init.sh > /tmp/desktop.log 2>&1 &\n\
    echo -e "\\n====================================================================\\n||          noVNC GUI STARTED                                     ||\\n||  URL: http://localhost:6080/ (Password: ros)                   ||\\n====================================================================\\n"\n\
else\n\
    echo "Error: /usr/local/share/desktop-init.sh not found."\n\
fi' > /usr/local/bin/start-vnc && chmod +x /usr/local/bin/start-vnc

# Automatically source base ROS 2 and workspace overlay (if built) in new terminals
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "cd /workspace" >> /root/.bashrc