ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1

WORKDIR /root/

RUN rm -f /var/lib/dpkg/info/libc-bin.* \
 && apt-get clean \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    libc-bin \
    tmux nano nginx wget git \
    python3-dev python3-pip python3-venv \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-gscam \
 && apt-get autoremove -y \
 && apt-get clean -y \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
 && apt-get autoremove -y \
 && apt-get clean -y \
 && rm -rf /var/lib/apt/lists/*

COPY ros2_ws /root/ros2_ws
WORKDIR /root/ros2_ws

RUN git submodule update --init --recursive

RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --symlink-install \
 && ros2 run mavros install_geographiclib_datasets.sh \
 && apt-get autoremove -y \
 && apt-get clean -y \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> ~/.bashrc \
 && echo "source /root/ros2_ws/install/setup.sh" >> ~/.bashrc

COPY files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh \
 && rm /install-ttyd.sh

COPY files/nginx.conf /etc/nginx/nginx.conf
COPY files/index.html /usr/share/ttyd/index.html

COPY files/start.sh /start.sh
COPY files/register_service /site/register_service
RUN chmod +x /start.sh /site/register_service

LABEL version="0.1.0"
LABEL authors='[{"name":"Sanket Sharma","email":"sharma.sanket272@gmail.com"}]'
LABEL company='{"about":"Sanket","name":"Sanket","email":"sharma.sanket272@gmail.com"}'
LABEL readme="https://github.com/snktshrma/blueos-viso-inertial/blob/main/README.md"
LABEL type="device-integration"
LABEL tags='["data-collection"]'
LABEL links='{"source":"https://github.com/snktshrma/blueos-viso-inertial"}'
LABEL requirements="core >= 1.1"
LABEL permissions='{\
  "NetworkMode":"host",\
  "HostConfig":{\
    "Binds":[\
      "/dev:/dev:rw",\
      "/usr/blueos/extensions/ros2/:/root/persistent_ws/:rw"\
    ],\
    "Privileged":true,\
    "CpuQuota":200000,\
    "CpuPeriod":100000,\
    "Memory":1097152000\
  }\
}'

RUN echo "set +e" >> ~/.bashrc

ENTRYPOINT ["/start.sh"]

