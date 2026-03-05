ARG ROS_DISTRO=humble

############################################
# -------- Stage 1: OpenCV Builder --------
############################################

FROM osrf/ros:${ROS_DISTRO}-desktop AS opencv-builder

ARG OPENCV_VER=4.10.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ninja-build \
    libgtk-3-dev \
    libjpeg-dev libpng-dev libtiff-dev libwebp-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libopenblas-dev \
    libeigen3-dev libboost-all-dev libode-dev libflann-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/opencv_build

RUN git clone --depth 1 --branch ${OPENCV_VER} https://github.com/opencv/opencv.git \
    && git clone --depth 1 --branch ${OPENCV_VER} https://github.com/opencv/opencv_contrib.git

RUN cmake -S opencv -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/opt/opencv \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_build/opencv_contrib/modules \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_hdf=OFF \
    -D WITH_GTK=ON -D GTK_VERSION=3 \
    && ninja -C build -j"$(nproc)" install

############################################
# -------- Stage 2: Runtime Image ---------
############################################

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG USER=ubuntu
ARG UID=1000
ARG GID=1000
ARG WORKSPACE_NAME="ros2_ws"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ninja-build ccache \
    libgtk-3-dev \
    libjpeg-dev libpng-dev libtiff-dev libwebp-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libopenblas-dev \
    libeigen3-dev libboost-all-dev libode-dev libflann-dev \
    libgtest-dev \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Copy OpenCV from builder
COPY --from=opencv-builder /opt/opencv /opt/opencv

ENV LD_LIBRARY_PATH=/opt/opencv/lib
ENV PKG_CONFIG_PATH=/opt/opencv/lib/pkgconfig

# ROS shell setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /etc/bash.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc

WORKDIR /opt/${WORKSPACE_NAME}
COPY ${WORKSPACE_NAME}/src /tmp/src

RUN rosdep init 2>/dev/null || true && rosdep update

RUN rosdep update \
    && apt-get update \
    &&rosdep install --from-paths /tmp/src --rosdistro ${ROS_DISTRO} --ignore-src -y \
    && rm -rf /tmp/src

RUN getent group ${GID} || groupadd -g ${GID} ${USER}
RUN getent passwd ${UID} || useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER}

USER ${USER}
WORKDIR /home/${USER}/work

# Keep colcon logs where you expect them
ENV COLCON_LOG_PATH=/home/${USER}/.colcon/log
ENV COLCON_DEFAULTS_FILE=/home/${USER}/work/${WORKSPACE_NAME}/.colcon/defaults.yaml

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default

# Default launch command
ENTRYPOINT [ "/bin/bash" ]