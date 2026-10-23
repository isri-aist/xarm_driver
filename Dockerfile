# Runs `uri interface` with the RobotDriverxArm plugin for UFACTORY xArm
# robots.
#
# URI, ROS Jazzy, mc_rtc, and Zenoh are provided by the base image under
# URI_PREFIX=/opt/uri. This image builds the official xArm C++ SDK and
# installs xarm_driver into the same prefix as URI.

ARG BASE_IMAGE=ghcr.io/isri-aist/unified_robot_interface:latest
FROM ${BASE_IMAGE}

USER root

ARG XARM_SDK_REPOSITORY=https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
ARG XARM_SDK_REF=master

# Build dependencies for the xArm SDK and xarm_driver.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
      ca-certificates \
      git \
      libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install the official xArm C++ SDK.
RUN git clone --depth 1 \
      --branch "${XARM_SDK_REF}" \
      "${XARM_SDK_REPOSITORY}" \
      /tmp/xarm_cplus_sdk \
    && cmake \
      -S /tmp/xarm_cplus_sdk \
      -B /tmp/xarm_cplus_sdk/build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_INSTALL_PREFIX="${URI_PREFIX}" \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build /tmp/xarm_cplus_sdk/build --parallel "$(nproc)" \
    && cmake --install /tmp/xarm_cplus_sdk/build \
    && rm -rf /tmp/xarm_cplus_sdk

# Build and install RobotDriverxArm.
COPY . /tmp/xarm_driver
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && cmake \
      -S /tmp/xarm_driver \
      -B /tmp/xarm_driver/build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_PREFIX_PATH="${URI_PREFIX};${EXTRA_DEPS_PREFIX};/opt/ros/${ROS_DISTRO}" \
      -DCMAKE_INSTALL_PREFIX="${URI_PREFIX}" \
      -DBUILD_TESTING=OFF \
    && cmake --build /tmp/xarm_driver/build --parallel "$(nproc)" \
    && cmake --install /tmp/xarm_driver/build \
    && mkdir -p /config "${URI_PREFIX}/share/xarm_driver" \
    && cp -r /tmp/xarm_driver/etc/. "${URI_PREFIX}/share/xarm_driver/" \
    && rm -rf /tmp/xarm_driver \
    && test -f "${URI_PREFIX}/lib/robot_interface/libRobotDriverxArm.so"

USER vscode
WORKDIR /config

# Mount a directory containing robot_interface.yaml on /config.
# Example configurations are installed in /opt/uri/share/xarm_driver.
# The base image entrypoint sources the ROS environment.
CMD ["uri", "interface", "-c", "/config/robot_interface.yaml"]
