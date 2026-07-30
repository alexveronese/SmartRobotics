FROM ubuntu:24.04

ARG ROS_DISTRO=rolling
ARG DEV_USER=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG PACKT_REPO_REF=main

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PACKT_REPO=/workspaces/mastering_ros2
ENV CHAPTER_WS=/workspaces/chapter_ws
ENV CHAPTER09_INSTALL=/opt/chapter09_ws/install
ENV SORTING_DEMO_INSTALL=/opt/square_sorting_ws/install
ENV ROS_ACTIVE_CHAPTER=Chapter03

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS 2 Rolling installation from the official Ubuntu deb guide:
# https://docs.ros.org/en/rolling/Get-Started/Installation/Ubuntu-Install-Debs.html
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        locales \
        software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && ros_apt_source_version="$( \
        curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | awk -F '"' '/tag_name/ { print $4; exit }' \
    )" \
    && test -n "${ros_apt_source_version}" \
    && source /etc/os-release \
    && ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        "ros-${ROS_DISTRO}-desktop" \
        bash-completion \
        fluxbox \
        git \
        less \
        nano \
        novnc \
        python3-numpy \
        ros-dev-tools \
        "ros-${ROS_DISTRO}-controller-manager" \
        "ros-${ROS_DISTRO}-gz-ros2-control" \
        "ros-${ROS_DISTRO}-joint-state-broadcaster" \
        "ros-${ROS_DISTRO}-joint-trajectory-controller" \
        "ros-${ROS_DISTRO}-realsense2-description" \
        "ros-${ROS_DISTRO}-ros-gz-bridge" \
        "ros-${ROS_DISTRO}-ros-gz-interfaces" \
        "ros-${ROS_DISTRO}-ros-gz-sim" \
        sudo \
        websockify \
        x11vnc \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN if getent passwd "${USER_UID}" >/dev/null; then \
        existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
        userdel --remove "${existing_user}"; \
    fi \
    && if getent group "${USER_GID}" >/dev/null; then \
        existing_group="$(getent group "${USER_GID}" | cut -d: -f1)"; \
        groupdel "${existing_group}"; \
    fi \
    && groupadd --gid "${USER_GID}" "${DEV_USER}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" \
        --create-home --shell /bin/bash "${DEV_USER}" \
    && echo "${DEV_USER} ALL=(ALL) NOPASSWD:ALL" \
        > "/etc/sudoers.d/${DEV_USER}" \
    && chmod 0440 "/etc/sudoers.d/${DEV_USER}" \
    && mkdir -p "${PACKT_REPO}" "${CHAPTER_WS}" \
    && chown -R "${DEV_USER}:${DEV_USER}" /workspaces

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
        rosdep init; \
    fi

COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/ros-entrypoint
COPY --chmod=0755 docker/install-chapter-deps /usr/local/bin/install-chapter-deps
COPY --chmod=0755 docker/build-chapter /usr/local/bin/build-chapter
COPY --chmod=0755 docker/build-sorting-demo /usr/local/bin/build-sorting-demo
COPY --chmod=0755 docker/run-square-sorting-demo /usr/local/bin/run-square-sorting-demo
COPY docker/chapter09-rolling.patch /tmp/chapter09-rolling.patch

USER ${DEV_USER}
WORKDIR /workspaces

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && git clone --depth 1 --branch "${PACKT_REPO_REF}" \
        https://github.com/PacktPublishing/Mastering-ROS-2-for-Robotics-Programming.git \
        "${PACKT_REPO}" \
    && colcon list --base-paths "${PACKT_REPO}/Chapter03" \
        | grep -q master_ros2_pkg \
    && printf '%s\n' \
        'source /opt/ros/${ROS_DISTRO}/setup.bash' \
        'source "${CHAPTER09_INSTALL}/setup.bash"' \
        'source "${SORTING_DEMO_INSTALL}/setup.bash"' \
        'if [[ -f "${CHAPTER_WS}/${ROS_ACTIVE_CHAPTER}/install/setup.bash" ]]; then' \
        '  source "${CHAPTER_WS}/${ROS_ACTIVE_CHAPTER}/install/setup.bash"' \
        'fi' \
        'if [[ -f /workspaces/sorting_ws/install/setup.bash ]]; then' \
        '  source /workspaces/sorting_ws/install/setup.bash' \
        'fi' \
        >> "/home/${DEV_USER}/.bashrc"

USER root

RUN mkdir -p /opt/chapter09_ws/src /opt/square_sorting_ws/src \
    && cp -a \
        "${PACKT_REPO}/Chapter09/panda_description" \
        /opt/chapter09_ws/src/panda_description \
    && cd /opt/chapter09_ws/src/panda_description \
    && patch -p1 < /tmp/chapter09-rolling.patch \
    && chown -R "${DEV_USER}:${DEV_USER}" \
        /opt/chapter09_ws /opt/square_sorting_ws

COPY --chown=${DEV_USER}:${DEV_USER} \
    sorting_ws/src/square_sorting_demo \
    /opt/square_sorting_ws/src/square_sorting_demo

USER ${DEV_USER}

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && cd /opt/chapter09_ws \
    && colcon build --merge-install --event-handlers console_direct+ \
    && source "${CHAPTER09_INSTALL}/setup.bash" \
    && cd /opt/square_sorting_ws \
    && colcon build --merge-install --event-handlers console_direct+

WORKDIR ${PACKT_REPO}

ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
CMD ["bash"]
