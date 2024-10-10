FROM ubuntu:noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt install -y software-properties-common && add-apt-repository universe
RUN apt update && apt install -y curl && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
RUN apt update && apt install -y \
        python3-flake8-blind-except \
        python3-flake8-class-newline \
        python3-flake8-deprecated \
        python3-mypy \
        python3-pip \
        python3-pytest \
        python3-pytest-cov \
        python3-pytest-mock \
        python3-pytest-repeat \
        python3-pytest-rerunfailures \
        python3-pytest-runner \
        python3-pytest-timeout \
        ros-dev-tools
WORKDIR /ros2_rolling
RUN mkdir src && vcs import --input https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos src
RUN apt upgrade -y && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"