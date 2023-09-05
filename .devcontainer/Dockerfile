FROM mcr.microsoft.com/vscode/devcontainers/base:jammy

ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y --no-install-recommends\
        mc \
        vim \
        gdb \
        build-essential \
        git \
        zip \
        wget \
        cmake \
	&& \
    rm -rf /var/lib/apt/lists/*  \
    && apt-get clean

RUN wget -O /etc/apt/trusted.gpg.d/ros2.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && \
	echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/ros2.gpg] http://packages.ros.org/ros2/ubuntu jammy main">/etc/apt/sources.list.d/ros2.list && \
   wget -O - https://cyberbotics.com/Cyberbotics.asc |apt-key add - && \
	echo "deb https://cyberbotics.com/debian/ binary-amd64/">/etc/apt/sources.list.d/webots.list 

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/vscode/.bashrc 

RUN apt-get update && apt-get install -y --no-install-recommends\
         python3-yapf \
         yapf3 \
         clang-format-14 \
         python3-pycodestyle \
         ros-humble-desktop \
         python3-colcon-ros \
         webots \
         ffmpeg \
         libaribb24-0 \
         ros-humble-webots-ros2 \
	&& \
    rm -rf /var/lib/apt/lists/*  \
    && apt-get clean

RUN chmod 777 /home/vscode