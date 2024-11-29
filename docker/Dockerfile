FROM ubuntu:22.04

# **************************************************************************** #

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND=noninteractive

# **************************************************************************** #

# Обновляем пакеты и устанавливаем необходимые утилиты
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8


# Устанавливаем язык и добавляем репозитории
RUN apt-get update && \
    add-apt-repository universe && \
    apt-get update

# Development tools
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    wget \
    vim \
    git \
    tmux \
    tree \
    gnome-terminal \
    dbus-x11 \    
    && rm -rf /var/lib/apt/lists/*

# Добавляем репозиторий ROS 2 и устанавливаем его
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop ros-dev-tools

# **************************************************************************** #

WORKDIR /opt

# Клонирование и сборка Micro XRCE-DDS Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

RUN ldconfig /usr/local/lib/

# **************************************************************************** #

ENV PROJECT_NAME ROS-Multiagent-Robotics-System

# set work directory
ENV WORKSPACE /workspace/src
RUN mkdir -p $WORKSPACE/$PROJECT_NAME
WORKDIR $WORKSPACE

# copy project
COPY . $WORKSPACE/ROS-Multiagent-Robotics-System

# **************************************************************************** #

# Install PX4 packages

RUN git clone https://github.com/PX4/PX4-Autopilot.git

RUN cd ./PX4-Autopilot && \
    git checkout v1.14.4 && \
    git submodule sync --recursive && \
    git submodule update --init --recursive

RUN  ./PX4-Autopilot/Tools/setup/ubuntu.sh

RUN cd PX4-Autopilot && make

# **************************************************************************** #

WORKDIR $WORKSPACE

RUN git clone https://github.com/PX4/px4_msgs.git && \
    cd px4_msgs && \
    git checkout release/1.14 && \
    cd ..

RUN source /opt/ros/humble/setup.bash && 
    colcon build --packages-select px4_msgs

# **************************************************************************** #

RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    ros-humble-simulation \
    ros-humble-joint-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-ros-gz \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# **************************************************************************** #

# Install Python 3 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install python dependencies
COPY ../requirements.txt /tmp/requirements.txt

RUN pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt  && \
    rm -rf /root/.cache/pip

# **************************************************************************** #

# Создание нового пользователя
RUN useradd -ms /bin/bash rosuser && \
    echo 'rosuser:1111' | chpasswd && \
    usermod -aG sudo rosuser

# **************************************************************************** #

# Install QGroundControl

WORKDIR /opt

RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    && rm -rf /var/lib/apt/lists/*

# Удаляем ModemManager, так как он может конфликтовать с MAVLink
RUN apt-get remove -y modemmanager

# Добавляем пользователя в группу dialout для доступа к последовательным устройствам
RUN usermod -a -G dialout rosuser

RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O /opt/QGroundControl.AppImage \
    && chmod +x /opt/QGroundControl.AppImage  

# **************************************************************************** #

WORKDIR $WORKSPACE

# Устонавливаем вспомоагтельные репозитории для разработки
RUN git clone https://github.com/gazebosim/ros_gz_project_template.git && \
    git clone https://github.com/PX4/PX4-gazebo-models.git

# **************************************************************************** #

# Переключаемся на пользователя rosuser
# USER rosuser

WORKDIR /workspace

# Настраиваем .bashrc для пользователя rosuser
RUN echo "source /opt/ros/humble/setup.bash" >> /home/rosuser/.bashrc
RUN echo "alias sros='source /workspace/install/setup.bash'" >> /home/rosuser/.bashrc
RUN echo "alias agent='MicroXRCEAgent udp4 -p8888'" >> /home/rosuser/.bashrc
RUN echo "export PATH=\$PATH:/workspace/src/PX4-Autopilot/build/px4_sitl_default/bin" >> /home/rosuser/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source /workspace/install/setup.bash'" >> ~/.bashrc
RUN echo "alias agent='MicroXRCEAgent udp4 -p8888'" >> ~/.bashrc
RUN echo "export PATH=\$PATH:/workspace/src/PX4-Autopilot/build/px4_sitl_default/bin" >> ~/.bashrc