#-----------------------------
# base image include 
    #Ubnutu20.04
    #Cuda11.3
    #Cudnn8
#-----------------------------
FROM nvidia/cuda:11.5.2-cudnn8-devel-ubuntu20.04

#-----------------------------
# speeding up
#-----------------------------
RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list

#-----------------------------
# Environment Variables
#-----------------------------
ENV LC_ALL=C.UTF-8
ENV export LANG=C.UTF-8
ENV SHELL /bin/bash
# no need input key
ENV DEBCONF_NOWARNINGS=yes
ENV DEBIAN_FRONTEND noninteractive
# for display
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

#-----------------------------
# install common library
#-----------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-dev \
    python3-pip \
    python-is-python3 \
    git \
    nano\
    curl \
    screen \
    sudo \
    g++ \
    wget \
    net-tools \
    software-properties-common \
    make \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    libgoogle-glog-dev \
    libboost-all-dev \
    libhdf5-dev \
    libpq-dev \
    libatlas-base-dev

#-----------------------------
# install additional packageat
#-----------------------------
RUN pip3 install --upgrade pip
RUN pip3 install setuptools \
    opencv-python \
    numpy==1.20.2 \
    flask \
    flask-core \
    requests \ 
    gunicorn \
    uWSGI \
    pillow \
    psutil \
    tqdm
#######################################################################
#                       install display settings                         
#######################################################################
RUN apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev \
    mesa-utils \
    x11-apps

# nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
#    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
# ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

#-----------------------------
# intall ros-noetic 
#-----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get -y update && \
    apt-get -y install ros-noetic-ros-base &&\
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    apt install -y python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-rosdep && \
    rosdep init &&\
    rosdep update 

RUN apt-get -y install ros-noetic-ros-numpy \ 
    ros-noetic-smach-ros \
    ros-noetic-cv-bridge \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

#-----------------------------
# create catkin_ws
#-----------------------------
RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
RUN mkdir -p /catkin_ws/src
RUN cd /catkin_ws/src && /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'

#-----------------------------
# install cmake
#-----------------------------
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.24.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.24.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.24.0-Linux-x86_64/bin:${PATH}"

#-----------------------------
# install openpose
#-----------------------------
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
RUN ls
RUN cd openpose/ && git submodule update --init --recursive --remote
RUN cd /openpose/ && mkdir build && cd build && cmake -DBUILD_PYTHON=ON -DPROFILER_ENABLED=ON -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DUSE_CUDNN=OFF -DDOWNLOAD_BODY_25_MODEL=OFF -DDOWNLOAD_BODY_COCO_MODEL=OFF -DDOWNLOAD_BODY_MPI_MODEL=OFF -DDOWNLOAD_FACE_MODEL=OFF -DDOWNLOAD_HAND_MODEL=OFF ..

COPY ./models/pose_iter_584000.caffemodel /openpose/models/pose/body_25
RUN cd /openpose/build/ && make -j10
RUN cd /openpose/build/ && make install
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
RUN echo "PYTHONPATH=/usr/local/python:$PYTHONPATH" >> ~/.bashrc && \
    /bin/bash -c  "source ~/.bashrc"

#-----------------------------
# set wokingdir
#-----------------------------
COPY ./human_position /catkin_ws/src/openpose-ros-docker/human_position
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

#for openpose ,smach
ENV PYTHONPATH /usr/local/python
ENV PYTHONPATH "/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"

WORKDIR /catkin_ws


