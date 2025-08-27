FROM osrf/ros:humble-desktop

RUN apt update
RUN apt update
RUN apt install -y usbutils net-tools software-properties-common wget
RUN apt-get install -y libjpeg-dev libjpeg8-dev libfreetype6-dev vim

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py
RUN python3 -m pip install --upgrade odrive

RUN apt-get update

RUN apt-get install -y ros-humble-diagnostic-updater
RUN apt-get install -y ros-humble-tf-transformations
Run apt-get install ros-humble-rosbag2

RUN apt-get update
RUN apt install -y ros-humble-navigation2 
RUN apt install -y ros-humble-nav2-bringup 
RUN apt-get install -y ros-humble-rosbridge-server
RUN apt install -y ros-humble-robot-localization

RUN python3 -m pip install pandas

RUN pip3 install is-msgs==1.1.10 \
    &&   pip3 install is-wire==1.2.0 \
    &&   pip3 install numpy==1.21.6 \
    &&   pip3 install vine==1.3.0 \
    && pip3 install --upgrade protobuf==3.20.0 
