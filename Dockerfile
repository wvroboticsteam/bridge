FROM ros:indigo-ros-base

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-tf \
    ros-indigo-tf2 \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
ENV WVWS /home/docker/wv
RUN mkdir -p ${WS}/src

WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
&& catkin build

#EXPOSE 8000
#EXPOSE 8001
#ENV ROS_MASTER_URI http://127.0.0.1:8001

RUN mkdir -p ${WVWS}/src
WORKDIR ${WVWS}
RUN git clone https://github.com/wvroboticsteam/bridge.git ${WVWS}/src/bridge
RUN . /opt/ros/indigo/setup.sh && catkin_make install
#RUN echo "source install/setup.sh; bridge-app" > runitdamnit.sh
ADD startup.bash startup.bash
CMD ["./startup.bash"]
