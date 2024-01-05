FROM leungjch/cuda118-tensorrt-base:latest as base

WORKDIR /home/docker/ament_ws

COPY model /model/pointpillars_model

RUN ROS_DISTRO=humble

COPY lidar_object_detection pointpillars_ws

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src \
    --rosdistro $ROS_DISTRO -y \
    --skip-keys "rti-connext-dds-6.0.1" 

COPY wato_ros_entrypoint.sh home/docker/wato_ros_entrypoint.sh
RUN sudo chmod +x home/docker/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ENTRYPOINT ["home/docker/wato_ros_entrypoint.sh"]
