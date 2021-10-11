ARG BASE_IMAGE
FROM ghcr.io/aica-technology/ros-ws:noetic

ARG USERNAME
USER ${USERNAME}
WORKDIR ${HOME}/${USERNAME}_ws/src
COPY --chown=${USER}  ./object_descriptions ./object_descriptions

WORKDIR ${HOME}/${USERNAME}_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"