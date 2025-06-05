FROM osrf/ros:humble-desktop-full-jammy


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DISPLAY=:0

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get upgrade \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly


# Set ENV for NVIDIA
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


WORKDIR /home/$USERNAME/

# RUN git clone git@github.com:mlisi1/momobs_mujoco.git
# RUN cd momobs_mujoco/ \
#     && git submodule init --recursive

RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libxrender1 \
    libxrandr2 \
    libxi6 \
    libosmesa6-dev \
    libgl1-mesa-glx \
    libglfw3 \
    libglfw3-dev \
    libglew-dev \
    ros-humble-robot-state-publisher \
    ros-humble-pinocchio \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*


COPY docker_res/entrypoint.sh /entrypoint.sh
COPY docker_res/bashrc /home/${USERNAME}/.bashrc


USER ros
COPY anymal_c_config /home/$USERNAME/haptiquad_mujoco/anymal_c_config
COPY anymal_c_simple_description /home/$USERNAME/haptiquad_mujoco/anymal_c_simple_description
COPY haptiquad_mujoco_bringup /home/$USERNAME/haptiquad_mujoco/haptiquad_mujoco_bringup
COPY champ /home/$USERNAME/haptiquad_mujoco/champ
COPY haptiquad_ros2 /home/$USERNAME/haptiquad_mujoco/haptiquad_ros2
COPY Spot-MuJoCo-ROS2 /home/$USERNAME/haptiquad_mujoco/Spot-MuJoCo-ROS2
COPY build.sh /home/$USERNAME/haptiquad_mujoco/
RUN sudo chmod +x /home/ros/haptiquad_mujoco/build.sh
RUN bash -c "source /home/ros/.bashrc && haptiquad_mujoco/build.sh"

# RUN sudo echo 'source /home/ros/install/setup.bash' >> /home/${USERNAME}/.bashrc



# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]