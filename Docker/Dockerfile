## Build with:
##   docker build -t lcas.lincoln.ac.uk/lcas/botanbot_lcas:docker .
##
## Run with:
##   docker-compose up
##
## Kill with:
##   docker-compose down

FROM tiryoh/ros2-desktop-vnc:foxy
LABEL maintainer="Nikos Tsagkopoulos<n.tsagko@gmail.com>"

RUN apt-get update; apt-get -y upgrade

# Creating ros2_ws
RUN mkdir -p ~/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
                  cd ~/ros2_ws/ && \
                  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4"

# Cloning botanbot repo
RUN cd ~/ros2_ws/src; git clone -b main --recursive https://github.com/LCAS/botanbot_sim.git

# Installing dependecies and compiling
RUN . /opt/ros/foxy/setup.sh; \
    apt install python3-colcon-common-extensions && \
    apt install -y python3-rosdep2 xdotool psmisc gnome-terminal && \
    apt install -y ros-foxy-twist-mux ros-foxy-navigation2 && \
    rosdep update && cd ~/ros2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy && \
    apt-get -y clean

# TODO: 'twist-mux' and 'navigation2' should be solved by rosdep

RUN /bin/bash -c '. /opt/ros/foxy/setup.bash; cd /home/ubuntu/ros2_ws; \
                  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4; \
                  echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc; \
                  echo "export NO_AT_BRIDGE=1" >> /home/ubuntu/.bashrc'

# export NO_AT_BRIDGE fixes an error in the rqt:
# "AT-SPI: Error retrieving accessibility bus address: org.freedesktop.DBus.Error.ServiceUnknown:
# The name org.a11y.Bus was not provided by any .service files"

RUN cd /tmp && curl -fOL https://github.com/cdr/code-server/releases/download/v3.12.0/code-server_3.12.0_amd64.deb && dpkg -i code-server_3.12.0_amd64.deb && rm code-server_3.12.0_amd64.deb

RUN bash -c 'echo -e "[supervisord]\nredirect_stderr=true\nstopsignal=QUIT\nautorestart=true\ndirectory=/root\n\n[program:codeserver]\ndirectory=/home/ubuntu\ncommand=/usr/bin/code-server --auth none --bind-addr 0.0.0.0:8888\nuser=ubuntu\nenvironment=DISPLAY=:1,HOME=/home/ubuntu,USER=ubuntu" > /etc/supervisor/conf.d/codeserver.conf'
