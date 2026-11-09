FROM ros-noetic-rviz:noetic
COPY . /opt/ego-display
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cmake -S /opt/ego-display -B /tmp/ego-display-build -DCMAKE_BUILD_TYPE=Release && cmake --build /tmp/ego-display-build -j2 && rm -rf /tmp/ego-display-build'
RUN chmod +x /opt/ego-display/scripts/entrypoint.sh
ENTRYPOINT ["/opt/ego-display/scripts/entrypoint.sh"]
WORKDIR /home/ros
