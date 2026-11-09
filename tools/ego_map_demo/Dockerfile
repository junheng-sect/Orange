FROM ego-swarm-rviz:noetic
COPY . /opt/ego-map-demo
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cmake -S /opt/ego-map-demo -B /tmp/map-demo-build -DCMAKE_BUILD_TYPE=Release && cmake --build /tmp/map-demo-build -j2 && rm -rf /tmp/map-demo-build'
