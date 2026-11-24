Source: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
Revision: 92fe9f7227b2da819133eb8e0e8c7fc000f6ae20
License: GPL-3.0 (upstream LICENSE)
Only six required packages imported. Local adaptations: standard body-frame odometry, true 3D goals, cancellation/status, persistent LiDAR occupancy with ray clearing, conservative unknown/footprint queries, input watchdogs, planning retries after more than `fsm/fail_fast_retries` (default 10) consecutive failed ticks limited by `fsm/fail_retry_interval` (default 0.1 s) and repeated optimizer/A* failure logs throttled to 1 Hz. Upstream demo launch files are reference only; use drone_navigation/navigation.launch.
Legacy generated Python message bindings and editor backup files from the upstream snapshot are omitted; catkin generates bindings from msg/ at build time.
