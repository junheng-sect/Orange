#!/usr/bin/env python3
import os
import rosgraph

master = rosgraph.Master('/ego_swarm_view/check_inputs')
publishers, _, _ = master.getSystemState()
pubs = dict(publishers)
print('ROS_MASTER_URI=' + os.environ['ROS_MASTER_URI'])
for name in ('/drone_nav/odom', '/cloud_registered', '/drone_nav/cloud',
             '/ego_swarm_view/saved_map',
             '/drone_0_ego_planner_node/grid_map/occupancy',
             '/drone_0_ego_planner_node/grid_map/occupancy_inflate',
             '/drone_0_ego_planner_node/optimal_list'):
    print(('PUBLISHER ' if pubs.get(name) else 'WAITING   ') + name)
own = sorted(name for name, nodes in publishers
             if any(node.startswith('/ego_swarm_view/') for node in nodes))
print('Display node publications: ' + ', '.join(own))
assert not any(name.startswith('/mavros/') for name in own), 'Unexpected MAVROS publisher'
