## *********************************************************
##
## File autogenerated for the moveit_ros_manipulation package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 292, 'description': 'The maximum number of robot configurations to attempt at a particular eef pose', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'max_attempted_states_per_pose', 'edit_method': '', 'default': 5, 'level': 1, 'min': 1, 'type': 'int'}, {'srcline': 292, 'description': 'The maximum consecutive failures at generating configurations matching a pose before failure', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'max_consecutive_fail_attempts', 'edit_method': '', 'default': 3, 'level': 2, 'min': 1, 'type': 'int'}, {'srcline': 292, 'description': 'The distance (meters, for end-effector) between consecutive waypoints on Cartesian motions', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'cartesian_motion_step_size', 'edit_method': '', 'default': 0.02, 'level': 3, 'min': 0.005, 'type': 'double'}, {'srcline': 292, 'description': 'The maximum allowed distance in configuration space between consecutive waypoints on Cartesian motions', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'jump_factor', 'edit_method': '', 'default': 2.0, 'level': 4, 'min': 0.0, 'type': 'double'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']
