# simple_launch
This package provides a Python class to help writing ROS 2 launch files.

The motivation behind this package is that the ROS 2 launch Python syntax may not be suited for many simple cases such as running basic nodes, spawning a `robot_state_publisher`, and grouping nodes in namespaces or components..

**Thanks**: this package is inspired by [ros2_launch_util](https://github.com/bponsler/ros2_launch_util).

The entry point is the `SimpleLauncher` class, which has several capabilities.

## Basic syntax

### Namespace and argument parser initialization

`sl = SimpleLauncher(namespace)`

### Node registration

`sl.node(package, executable, **node_args)` where 

- `package` is the node package
- `executable` is the name of the executable
- `node_args` are any additional `Node` arguments given as a list

### Launch file include

`sl.include(package, launch_file, launch_dir = None, launch_arguments=None)` where

- `package` is the package of the included launch file
- `launch_file` is the name of the launch file
- `launch_dir` is its directory inside the package share (`None` to have it found)
- `launch_arguments` is a list of arguments to pass to the included launch file

### Robust types for parameters

In the launch API, differents types are expected for:

- node parameters: a list of dictionaries
- node remappings: a list of (`key`, `value`) pairs
- included launch arguments: a list of (`key`, `value`) pairs

The `sl.include` and `sl.node` syntax allow using any type (the simplest being a single dictionary)  and will convert to the one expected by the API.

## Launch arguments

The helper class allows declaring launch arguments and getting them in return:

### Declare a launch argument

`sl.declare_arg(name, default_value, description = None)`

### Retrieve a launch argument

`sl.arg(name)`: returns the argument `name`

### Retrieve several arguments as a dictionary

`sl.arg_map(('robot', 'x', 'y'))`: returns `{'robot': <robot arg value>, 'x': <x arg value>, 'y': <y arg value>}`

## Node groups

Groups are created through the `sl.with()` syntax and accepts both a namespace and/or an if/unless condition:

### By namespace

```
  with sl.group(ns=sub_namespace):
    sl.node(package, executable)
```

### From conditional arguments

```
  with sl.group(if_arg='use_gui'):
    sl.node(package, executable)
    
  with sl.group(unless_arg='dont_do'):
    sl.node(package, executable)
```

If `if_arg` / `unless_arg` is a string then the corresponding launch argument is used. Otherwise the raw value is passed as a condition (Boolean / Substitution).

### Creating containers

This syntax adds the `composition/composition::Talker` as a ComposableNode 

```
  with sl.container(name='my_container', output='screen'):
    sl.node(package='composition', plugin='Talker', name='talker')
```

## Simulation and `use_sim_time`

Calling `sl.auto_sim_time()` will detect (and return) if the `/clock` topic is advertized. (typically by a simulator).
If this is the case then all subsequent nodes will be run with `use_sim_time:=True`, unless this parameter is explicitely given.

### Spawn a model

The `sl.spawn_ign_model(name, topic, spawn_args = [])` functions allows easily spawing a model from its `robot_description`:

- `name` is the name this model will get in Ignition
- `topic` is the topic to obtain the model from, default is `robot_description`
- `spawn_args` are any additional spawn arguments, e.g. the initial pose

### Ignition world

The `IgnitionBridge` class has a few static methods to interact with a running ignition. Namely:

- `IgnitionBridge.world()` returns the current world name
- `IgnitionBridge.model_prefix(model)` builds the Ignition topic relative to the given model `/world/<world>/model`
- `IgnitionBridge.has_model(model)` returns `True` of `False` depending on the passed model being in Ignition

### Ignition bridge

The `IgnitionBridge` class allows easily creating bridges when running Ignition.

An instance is created with: `bridge = IgnitionBridge(<ignition_topic>, <ros_topic>, <ros_message>, direction)` where `direction` is either:

- `IgnitionBridge.ign2ros` for Ignition -> ROS
- `IgnitionBridge.ros2ign` for ROS -> Ignition
- `IgnitionBridge.bidirectional` for both

The Ignition message type is deduced from the ros message type. Remapping will be set to the given `ros_topic`.

The SimpleLauncher instance can then run all created bridges with: `sl.create_ign_bridge([bridges], <node_name>)`, as illustrated in the examples at this end of this document.

## Other shortcuts

### String / substitution concatenation

The following syntax builds `<robot name>.xacro`:

`file_name = sl.name_join(sl.arg('robot'), '.xacro')`

### Path concatenation

The following syntax builds `<my_package_path>/urdf/<robot name>.xacro`:

```
file_name = sl.name_join(sl.arg('robot'), '.xacro')
urdf_file = sl.path_join(get_package_share_directory(package), 'urdf', file_name)
```


### Find a share file

`path = sl.find(package, file_name, file_dir = None)` where:

- `package` is the name of the package or None
- `file_name` is the name of the file to find
- `file_dir` is the path inside the package

If `file_dir` is `None` then the `find` function will actually look for the file inside the package share, assuming that `package` and `file_name` are raw strings.

### Robot state publisher

`sl.robot_state_publisher(package, description_file, description_dir=None, xacro_args=None, prefix_gz_plugins=False, **node_args)` where

- `description_file` is a URDF or xacro file
- `description_dir` is the sub-directory of the file. If omitted, let the script search for the file assuming it is a raw string
- `xacro_args` are passed to xacro
- `prefix_gz_plugins` is used only if a `frame_prefix` parameter is given to `robot_state_publisher`. In this case it will forward the frame prefix to Gazebo-published messages that include frame names
- `node_args` are any additional arguments for `robot_state_publisher` (typically remapping)

### Joint state publisher

`sl.joint_state_publisher(use_gui, **node_args)`: fires up a `joint_state_publisher`, with or without the gui.

### Fallback to low-level syntax

If any unavailable functionality is needed, the `sl.entity(entity)` function adds any passed `Entity` at the current namespace / conditional / composition level.
        
        
## Examples

Examples can be found in the corresponding folder.


### Launch with arguments

Here is a file spawning a `robot_state_publisher` and its `joint_state_publisher`, where:

- `prefix`, `x` and `y` are launch arguments that are passed to xacro
- `use_gui` tells whether `joint_state_publisher` should run the gui

```
from simple_launch import SimpleLauncher

def generate_launch_description():
    '''
    Launch description for a single robot - runs the two nodes in their own namespace
    '''
    sl = SimpleLauncher()
        
    sl.declare_arg('prefix', default_value = '', description='name of the robot (+ tf prefix)')
    sl.declare_arg('x', default_value = 0, description='x-offset of the robot')
    sl.declare_arg('y', default_value = 0, description='y-offset of the robot')
    sl.declare_arg('use_gui', default_value = True, description='Use JSP gui')
    
    xacro_args = sl.arg_map(('prefix', 'x', 'y'))
    xacro_args['prefix'] = [xacro_args['prefix'], ':']                            
    
    with sl.group(ns=sl.arg('prefix')):
        sl.robot_state_publisher('simple_launch', 'turret.xacro', xacro_args = xacro_args)
        sl.joint_state_publisher(sources_list = ['source_joints'], use_gui = sl.arg('use_gui'))
        
    return sl.launch_description()
```

### Conditional blocks and includes

The file below fires up either `robot1` or `robot2` (or both) and also has a boolean argument to spawn `RViz2`:

```
from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    # conditional args
    sl.declare_arg('robot1', default_value=False, description='use robot 1')
    sl.declare_arg('robot2', default_value=True, description='use robot 2')
    sl.declare_arg('no_robot2', default_value=False, description='cancel use of robot 2')
    sl.declare_arg('rviz', default_value=False, description='Bringup RViz2')
    
    # numeric args
    sl.declare_arg('robot2_x', default_value=1, description='x-offset of robot 2')
    sl.declare_arg('robot2_y', default_value=1, description='y-offset of robot 2')
    
    with sl.group(if_arg='robot1'):
        sl.include('simple_launch', 'included_launch.py', launch_arguments = [('prefix', 'robot1')])
        
    with sl.group(if_arg='robot2'):
        with sl.group(unless_arg='no_robot2'):
            args = [('prefix', 'robot2'), ('x', sl.arg('robot2_x')), ('y', sl.arg('robot2_y'))]
            sl.include('simple_launch', 'included_launch.py', launch_arguments=args)
            
    with sl.group(if_arg='rviz'):
        rviz_config = sl.find('simple_launch', 'turret.rviz')
        sl.node('rviz2', 'rviz2', arguments = ['-d', rviz_config])
        
    return sl.launch_description()
```

### Composition

The file below is another way to write the [composition launch example](https://index.ros.org/doc/ros2/Tutorials/Composition/#composition-using-launch-actions):

```
from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    with sl.container(name='my_container', output='screen'):
        sl.node(package='composition', plugin='Talker', name='talker')
        sl.node(package='composition', plugin='Listener', name='listener')
        
    return sl.launch_description()
```

### Ignition bridge

The file below runs a bridge for joint states and pose, together with a standard clock bridge.
We assume the corresponding plugins do exist in the robot description.

```
from simple_launch import SimpleLauncher, IgnitionBridge

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    # namespace is a launch argument, not a Python string
    sl.declare_arg('robot', default_value = 'turtlebot')
    robot = sl.arg('robot')
    
    with sl.group(ns = robot):
        # assume some robot_state_publisher publishes the robot_description
    
        # spawn in Ignition at default pose if not already here
        # used IgnitionBridge.has_model(robot) under the hood and calls ros_ign_gazebo::create
        sl.spawn_ign_model(robot)

        # create a bridge for joint states @ /world/<world>/model/<robot>/joint_state
        # note the relative ROS topic 'joint_states' that is actually namespaced     
        ign_js_topic = sl.name_join(IgnitionBridge.model_prefix(robot), '/joint_state')
        js_bridge = IgnitionBridge(ign_js_topic, 'joint_states', 'sensor_msgs/JointState', IgnitionBridge.ign2ros)        

        # pose publisher bridge @ /model/<robot>
        pose_bridge = IgnitionBridge(sl.name_join('/model/', robot, '/pose'),
                                        'pose_gt', 'geometry_msgs/Pose', IgnitionBridge.ign2ros)

        # create bridge, also add standard /clock topic
        sl.create_ign_bridge([IgnitionBridge.clock(), js_bridge, pose_bridge], 'ign_bridge')
    
    return sl.launch_description()

```
